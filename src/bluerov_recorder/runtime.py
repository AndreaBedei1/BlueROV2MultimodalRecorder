"""Bounded cross-thread primitives and asynchronous session writers.

The live recorder has three deliberately different communication classes:

* small control events use a bounded :class:`queue.Queue`;
* lossy display previews use :class:`LatestValueMailbox`;
* loss-intolerant recording data use bounded asynchronous writers.

Preview replacement is expected and counted.  A recording queue timeout is a
critical application-side data loss condition and is never hidden.
"""

from __future__ import annotations

import csv
import json
import os
import queue
import threading
import time
from collections import Counter, deque
from pathlib import Path
from typing import Any, Deque, Dict, Iterable, List, Optional, Sequence, Tuple


class RecordingBackpressureError(RuntimeError):
    """Raised when received raw data cannot enter a bounded writer queue."""


class MetricsRegistry:
    """Small thread-safe counter/gauge registry shared by live components."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._counters: Counter = Counter()
        self._gauges: Dict[str, float] = {}

    def increment(self, name: str, value: float = 1.0) -> float:
        with self._lock:
            self._counters[str(name)] += value
            return float(self._counters[str(name)])

    def set(self, name: str, value: float) -> None:
        with self._lock:
            self._gauges[str(name)] = value

    def maximum(self, name: str, value: float) -> float:
        with self._lock:
            key = str(name)
            current = float(self._gauges.get(key, 0.0))
            if value > current:
                self._gauges[key] = value
                current = value
            return current

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            values: Dict[str, Any] = dict(self._counters)
            values.update(self._gauges)
        values["metrics_monotonic_ns"] = time.monotonic_ns()
        return values


class LatestValueMailbox:
    """A capacity-one mailbox for disposable display state.

    ``publish`` replaces an unread value instead of allocating backlog.  The
    replacement is a preview drop, not acquisition or recording data loss.
    """

    def __init__(
        self,
        metrics: Optional[MetricsRegistry] = None,
        dropped_metric: Optional[str] = None,
        published_metric: Optional[str] = None,
    ) -> None:
        self._lock = threading.Lock()
        self._value: Any = None
        self._sequence = 0
        self._has_value = False
        self.metrics = metrics
        self.dropped_metric = dropped_metric
        self.published_metric = published_metric

    def publish(self, value: Any) -> int:
        with self._lock:
            replaced = self._has_value
            self._value = value
            self._has_value = True
            self._sequence += 1
            sequence = self._sequence
        if self.metrics is not None:
            if self.published_metric:
                self.metrics.increment(self.published_metric)
            if replaced and self.dropped_metric:
                self.metrics.increment(self.dropped_metric)
        return sequence

    def take(self) -> Optional[Tuple[int, Any]]:
        with self._lock:
            if not self._has_value:
                return None
            value = self._value
            sequence = self._sequence
            self._value = None
            self._has_value = False
            return sequence, value

    def peek(self) -> Optional[Tuple[int, Any]]:
        with self._lock:
            if not self._has_value:
                return None
            return self._sequence, self._value

    def __len__(self) -> int:
        with self._lock:
            return 1 if self._has_value else 0


def publish_control_event(
    events: queue.Queue,
    item: Tuple[str, Any],
    metrics: Optional[MetricsRegistry] = None,
    critical: bool = False,
) -> bool:
    """Publish a small control event without ever building unlimited backlog."""
    try:
        events.put(item, timeout=1.0 if critical else 0.0)
        if metrics is not None:
            metrics.set("control_queue_depth", events.qsize())
            metrics.maximum("control_queue_high_watermark", events.qsize())
        return True
    except queue.Full:
        if metrics is not None:
            metrics.increment("control_events_dropped")
        return False


class ByteBoundedQueue:
    """FIFO bounded by both bytes and item count with batch retrieval."""

    def __init__(self, max_bytes: int, max_items: int = 65536) -> None:
        if max_bytes <= 0 or max_items <= 0:
            raise ValueError("queue bounds must be positive")
        self.max_bytes = int(max_bytes)
        self.max_items = int(max_items)
        self._items: Deque[Tuple[Any, int, int]] = deque()
        self._bytes = 0
        self._closed = False
        self._condition = threading.Condition()

    def put(self, item: Any, size: int, timeout: Optional[float] = None) -> Tuple[int, int]:
        size = int(size)
        if size < 0 or size > self.max_bytes:
            raise ValueError("item size exceeds queue capacity")
        deadline = None if timeout is None else time.monotonic() + max(0.0, timeout)
        with self._condition:
            while not self._closed and (
                self._bytes + size > self.max_bytes or len(self._items) >= self.max_items
            ):
                remaining = None if deadline is None else deadline - time.monotonic()
                if remaining is not None and remaining <= 0:
                    raise queue.Full
                self._condition.wait(remaining)
            if self._closed:
                raise RuntimeError("queue is closed")
            self._items.append((item, size, time.monotonic_ns()))
            self._bytes += size
            depth = len(self._items)
            byte_depth = self._bytes
            self._condition.notify_all()
            return depth, byte_depth

    def get_batch(
        self,
        max_bytes: int,
        max_items: int,
        timeout: Optional[float] = None,
    ) -> List[Tuple[Any, int, int]]:
        deadline = None if timeout is None else time.monotonic() + max(0.0, timeout)
        with self._condition:
            while not self._items and not self._closed:
                remaining = None if deadline is None else deadline - time.monotonic()
                if remaining is not None and remaining <= 0:
                    return []
                self._condition.wait(remaining)
            if not self._items:
                return []
            batch: List[Tuple[Any, int, int]] = []
            total = 0
            while self._items and len(batch) < max_items:
                item, size, queued_ns = self._items[0]
                if batch and total + size > max_bytes:
                    break
                self._items.popleft()
                self._bytes -= size
                batch.append((item, size, queued_ns))
                total += size
            self._condition.notify_all()
            return batch

    def close_input(self) -> None:
        with self._condition:
            self._closed = True
            self._condition.notify_all()

    @property
    def closed(self) -> bool:
        with self._condition:
            return self._closed

    @property
    def depth(self) -> int:
        with self._condition:
            return len(self._items)

    @property
    def byte_depth(self) -> int:
        with self._condition:
            return self._bytes


class BufferedBinaryWriter:
    """Loss-intolerant ordered binary writer with bounded byte buffering."""

    def __init__(
        self,
        path: Path,
        metrics: MetricsRegistry,
        metric_prefix: str,
        max_queue_bytes: int = 32 * 1024 * 1024,
        batch_bytes: int = 512 * 1024,
        flush_interval_s: float = 0.25,
        submit_timeout_s: float = 2.0,
        initial_bytes: bytes = b"",
    ) -> None:
        self.path = Path(path)
        self.metrics = metrics
        self.prefix = str(metric_prefix)
        self.queue = ByteBoundedQueue(max_queue_bytes)
        self.batch_bytes = int(batch_bytes)
        self.flush_interval_s = float(flush_interval_s)
        self.submit_timeout_s = float(submit_timeout_s)
        self.initial_bytes = bytes(initial_bytes)
        self.error: Optional[BaseException] = None
        self._thread = threading.Thread(target=self._run, name=f"{self.prefix}-writer", daemon=False)
        self._started = False

    def start(self) -> None:
        if not self._started:
            self._started = True
            self._thread.start()

    def submit(self, data: bytes, units: int = 1) -> None:
        payload = bytes(data)
        if self.error is not None:
            self._record_drop_and_raise(f"writer failed: {self.error}")
        try:
            depth, byte_depth = self.queue.put((payload, int(units)), len(payload), self.submit_timeout_s)
        except (queue.Full, RuntimeError) as exc:
            self._record_drop_and_raise(f"recording queue unavailable: {exc}")
        self.metrics.increment(f"{self.prefix}_packets_received", int(units))
        self.metrics.increment(f"{self.prefix}_bytes_received", len(payload))
        self.metrics.set(f"{self.prefix}_queue_depth", depth)
        self.metrics.set(f"{self.prefix}_queue_bytes", byte_depth)
        self.metrics.maximum(f"{self.prefix}_queue_high_watermark", depth)
        self.metrics.maximum(f"{self.prefix}_queue_bytes_high_watermark", byte_depth)
        if self.prefix == "surveyor_raw":
            self.metrics.set("surveyor_raw_queue_depth", depth)
            self.metrics.set("surveyor_raw_queue_bytes", byte_depth)
            self.metrics.maximum("surveyor_raw_queue_high_watermark", depth)

    def _record_drop_and_raise(self, reason: str) -> None:
        self.metrics.increment(f"{self.prefix}_app_raw_drop")
        self.metrics.increment(f"{self.prefix}_writer_errors")
        if self.prefix == "surveyor_raw":
            self.metrics.increment("app_raw_drop_count")
        raise RecordingBackpressureError(f"CRITICAL DATA LOSS: {self.prefix}: {reason}")

    def _run(self) -> None:
        last_flush = time.monotonic()
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with self.path.open("wb", buffering=1024 * 1024) as stream:
                if self.initial_bytes:
                    stream.write(self.initial_bytes)
                while True:
                    batch = self.queue.get_batch(self.batch_bytes, 4096, self.flush_interval_s)
                    if batch:
                        stream.write(b"".join(item[0] for item, _size, _queued_ns in batch))
                        now_ns = time.monotonic_ns()
                        for item, size, queued_ns in batch:
                            self.metrics.increment(f"{self.prefix}_packets_written", item[1])
                            self.metrics.increment(f"{self.prefix}_bytes_written", size)
                            self.metrics.set(f"{self.prefix}_writer_latency_ms", (now_ns - queued_ns) / 1_000_000.0)
                            if self.prefix == "surveyor_raw":
                                self.metrics.increment("surveyor_packets_written", item[1])
                                self.metrics.increment("surveyor_bytes_written", size)
                                self.metrics.set("surveyor_writer_latency_ms", (now_ns - queued_ns) / 1_000_000.0)
                    now = time.monotonic()
                    if now - last_flush >= self.flush_interval_s:
                        stream.flush()
                        last_flush = now
                    self.metrics.set(f"{self.prefix}_queue_depth", self.queue.depth)
                    self.metrics.set(f"{self.prefix}_queue_bytes", self.queue.byte_depth)
                    if self.prefix == "surveyor_raw":
                        self.metrics.set("surveyor_raw_queue_depth", self.queue.depth)
                        self.metrics.set("surveyor_raw_queue_bytes", self.queue.byte_depth)
                    if self.queue.closed and self.queue.depth == 0:
                        stream.flush()
                        os.fsync(stream.fileno())
                        break
        except BaseException as exc:
            self.error = exc
            self.metrics.increment(f"{self.prefix}_writer_errors")

    def close(self, timeout: float = 10.0) -> None:
        if not self._started:
            return
        self.queue.close_input()
        self._thread.join(timeout)
        if self._thread.is_alive():
            self.metrics.increment(f"{self.prefix}_writer_errors")
            raise RuntimeError(f"{self.prefix} writer did not terminate")
        if self.error is not None:
            raise RuntimeError(f"{self.prefix} writer failed: {self.error}")


class BufferedJsonlWriter:
    """Bounded JSONL writer; serialization happens on its writer thread."""

    def __init__(
        self,
        path: Path,
        metrics: MetricsRegistry,
        metric_prefix: str,
        max_items: int = 8192,
        flush_interval_s: float = 0.5,
        submit_timeout_s: float = 2.0,
    ) -> None:
        self.path = Path(path)
        self.metrics = metrics
        self.prefix = str(metric_prefix)
        self.queue: queue.Queue = queue.Queue(maxsize=int(max_items))
        self.flush_interval_s = float(flush_interval_s)
        self.submit_timeout_s = float(submit_timeout_s)
        self._closing = threading.Event()
        self.error: Optional[BaseException] = None
        self._thread = threading.Thread(target=self._run, name=f"{self.prefix}-writer", daemon=False)
        self._thread.start()

    def submit(self, item: Dict[str, Any]) -> None:
        if self.error is not None:
            raise RecordingBackpressureError(f"CRITICAL DATA LOSS: {self.prefix}: {self.error}")
        try:
            self.queue.put(dict(item), timeout=self.submit_timeout_s)
        except queue.Full as exc:
            self.metrics.increment(f"{self.prefix}_app_drop")
            raise RecordingBackpressureError(f"CRITICAL DATA LOSS: {self.prefix} queue full") from exc
        depth = self.queue.qsize()
        self.metrics.set(f"{self.prefix}_queue_depth", depth)
        self.metrics.maximum(f"{self.prefix}_queue_high_watermark", depth)

    def _run(self) -> None:
        last_flush = time.monotonic()
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with self.path.open("w", encoding="utf-8", buffering=1024 * 1024) as stream:
                while not self._closing.is_set() or not self.queue.empty():
                    try:
                        item = self.queue.get(timeout=self.flush_interval_s)
                    except queue.Empty:
                        item = None
                    if item is not None:
                        stream.write(json.dumps(item, ensure_ascii=False) + "\n")
                        self.metrics.increment(f"{self.prefix}_records_written")
                    now = time.monotonic()
                    if now - last_flush >= self.flush_interval_s:
                        stream.flush()
                        last_flush = now
                    self.metrics.set(f"{self.prefix}_queue_depth", self.queue.qsize())
                stream.flush()
                os.fsync(stream.fileno())
        except BaseException as exc:
            self.error = exc
            self.metrics.increment(f"{self.prefix}_writer_errors")

    def close(self, timeout: float = 10.0) -> None:
        self._closing.set()
        self._thread.join(timeout)
        if self._thread.is_alive():
            raise RuntimeError(f"{self.prefix} writer did not terminate")
        if self.error is not None:
            raise RuntimeError(f"{self.prefix} writer failed: {self.error}")


class BufferedCsvWriter:
    """Bounded asynchronous CSV writer with a stable legacy header."""

    def __init__(
        self,
        path: Path,
        header: Sequence[str],
        metrics: MetricsRegistry,
        metric_prefix: str,
        max_items: int = 8192,
        flush_interval_s: float = 0.5,
    ) -> None:
        self.path = Path(path)
        self.header = list(header)
        self.metrics = metrics
        self.prefix = str(metric_prefix)
        self.queue: queue.Queue = queue.Queue(maxsize=int(max_items))
        self.flush_interval_s = float(flush_interval_s)
        self._closing = threading.Event()
        self.error: Optional[BaseException] = None
        self._thread = threading.Thread(target=self._run, name=f"{self.prefix}-writer", daemon=False)
        self._thread.start()

    def submit(self, row: Sequence[Any]) -> None:
        try:
            self.queue.put(tuple(row), timeout=2.0)
        except queue.Full as exc:
            self.metrics.increment(f"{self.prefix}_app_drop")
            raise RecordingBackpressureError(f"CRITICAL DATA LOSS: {self.prefix} queue full") from exc
        depth = self.queue.qsize()
        self.metrics.set(f"{self.prefix}_queue_depth", depth)
        self.metrics.maximum(f"{self.prefix}_queue_high_watermark", depth)

    def _run(self) -> None:
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with self.path.open("w", newline="", encoding="utf-8", buffering=1024 * 1024) as stream:
                writer = csv.writer(stream)
                writer.writerow(self.header)
                last_flush = time.monotonic()
                while not self._closing.is_set() or not self.queue.empty():
                    try:
                        row = self.queue.get(timeout=self.flush_interval_s)
                    except queue.Empty:
                        row = None
                    if row is not None:
                        writer.writerow(row)
                        self.metrics.increment(f"{self.prefix}_records_written")
                    now = time.monotonic()
                    if now - last_flush >= self.flush_interval_s:
                        stream.flush()
                        last_flush = now
                    self.metrics.set(f"{self.prefix}_queue_depth", self.queue.qsize())
                stream.flush()
                os.fsync(stream.fileno())
        except BaseException as exc:
            self.error = exc
            self.metrics.increment(f"{self.prefix}_writer_errors")

    def close(self, timeout: float = 10.0) -> None:
        self._closing.set()
        self._thread.join(timeout)
        if self._thread.is_alive():
            raise RuntimeError(f"{self.prefix} writer did not terminate")
        if self.error is not None:
            raise RuntimeError(f"{self.prefix} writer failed: {self.error}")
