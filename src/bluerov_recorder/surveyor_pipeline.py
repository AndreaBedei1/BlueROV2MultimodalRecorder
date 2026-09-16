"""Bounded Surveyor decode, accounting, beamforming, and preview pipeline."""

from __future__ import annotations

import queue
import threading
import time
from collections import OrderedDict
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Set, Tuple

from .processing import (
    MSG_ATOF,
    MSG_ATTITUDE,
    MSG_END_PING,
    MSG_RAW_PROFILE,
    SURVEYOR_PACKET_COUNT,
    beamform_surveyor_channels,
    decode_atof_payload,
    decode_attitude_payload,
    decode_end_ping_payload,
    parse_channel_data_header,
    validate_channel_payloads,
)
from .runtime import (
    LatestValueMailbox, MetricsRegistry, RecordingBackpressureError,
    publish_control_event,
)


def _device_timestamp_ns(value: int) -> Optional[int]:
    value = int(value or 0)
    if value <= 0:
        return None
    return value if value > 10_000_000_000_000 else value * 1_000_000


@dataclass
class PingAssembly:
    ping_number: int
    first_host_monotonic_ns: int
    last_host_monotonic_ns: int
    first_host_utc_ns: int
    last_host_utc_ns: int
    channel_payloads: List[bytes] = field(default_factory=list)
    channel_pairs: Set[Tuple[int, int]] = field(default_factory=set)
    end_data: Optional[Dict[str, Any]] = None
    atof_data: Optional[Dict[str, Any]] = None
    attitude: Optional[Dict[str, Any]] = None

    def touch(self, monotonic_ns: int, utc_ns: int) -> None:
        self.last_host_monotonic_ns = int(monotonic_ns)
        self.last_host_utc_ns = int(utc_ns)


class SurveyorBeamformingWorker(threading.Thread):
    """Compute at most one fan matrix per complete ping off the GUI thread."""

    def __init__(
        self,
        preview_mailbox: LatestValueMailbox,
        metrics: MetricsRegistry,
        control_events: queue.Queue,
        max_pending_pings: int = 128,
    ) -> None:
        super().__init__(name="surveyor-beamforming", daemon=False)
        self.preview_mailbox = preview_mailbox
        self.metrics = metrics
        self.control_events = control_events
        self.queue: queue.Queue = queue.Queue(maxsize=int(max_pending_pings))
        self._closing = threading.Event()

    def submit(self, record: Dict[str, Any]) -> None:
        try:
            self.queue.put(record, timeout=1.0)
        except queue.Full:
            # Derived preview work may be omitted under overload; raw and
            # decoded records have already crossed their persistence boundary.
            self.metrics.increment("surveyor_beamforming_skipped")
            publish_control_event(
                self.control_events,
                ("performance_warning", "Surveyor beamforming backlog; preview work skipped"),
                self.metrics,
            )
            return
        depth = self.queue.qsize()
        self.metrics.set("surveyor_beam_queue_depth", depth)
        self.metrics.maximum("surveyor_beam_queue_high_watermark", depth)

    def run(self) -> None:
        while not self._closing.is_set() or not self.queue.empty():
            try:
                record = self.queue.get(timeout=0.1)
            except queue.Empty:
                continue
            started = time.perf_counter_ns()
            try:
                record["matrix"] = beamform_surveyor_channels(
                    record["channel_signals"],
                    record.get("range_start_m", 0.0),
                    record.get("range_end_m", 10.0),
                    record.get("sos_mps", 1500.0),
                )
                self.metrics.increment("surveyor_pings_beamformed")
                self.metrics.set(
                    "surveyor_beamforming_ms",
                    (time.perf_counter_ns() - started) / 1_000_000.0,
                )
                self.preview_mailbox.publish(record)
            except Exception as exc:
                self.metrics.increment("surveyor_beamforming_errors")
                publish_control_event(
                    self.control_events,
                    ("surveyor_processing_error", str(exc)),
                    self.metrics,
                )
            finally:
                self.metrics.set("surveyor_beam_queue_depth", self.queue.qsize())

    def close(self, timeout: float = 10.0) -> None:
        self._closing.set()
        self.join(timeout)
        if self.is_alive():
            raise RuntimeError("Surveyor beamforming worker did not terminate")


class SurveyorDecoderWorker(threading.Thread):
    """Decode framed packets without blocking socket acquisition or the GUI."""

    def __init__(
        self,
        preview_mailbox: LatestValueMailbox,
        attitude_mailbox: LatestValueMailbox,
        control_events: queue.Queue,
        metrics: MetricsRegistry,
        decoded_callback: Optional[Callable[[Dict[str, Any]], None]] = None,
        configured_ping_rate_hz: Optional[float] = None,
        max_pending_packets: int = 8192,
    ) -> None:
        super().__init__(name="surveyor-decoder", daemon=False)
        self.preview_mailbox = preview_mailbox
        self.attitude_mailbox = attitude_mailbox
        self.control_events = control_events
        self.metrics = metrics
        self.decoded_callback = decoded_callback
        self.configured_ping_rate_hz = configured_ping_rate_hz
        self.queue: queue.Queue = queue.Queue(maxsize=int(max_pending_packets))
        self._closing = threading.Event()
        self._states: "OrderedDict[int, PingAssembly]" = OrderedDict()
        self._latest_attitude: Optional[Dict[str, Any]] = None
        self._last_finalized_ping: Optional[int] = None
        self._seen_ping_numbers: Set[int] = set()
        self.beamformer = SurveyorBeamformingWorker(
            preview_mailbox, metrics, control_events,
        )

    def start(self) -> None:
        self.beamformer.start()
        super().start()

    def submit(self, packet: Tuple[int, bytes, bytes, int, int]) -> None:
        try:
            self.queue.put(packet, timeout=2.0)
        except queue.Full as exc:
            self.metrics.increment("surveyor_decoder_app_drop")
            raise RuntimeError("CRITICAL DATA LOSS: Surveyor decoder queue full") from exc
        depth = self.queue.qsize()
        self.metrics.set("surveyor_decoder_queue_depth", depth)
        self.metrics.maximum("surveyor_decoder_queue_high_watermark", depth)

    def _state(self, ping_number: int, monotonic_ns: int, utc_ns: int) -> PingAssembly:
        state = self._states.get(int(ping_number))
        if state is None:
            state = PingAssembly(
                int(ping_number), int(monotonic_ns), int(monotonic_ns),
                int(utc_ns), int(utc_ns), attitude=self._latest_attitude,
            )
            self._states[int(ping_number)] = state
            self._seen_ping_numbers.add(int(ping_number))
            low = min(self._seen_ping_numbers)
            high = max(self._seen_ping_numbers)
            self.metrics.set(
                "surveyor_expected_ping_gap",
                high - low + 1 - len(self._seen_ping_numbers),
            )
        else:
            state.touch(monotonic_ns, utc_ns)
        return state

    def _build_record(self, state: PingAssembly) -> Dict[str, Any]:
        valid, note, channels, bins = validate_channel_payloads(
            state.channel_payloads, state.ping_number,
        )
        end_data = state.end_data or {}
        atof_data = state.atof_data or {}
        points = list(atof_data.get("points", []))
        acoustic_frequency = float(
            atof_data.get("ping_hz", end_data.get("ping_hz", 0.0)) or 0.0
        )
        record: Dict[str, Any] = {
            "ping_number": state.ping_number,
            "host_monotonic_ns": state.last_host_monotonic_ns,
            "host_utc_ns": state.last_host_utc_ns,
            "device_timestamp_ns": _device_timestamp_ns(
                int(atof_data.get("utc_msec", 0))
            ) or _device_timestamp_ns(int(end_data.get("timestamp", 0))),
            "range_start_m": float(end_data.get("start_m", 0.0)),
            "range_end_m": float(end_data.get("end_m", 0.0)),
            "sos_mps": float(atof_data.get("sos_mps", 1500.0) or 1500.0),
            "ping_rate_hz": self.configured_ping_rate_hz,
            "acoustic_frequency_hz": acoustic_frequency,
            "points": points,
            "detection_count": len(points),
            "channel_data_status": "AVAILABLE" if valid else "NOT AVAILABLE",
            "channel_packets_received": len(state.channel_payloads),
            "expected_channel_packets": SURVEYOR_PACKET_COUNT,
            "channel_pairs_received": [list(pair) for pair in sorted(state.channel_pairs)],
            "complete_16_channels": bool(valid),
            "has_end_ping": state.end_data is not None,
            "has_atof": state.atof_data is not None,
            "has_attitude": state.attitude is not None,
            "first_host_monotonic_ns": state.first_host_monotonic_ns,
            "last_host_monotonic_ns": state.last_host_monotonic_ns,
            "channel_data_note": note,
            "attitude": state.attitude,
        }
        if valid:
            record["bins"] = int(bins)
            record["channel_signals"] = channels
        return record

    def _finalize(self, ping_number: int) -> None:
        state = self._states.pop(int(ping_number), None)
        if state is None:
            return
        record = self._build_record(state)
        self._last_finalized_ping = max(self._last_finalized_ping or ping_number, ping_number)
        self.metrics.increment("surveyor_ping_records")
        if record["complete_16_channels"]:
            self.metrics.increment("surveyor_complete_channel_pings")
        else:
            self.metrics.increment("surveyor_incomplete_channel_pings")
        if not record["has_end_ping"]:
            self.metrics.increment("surveyor_missing_end_ping")
        if not record["has_atof"]:
            self.metrics.increment("surveyor_missing_atof")
        if self.decoded_callback is not None:
            self.decoded_callback(record)
        if record["complete_16_channels"]:
            self.beamformer.submit(record)
        else:
            self.preview_mailbox.publish(record)

    def _evict_old(self, newest_ping: int) -> None:
        # Channel packets from ping N+1 can precede END_PING/ATOF for ping N.
        # Keep a short reorder window and finalize complete metadata as soon as
        # both terminal messages have arrived.
        for ping_number in list(self._states):
            if ping_number < newest_ping - 2:
                self._finalize(ping_number)
        while len(self._states) > 256:
            self._finalize(next(iter(self._states)))

    def _process(self, packet: Tuple[int, bytes, bytes, int, int]) -> None:
        message_id, payload, _raw_packet, monotonic_ns, utc_ns = packet
        self.metrics.increment("surveyor_packets_decoded")
        if message_id == MSG_ATTITUDE:
            attitude = decode_attitude_payload(payload)
            if attitude:
                self._latest_attitude = attitude
                self.attitude_mailbox.publish(attitude)
            return
        ping_number: Optional[int] = None
        if message_id == MSG_RAW_PROFILE:
            header = parse_channel_data_header(payload)
            if header:
                ping_number = int(header["ping_number"])
                self._evict_old(ping_number)
                state = self._state(ping_number, monotonic_ns, utc_ns)
                state.channel_payloads.append(bytes(payload))
                state.channel_pairs.add((int(header["ch1"]), int(header["ch2"])))
                self.metrics.increment("surveyor_channel_packets_received")
        elif message_id == MSG_ATOF:
            data = decode_atof_payload(payload)
            if data:
                ping_number = int(data["ping_number"])
                self._evict_old(ping_number)
                state = self._state(ping_number, monotonic_ns, utc_ns)
                state.atof_data = data
                if state.end_data is not None:
                    self._finalize(ping_number)
        elif message_id == MSG_END_PING:
            data = decode_end_ping_payload(payload)
            if data:
                ping_number = int(data["ping_number"])
                self._evict_old(ping_number)
                state = self._state(ping_number, monotonic_ns, utc_ns)
                state.end_data = data
        else:
            self.metrics.increment("surveyor_unknown_message_count")

    def run(self) -> None:
        while not self._closing.is_set() or not self.queue.empty():
            try:
                packet = self.queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                self._process(packet)
            except RecordingBackpressureError as exc:
                self.metrics.increment("surveyor_decoder_recording_errors")
                publish_control_event(
                    self.control_events,
                    ("critical_data_loss", str(exc)), self.metrics,
                    critical=True,
                )
            except Exception as exc:
                self.metrics.increment("surveyor_decoder_errors")
                publish_control_event(
                    self.control_events,
                    ("surveyor_processing_error", str(exc)),
                    self.metrics,
                )
            finally:
                self.metrics.set("surveyor_decoder_queue_depth", self.queue.qsize())
        for ping_number in list(self._states):
            self._finalize(ping_number)

    def close(self, timeout: float = 10.0) -> None:
        self._closing.set()
        self.join(timeout)
        if self.is_alive():
            raise RuntimeError("Surveyor decoder worker did not terminate")
        self.beamformer.close(timeout)
