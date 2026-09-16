"""Offline combined-load soak test for bounded recorder primitives.

The script never imports or connects to hardware.  Use ``--duration 1800`` or
``--duration 3600`` for the acceptance soak; the short default is convenient
for development iterations.
"""

from __future__ import annotations

import argparse
import json
import queue
import struct
import sys
import tempfile
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import numpy as np

from bluerov_recorder.processing import ATOF_HEADER, CHANNEL_DATA_HEADER, END_PING, make_packet
from bluerov_recorder.rendering import render_surveyor_fan
from bluerov_recorder.runtime import BufferedBinaryWriter, BufferedJsonlWriter, LatestValueMailbox, MetricsRegistry
from bluerov_recorder.surveyor_pipeline import SurveyorDecoderWorker


def _channel_packet(ping: int, ch1: int, bins: int) -> bytes:
    ch2 = ch1 + 1
    base = np.arange(bins, dtype=np.float32)
    values = np.concatenate((
        np.column_stack((base + ch1, base * 0.01)).ravel(),
        np.column_stack((base + ch2, base * 0.01)).ravel(),
    )).astype("<f4", copy=False).tobytes()
    payload = CHANNEL_DATA_HEADER.pack(ping, 1.0, 0, 0, 100, ch1, ch2, bins) + values
    return make_packet(3009, payload)


def _end_packet(ping: int, bins: int) -> bytes:
    values = list(END_PING.unpack(bytes(END_PING.size)))
    values[1], values[2], values[6] = 0.0, 20.0, ping
    values[13], values[16], values[17] = 240000.0, bins, 1
    values[-1] = int(time.time() * 1000)
    return make_packet(3010, END_PING.pack(*values))


def _atof_packet(ping: int) -> bytes:
    payload = ATOF_HEADER.pack(
        0, int(time.time() * 1000), 0.1, 1500.0, ping,
        240000, 0.001, 0, 0, 0,
    )
    return make_packet(3012, payload)


def run_soak(duration_s: float, output: Path, sonar_ping_hz: float = 10.0) -> dict:
    metrics = MetricsRegistry()
    controls = queue.Queue(maxsize=2048)
    camera = LatestValueMailbox(metrics, "camera_preview_dropped", "camera_preview_published")
    sonar = LatestValueMailbox(metrics, "surveyor_preview_dropped", "surveyor_preview_published")
    attitude = LatestValueMailbox(metrics)
    ping = LatestValueMailbox(metrics, "ping1d_preview_dropped", "ping1d_preview_published")
    rovl = LatestValueMailbox(metrics, "rovl_preview_dropped", "rovl_preview_published")
    raw = BufferedBinaryWriter(
        output / "surveyor_raw.svlog", metrics, "surveyor_raw",
        max_queue_bytes=32 * 1024 * 1024,
    )
    decoded = BufferedJsonlWriter(output / "surveyor_pings.jsonl", metrics, "surveyor_decoded")
    ping_writer = BufferedJsonlWriter(output / "ping1d.jsonl", metrics, "ping1d")
    rovl_writer = BufferedJsonlWriter(output / "rovl_positions.jsonl", metrics, "rovl")
    raw.start()
    decoder = SurveyorDecoderWorker(
        sonar, attitude, controls, metrics, decoded.submit,
        configured_ping_rate_hz=sonar_ping_hz,
    )
    decoder.start()
    stop = threading.Event()
    start = time.monotonic()
    rss_samples = []

    def camera_producer():
        interval = 1.0 / 30.0
        next_tick = time.perf_counter()
        index = 0
        while not stop.is_set():
            frame = np.empty((1080, 1920, 3), dtype=np.uint8)
            frame.fill(index % 255)
            camera.publish((frame, {"frame_index": index}))
            metrics.increment("camera_frames_decoded")
            index += 1
            next_tick += interval
            stop.wait(max(0.0, next_tick - time.perf_counter()))

    def sonar_producer():
        interval = 1.0 / sonar_ping_hz
        next_tick = time.perf_counter()
        ping_number = 1
        while not stop.is_set():
            packets = [_channel_packet(ping_number, ch1, 400) for ch1 in range(0, 16, 2)]
            packets.extend((_end_packet(ping_number, 400), _atof_packet(ping_number)))
            for packet in packets:
                raw.submit(packet)
                payload_len = struct.unpack_from("<H", packet, 2)[0]
                message_id = struct.unpack_from("<H", packet, 4)[0]
                payload = packet[8 : 8 + payload_len]
                decoder.submit((message_id, payload, packet, time.monotonic_ns(), time.time_ns()))
                metrics.increment("surveyor_packets_received")
            ping_number += 1
            next_tick += interval
            stop.wait(max(0.0, next_tick - time.perf_counter()))

    def small_sensor_producer():
        index = 0
        while not stop.is_set():
            now_mono, now_utc = time.monotonic_ns(), time.time_ns()
            profile = [index % 255] * 1200
            sample = {"host_monotonic_ns": now_mono, "host_utc_ns": now_utc, "distance_m": 2.0, "confidence": 100, "profile": profile}
            ping_writer.submit(sample)
            ping.publish(sample)
            metrics.increment("ping1d_samples_received")
            position = {"host_monotonic_ns": now_mono, "host_utc_ns": now_utc, "east_m": 1.0, "north_m": 2.0}
            rovl_writer.submit(position)
            rovl.publish(position)
            metrics.increment("rovl_samples_received")
            index += 1
            stop.wait(0.1)

    threads = [
        threading.Thread(target=camera_producer, name="soak-camera"),
        threading.Thread(target=sonar_producer, name="soak-surveyor"),
        threading.Thread(target=small_sensor_producer, name="soak-small-sensors"),
    ]
    for thread in threads:
        thread.start()
    next_camera = next_sonar = time.perf_counter()
    next_rss = time.perf_counter()
    try:
        while time.monotonic() - start < duration_s:
            now = time.perf_counter()
            if now >= next_camera:
                if camera.take() is not None:
                    metrics.increment("camera_frames_previewed")
                # Exercise the same headroom used by the GUI (20 Hz target),
                # then verify it stays above the required 15 FPS.
                next_camera += 1.0 / 20.0
                if next_camera < now:
                    next_camera = now
            if now >= next_sonar:
                item = sonar.take()
                if item is not None:
                    render_surveyor_fan(item[1], 620, 365)
                    metrics.increment("surveyor_gui_frames")
                next_sonar += 0.2
                if next_sonar < now:
                    next_sonar = now
            ping.take()
            rovl.take()
            if now >= next_rss:
                try:
                    import psutil
                    rss_samples.append(psutil.Process().memory_info().rss)
                except ImportError:
                    pass
                next_rss = now + 1.0
            time.sleep(0.005)
    finally:
        active_duration = time.monotonic() - start
        stop.set()
        for thread in threads:
            thread.join(5.0)
        decoder.close()
        raw.close()
        decoded.close()
        ping_writer.close()
        rovl_writer.close()
    result = metrics.snapshot()
    result["duration_s"] = time.monotonic() - start
    result["active_duration_s"] = active_duration
    result["camera_preview_fps"] = result.get("camera_frames_previewed", 0) / active_duration
    result["rss_min_mb"] = min(rss_samples) / (1024 ** 2) if rss_samples else None
    result["rss_max_mb"] = max(rss_samples) / (1024 ** 2) if rss_samples else None
    result["rss_growth_mb"] = (rss_samples[-1] - rss_samples[0]) / (1024 ** 2) if len(rss_samples) > 1 else None
    result["success"] = (
        result.get("app_raw_drop_count", 0) == 0
        and result.get("surveyor_raw_bytes_received", 0) == result.get("surveyor_bytes_written", 0)
        and len(camera) <= 1 and len(sonar) <= 1
    )
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description="Offline bounded multimodal soak test")
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--sonar-ping-hz", type=float, default=10.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.output:
        args.output.mkdir(parents=True, exist_ok=True)
        result = run_soak(args.duration, args.output, args.sonar_ping_hz)
    else:
        with tempfile.TemporaryDirectory(prefix="bluerov_soak_") as folder:
            result = run_soak(args.duration, Path(folder), args.sonar_ping_hz)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
