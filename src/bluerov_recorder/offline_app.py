"""Recorder workers and offline compatibility helpers.

The live entry point is :mod:`bluerov_recorder.live_app` and owns only the RGB
camera, Ping1D and read-only ROVL streams. Legacy Surveyor parser/worker
classes remain in this module solely for old offline fixtures and are never
constructed by the live CLI. BlueOS/SonarView owns onboard Surveyor capture.
"""

from __future__ import annotations

import argparse
import csv
import gc
import json
import logging
import math
import os
import queue
import shutil
import socket
import struct
import threading
import time
import tkinter as tk
import uuid
from datetime import datetime, timezone
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from PIL import Image, ImageDraw, ImageTk

try:
    from .rovl import DemoROVLWorker, ROVLWorker, list_serial_devices
except ImportError:  # Support direct execution from the package directory too.
    from rovl import DemoROVLWorker, ROVLWorker, list_serial_devices

try:
    from .runtime import (
        BufferedBinaryWriter, BufferedCsvWriter, BufferedJsonlWriter,
        LatestValueMailbox, MetricsRegistry, RecordingBackpressureError,
        publish_control_event,
    )
    from .surveyor_pipeline import SurveyorDecoderWorker
except ImportError:
    from runtime import (
        BufferedBinaryWriter, BufferedCsvWriter, BufferedJsonlWriter,
        LatestValueMailbox, MetricsRegistry, RecordingBackpressureError,
        publish_control_event,
    )
    from surveyor_pipeline import SurveyorDecoderWorker

try:
    from .rendering import render_surveyor_fan
except ImportError:
    from rendering import render_surveyor_fan

try:
    from .diagnostics import configure_diagnostics
except ImportError:
    from diagnostics import configure_diagnostics

try:
    from .processing import (
        MSG_ATOF, MSG_ATTITUDE, MSG_END_PING, MSG_JSON, MSG_RAW_PROFILE,
        PACKET_CHECKSUM, PACKET_HEADER, PingProtocolFramer, SurveyorChannelAccumulator,
        beamform_surveyor_channels, decode_atof_payload,
        decode_attitude_payload, decode_end_ping_payload, iter_svlog_packets,
        make_packet,
    )
except ImportError:  # Support direct execution from the package directory too.
    from processing import (
        MSG_ATOF, MSG_ATTITUDE, MSG_END_PING, MSG_JSON, MSG_RAW_PROFILE,
        PACKET_CHECKSUM, PACKET_HEADER, PingProtocolFramer, SurveyorChannelAccumulator,
        beamform_surveyor_channels, decode_atof_payload,
        decode_attitude_payload, decode_end_ping_payload, iter_svlog_packets,
        make_packet,
    )


SURVEYOR_HOST = "192.168.2.86"
SURVEYOR_PORT = 62312
PING1D_HOST = "192.168.2.2"
PING1D_PORT = 9090
BLUEOS_HOST = "192.168.2.2"
CAMERA_DEFAULT_PORT = 5600
CAMERA_ALTERNATE_PORT = 5602
# A little headroom above the 15 FPS display acceptance target compensates for
# ordinary Tk timer jitter while the capacity-one mailbox keeps memory bounded.
CAMERA_PREVIEW_FPS = 20.0
SURVEYOR_PREVIEW_FPS = 2.0
CONTROL_QUEUE_MAX_ITEMS = 2048

APP_ROOT = Path(__file__).resolve().parents[2]
SESSION_ROOT = APP_ROOT / "records" / "real_sessions"
LOGGER = logging.getLogger("bluerov_recorder.app")


def default_camera_source(port):
    """Use the bundled RTP description when one exists for the UDP port."""
    port = int(port)
    sdp_path = APP_ROOT / ("config_bluerov_%d.sdp" % port)
    if sdp_path.is_file():
        return str(sdp_path)
    return "udp://0.0.0.0:%d" % port

try:
    from brping import Ping1D, Surveyor240, definitions
    BRPING_IMPORT_ERROR = None
except ImportError as exc:  # Keep replay and GUI importable before install.
    Ping1D = None
    Surveyor240 = None
    definitions = None
    BRPING_IMPORT_ERROR = exc

try:
    import cv2
except ImportError:  # Camera is optional for replay/offline use.
    cv2 = None

try:
    import av
except ImportError:  # PyAV is preferred, but offline/replay stays importable.
    av = None


def utc_iso(ns: Optional[int] = None) -> str:
    value = time.time_ns() if ns is None else int(ns)
    return datetime.fromtimestamp(value / 1_000_000_000.0, timezone.utc).isoformat(timespec="milliseconds")


def normalized_row(values: Iterable[float], low=None, high=None) -> List[int]:
    """Convert an iterable of values into 0..255 display intensities."""
    values = list(values or [])
    if not values:
        return []
    if low is None:
        low = min(values)
    if high is None:
        high = max(values)
    low = float(low)
    high = float(high)
    if high <= low:
        return [0 for _ in values]
    return [
        max(0, min(255, int(round((float(value) - low) * 255.0 / (high - low)))))
        for value in values
    ]


def colorize(row: Sequence[int]) -> bytes:
    """Blue-to-yellow display palette used by the Ping1D profile."""
    out = bytearray(len(row) * 3)
    for index, value in enumerate(row):
        x = float(value) / 255.0
        red = int(255 * max(0.0, min(1.0, (x - 0.42) * 2.2)))
        green = int(255 * max(0.0, min(1.0, (x - 0.18) * 1.55)))
        blue = int(255 * max(0.0, min(1.0, 0.25 + x * 0.9)))
        offset = index * 3
        out[offset : offset + 3] = bytes((red, green, blue))
    return bytes(out)


def close_brping_device(device) -> None:
    """Close a brping device without assuming a package version."""
    if device is None:
        return
    io_device = getattr(device, "iodev", None)
    if io_device is not None:
        try:
            io_device.close()
        except Exception:
            pass


def ping1d_profile_record(profile, distance_record, host_monotonic_ns=None, host_utc_ns=None):
    """Normalize an official Ping1D profile and preserve the complete echo."""
    if not profile:
        return None
    data = list(profile.get("profile_data", []))
    start_mm = int(profile.get("scan_start", 0))
    length_mm = int(profile.get("scan_length", 0))
    distance_mm = int((distance_record or {}).get("distance", profile.get("distance", 0)))
    return {
        "timestamp": utc_iso(host_utc_ns),
        "host_monotonic_ns": int(host_monotonic_ns or time.monotonic_ns()),
        "host_utc_ns": int(host_utc_ns or time.time_ns()),
        "distance_mm": distance_mm,
        "distance_m": distance_mm / 1000.0,
        "confidence": int((distance_record or {}).get("confidence", profile.get("confidence", 0))),
        "scan_start_mm": start_mm,
        "scan_length_mm": length_mm,
        "gain": int(profile.get("gain_setting", -1)),
        "profile": [int(value) for value in data],
        "display_row": normalized_row(data, 0, 255),
    }


def decode_packet_stream_chunk(buffer: bytearray) -> List[Tuple[int, bytes, bytes]]:
    """Extract complete Ping Protocol packets from a mutable receive buffer."""
    packets = []
    while True:
        start = buffer.find(b"BR")
        if start < 0:
            if len(buffer) > 1:
                del buffer[:-1]
            break
        if start:
            del buffer[:start]
        if len(buffer) < PACKET_HEADER.size:
            break
        try:
            _a, _b, payload_len, message_id, _src, _dst = PACKET_HEADER.unpack_from(buffer)
        except struct.error:
            break
        total = PACKET_HEADER.size + int(payload_len) + PACKET_CHECKSUM.size
        if len(buffer) < total:
            break
        packet = bytes(buffer[:total])
        payload_start = PACKET_HEADER.size
        payload = packet[payload_start : payload_start + int(payload_len)]
        del buffer[:total]
        packets.append((int(message_id), payload, packet))
    return packets


def iter_live_packets(device, stop_event: threading.Event, idle_callback=None):
    """Read raw protocol packets from an already connected Surveyor socket."""
    io_device = getattr(device, "iodev", None)
    if io_device is None:
        raise RuntimeError("Surveyor socket non disponibile")
    receive_buffer = bytearray()
    while not stop_event.is_set():
        try:
            chunk = io_device.recv(65536)
            if not chunk:
                raise RuntimeError("connessione Surveyor chiusa")
            receive_buffer.extend(chunk)
        except (BlockingIOError, socket.timeout):
            if idle_callback is not None:
                idle_callback()
            time.sleep(0.005)
            continue
        for packet in decode_packet_stream_chunk(receive_buffer):
            yield packet


def normalise_device_timestamp(value: int) -> Optional[int]:
    """Convert common Surveyor millisecond/nanosecond timestamps to ns."""
    value = int(value or 0)
    if value <= 0:
        return None
    if value > 10_000_000_000_000:
        return value
    return value * 1_000_000


def time_base_parts(time_base) -> Tuple[Optional[int], Optional[int]]:
    """Return a PyAV Fraction-like time base as numerator/denominator."""
    if time_base is None:
        return None, None
    try:
        return int(time_base.numerator), int(time_base.denominator)
    except (AttributeError, TypeError, ValueError):
        return None, None


def timestamp_seconds(timestamp, time_base) -> Optional[float]:
    """Convert a native PTS/DTS only when both value and time base exist."""
    if timestamp is None or time_base is None:
        return None
    try:
        return float(timestamp * time_base)
    except (TypeError, ValueError):
        return None


def serializable_surveyor_record(record: Dict[str, object]) -> Dict[str, object]:
    """Keep JSONL compact while retaining parsed detections and timing."""
    return {key: value for key, value in record.items() if key not in ("matrix", "channel_signals")}


def build_surveyor_record(end_data, atof_data, decoded_channels, host_monotonic_ns, host_utc_ns, attitude=None):
    end_data = end_data or {}
    atof_data = atof_data or {}
    points = list(atof_data.get("points", []))
    record = {
        "ping_number": int(end_data.get("ping_number", atof_data.get("ping_number", 0))),
        "host_monotonic_ns": int(host_monotonic_ns),
        "host_utc_ns": int(host_utc_ns),
        "device_timestamp_ns": normalise_device_timestamp(int(atof_data.get("utc_msec", 0))) or normalise_device_timestamp(int(end_data.get("timestamp", 0))),
        "range_start_m": float(end_data.get("start_m", 0.0)),
        "range_end_m": float(end_data.get("end_m", 10.0)),
        "sos_mps": float(atof_data.get("sos_mps", 1500.0) or 1500.0),
        "ping_rate_hz": float(atof_data.get("ping_hz", end_data.get("ping_hz", 0.0)) or 0.0),
        "points": points,
        "detection_count": len(points),
        "channel_data_status": "AVAILABLE" if decoded_channels else "NOT AVAILABLE",
        "attitude": attitude,
    }
    if decoded_channels:
        channels, bins, note = decoded_channels
        record["bins"] = int(bins)
        record["channel_data_note"] = note
        record["channel_signals"] = channels
        try:
            record["matrix"] = beamform_surveyor_channels(channels, record["range_start_m"], record["range_end_m"], record["sos_mps"])
        except ValueError as exc:
            record["channel_data_status"] = "INVALID"
            record["channel_data_note"] = str(exc)
    return record


def replay_surveyor(path: Path, events: queue.Queue, stop_event: threading.Event, raw_callback=None) -> None:
    """Replay a local ``.svlog`` without creating a device or sending data."""
    accumulator = SurveyorChannelAccumulator()
    atof_by_ping = {}
    attitude = None
    last_ping_host_ns = time.monotonic_ns()
    for message_id, payload, raw_packet in iter_svlog_packets(Path(path)):
        if stop_event.is_set():
            break
        if raw_callback is not None:
            raw_callback(raw_packet, time.monotonic_ns(), time.time_ns())
        if message_id == MSG_RAW_PROFILE:
            previous_ping = accumulator.ping_number
            if not accumulator.add(payload) and previous_ping is not None:
                accumulator.reset()
                accumulator.add(payload)
        elif message_id == MSG_ATOF:
            data = decode_atof_payload(payload)
            if data:
                atof_by_ping[int(data["ping_number"])] = data
        elif message_id == MSG_ATTITUDE:
            attitude = decode_attitude_payload(payload)
        elif message_id == MSG_END_PING:
            end_data = decode_end_ping_payload(payload)
            if not end_data:
                continue
            ping_number = int(end_data["ping_number"])
            decoded = accumulator.decode() if accumulator.ping_number == ping_number else None
            now_mono = time.monotonic_ns()
            now_utc = time.time_ns()
            record = build_surveyor_record(end_data, atof_by_ping.pop(ping_number, None), decoded, now_mono, now_utc, attitude)
            record["replay_source"] = str(path)
            events.put(("surveyor_ping", record))
            last_ping_host_ns = now_mono
            accumulator.reset()
    events.put(("surveyor_replay_closed", {"path": str(path), "last_ping_host_ns": last_ping_host_ns}))


class SurveyorWorker(threading.Thread):
    """Passive/replay Surveyor reader with an explicit dry-mode TX lock."""

    def __init__(
        self, host, port, events, dry_mode=True, wet_authorized=False,
        replay_path=None, raw_callback=None, decoded_callback=None,
        preview_mailbox=None, attitude_mailbox=None, metrics=None,
    ):
        super(SurveyorWorker, self).__init__(daemon=False, name="surveyor-acquisition")
        self.host = host
        self.port = int(port)
        self.events = events
        self.dry_mode = bool(dry_mode)
        self.wet_authorized = bool(wet_authorized)
        self.replay_path = Path(replay_path) if replay_path else None
        self.raw_callback = raw_callback
        self.decoded_callback = decoded_callback
        self.metrics = metrics or MetricsRegistry()
        self.preview_mailbox = preview_mailbox if preview_mailbox is not None else LatestValueMailbox(
            self.metrics, "surveyor_preview_dropped", "surveyor_preview_published",
        )
        self.attitude_mailbox = attitude_mailbox if attitude_mailbox is not None else LatestValueMailbox(self.metrics)
        self.stop_event = threading.Event()
        self.commands = queue.Queue(maxsize=32)
        self.device = None
        self.started_by_app = False
        self.configured_ping_rate_hz = None
        self.decoder = None

    def _event(self, kind, data=None, critical=False):
        return publish_control_event(
            self.events, (kind, data), self.metrics, critical=critical,
        )

    def set_recording_callbacks(self, raw_callback=None, decoded_callback=None):
        self.raw_callback = raw_callback
        self.decoded_callback = decoded_callback

    def _record_decoded(self, record):
        callback = self.decoded_callback
        if callback is not None:
            callback(record)

    @property
    def tx_locked(self):
        return self.dry_mode or not self.wet_authorized

    def request_start(self, config=None):
        """Queue a future wet acquisition, never from default dry mode."""
        if self.tx_locked:
            raise PermissionError("SURVEYOR TX LOCKED: dry mode; nessuna trasmissione autorizzata")
        self.commands.put(("start", config or {}))

    def request_stop(self):
        self.commands.put(("stop", None))

    def disconnect(self):
        self.stop_event.set()
        close_brping_device(self.device)

    def _wet_start(self, config):
        """The only acoustic-start path; unreachable while the TX lock is set."""
        if self.tx_locked:
            raise PermissionError("SURVEYOR TX LOCKED")
        if self.device is None:
            raise RuntimeError("Surveyor non connesso")
        range_m = float(config.get("range_m", 20.0))
        ping_hz = float(config.get("ping_rate_hz", 5.0))
        self.device.control_set_ping_parameters(
            start_mm=0,
            end_mm=int(round(range_m * 1000.0)),
            sos_mps=int(config.get("sos_mps", 1500)),
            gain_index=-1,
            msec_per_ping=int(round(1000.0 / max(0.2, ping_hz))),
            ping_enable=True,
            enable_channel_data=True,
            enable_atof_data=True,
            target_ping_hz=240000,
            n_range_steps=int(config.get("n_range_steps", 400)),
        )
        self.configured_ping_rate_hz = ping_hz
        if self.decoder is not None:
            self.decoder.configured_ping_rate_hz = ping_hz
        self.started_by_app = True
        self._event("surveyor_started", None, critical=True)

    def _handle_commands(self):
        while True:
            try:
                name, payload = self.commands.get_nowait()
            except queue.Empty:
                return
            if name == "start":
                self._wet_start(payload)
            elif name == "stop":
                if self.started_by_app and self.device is not None and not self.tx_locked:
                    self.device.control_set_ping_parameters(ping_enable=False)
                self.started_by_app = False
                self._event("surveyor_stopped", None, critical=True)

    def _run_high_throughput(self):
        io_device = getattr(self.device, "iodev", None)
        if io_device is None:
            raise RuntimeError("Surveyor socket non disponibile")
        try:
            io_device.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
            self.metrics.set("surveyor_socket_receive_buffer_bytes", io_device.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF))
        except (AttributeError, OSError):
            self.metrics.increment("surveyor_socket_buffer_configuration_errors")
        framer = PingProtocolFramer(validate_checksum=True, accept_invalid=False)
        self.decoder = SurveyorDecoderWorker(
            self.preview_mailbox, self.attitude_mailbox, self.events,
            self.metrics, decoded_callback=self._record_decoded,
            configured_ping_rate_hz=self.configured_ping_rate_hz,
        )
        self.decoder.start()
        try:
            while not self.stop_event.is_set():
                self._handle_commands()
                try:
                    chunk = io_device.recv(65536)
                except (BlockingIOError, socket.timeout):
                    continue
                if not chunk:
                    raise RuntimeError("connessione Surveyor chiusa")
                host_mono = time.monotonic_ns()
                host_utc = time.time_ns()
                self.metrics.increment("surveyor_socket_chunks_received")
                self.metrics.increment("surveyor_bytes_received", len(chunk))
                packets = framer.feed(chunk)
                self.metrics.increment("surveyor_packets_received", len(packets))
                diagnostics = framer.diagnostics
                for key, value in diagnostics.as_dict().items():
                    self.metrics.set("surveyor_%s" % key, value)
                callback = self.raw_callback
                if callback is not None:
                    # Persist the exact TCP byte stream, including partial or
                    # malformed framing, before any optional processing.
                    callback(chunk, host_mono, host_utc, len(packets))
                for message_id, payload, raw_packet in packets:
                    self.metrics.increment("surveyor_message_%d" % message_id)
                    self.decoder.submit((
                        message_id, payload, raw_packet, host_mono, host_utc,
                    ))
        finally:
            if self.decoder is not None:
                self.decoder.close()
                self.decoder = None

    def _run_replay_pipeline(self):
        self.decoder = SurveyorDecoderWorker(
            self.preview_mailbox, self.attitude_mailbox, self.events,
            self.metrics, decoded_callback=self._record_decoded,
            configured_ping_rate_hz=self.configured_ping_rate_hz,
        )
        self.decoder.start()
        try:
            for message_id, payload, raw_packet in iter_svlog_packets(self.replay_path):
                if self.stop_event.is_set():
                    break
                host_mono = time.monotonic_ns()
                host_utc = time.time_ns()
                self.metrics.increment("surveyor_packets_received")
                self.metrics.increment("surveyor_bytes_received", len(raw_packet))
                callback = self.raw_callback
                if callback is not None:
                    callback(raw_packet, host_mono, host_utc, 1)
                self.decoder.submit((
                    message_id, payload, raw_packet, host_mono, host_utc,
                ))
        finally:
            self.decoder.close()
            self.decoder = None

    def _process_packets(self, packet_iterator):
        accumulator = SurveyorChannelAccumulator()
        atof_by_ping = {}
        attitude = None
        last_attitude_event_ns = 0
        for message_id, payload, raw_packet in packet_iterator:
            if self.stop_event.is_set():
                break
            # Commands are checked only while a packet is being processed. In
            # default dry mode no start command can enter this queue; with the
            # explicit future wet authorization this keeps the control path
            # on the same worker that owns the Surveyor socket.
            self._handle_commands()
            if self.stop_event.is_set():
                break
            host_mono = time.monotonic_ns()
            host_utc = time.time_ns()
            if self.raw_callback is not None:
                self.raw_callback(raw_packet, host_mono, host_utc)
            if message_id == MSG_RAW_PROFILE:
                previous_ping = accumulator.ping_number
                if not accumulator.add(payload) and previous_ping is not None:
                    accumulator.reset()
                    accumulator.add(payload)
            elif message_id == MSG_ATOF:
                data = decode_atof_payload(payload)
                if data:
                    atof_by_ping[int(data["ping_number"])] = data
            elif message_id == MSG_ATTITUDE:
                attitude = decode_attitude_payload(payload)
                if host_mono - last_attitude_event_ns >= 200_000_000:
                    self.events.put(("surveyor_attitude", attitude))
                    last_attitude_event_ns = host_mono
            elif message_id == MSG_END_PING:
                end_data = decode_end_ping_payload(payload)
                if not end_data:
                    continue
                ping_number = int(end_data["ping_number"])
                decoded = accumulator.decode() if accumulator.ping_number == ping_number else None
                record = build_surveyor_record(end_data, atof_by_ping.pop(ping_number, None), decoded, host_mono, host_utc, attitude)
                self.events.put(("surveyor_ping", record))
                accumulator.reset()

    def run(self):
        if self.replay_path is not None:
            try:
                self._event("surveyor_connected", {"replay": str(self.replay_path)})
                self._run_replay_pipeline()
            except Exception as exc:
                if not self.stop_event.is_set():
                    LOGGER.exception("Surveyor acquisition failure")
                    self._event("surveyor_error", str(exc), critical=True)
            finally:
                self._event("surveyor_closed", None)
            return
        if Surveyor240 is None:
            self._event("surveyor_error", "bluerobotics-ping non installato", critical=True)
            self._event("surveyor_closed", None)
            return
        while not self.stop_event.is_set():
            try:
                self.device = Surveyor240()
                self.device.connect_tcp(self.host, self.port)
                if self.device.initialize() is False:
                    raise RuntimeError("Surveyor240 initialize() fallita")
                self._event("surveyor_connected", {"replay": None, "tx_locked": self.tx_locked})
                # A wet start may have been requested while the network device
                # was still connecting. Process it before waiting for packets.
                self._handle_commands()
                self._run_high_throughput()
            except Exception as exc:
                if not self.stop_event.is_set():
                    critical = isinstance(exc, RecordingBackpressureError) or "CRITICAL DATA LOSS" in str(exc)
                    self._event(
                        "critical_data_loss" if critical else "surveyor_error",
                        str(exc) if critical else "%s; nuovo tentativo tra 2 s" % exc,
                        critical=True,
                    )
                    if critical:
                        self.stop_event.set()
            finally:
                if self.started_by_app and self.device is not None and not self.tx_locked:
                    try:
                        self.device.control_set_ping_parameters(ping_enable=False)
                    except Exception:
                        pass
                    self.started_by_app = False
                close_brping_device(self.device)
                self.device = None
            if not self.stop_event.is_set():
                self.stop_event.wait(2.0)
        self._event("surveyor_closed", None)


class Ping1DWorker(threading.Thread):
    """Official Ping1D worker through BlueOS PingProxy UDP."""

    def __init__(self, host, port, events, preview_mailbox=None, metrics=None, record_callback=None):
        super(Ping1DWorker, self).__init__(daemon=False, name="ping1d-acquisition")
        self.host = host
        self.port = int(port)
        self.events = events
        self.preview_mailbox = preview_mailbox
        self.metrics = metrics or MetricsRegistry()
        self.record_callback = record_callback
        self.stop_event = threading.Event()
        self.device = None

    def _event(self, kind, data=None, critical=False):
        return publish_control_event(self.events, (kind, data), self.metrics, critical)

    def set_record_callback(self, callback):
        self.record_callback = callback

    def run(self):
        try:
            if Ping1D is None:
                raise RuntimeError("bluerobotics-ping non installato")
            self.device = Ping1D()
            self.device.connect_udp(self.host, self.port)
            if self.device.initialize() is False:
                raise RuntimeError("Ping1D initialize() fallita")
            self._event("ping_connected", None)
            while not self.stop_event.is_set():
                profile = None
                get_profile = getattr(self.device, "get_profile", None)
                if callable(get_profile):
                    try:
                        profile = get_profile()
                    except Exception:
                        profile = None
                distance = {"distance": profile.get("distance", 0), "confidence": profile.get("confidence", 0)} if profile else self.device.get_distance()
                if not distance:
                    self._event("ping_warning", "nessuna risposta Ping1D")
                    time.sleep(0.1)
                    continue
                host_mono = time.monotonic_ns()
                host_utc = time.time_ns()
                record = {
                    "timestamp": utc_iso(host_utc),
                    "host_monotonic_ns": host_mono,
                    "host_utc_ns": host_utc,
                    "distance_mm": int(distance.get("distance", 0)),
                    "distance_m": float(distance.get("distance", 0)) / 1000.0,
                    "confidence": int(distance.get("confidence", 0)),
                }
                profile_record = ping1d_profile_record(profile, distance, host_mono, host_utc)
                self.metrics.increment("ping1d_samples_received")
                callback = self.record_callback
                if callback is not None:
                    callback({"distance": record, "profile": profile_record})
                    self.metrics.increment("ping1d_samples_recorded")
                if self.preview_mailbox is not None:
                    self.preview_mailbox.publish((record, profile_record))
                else:
                    self._event("ping_sample", (record, profile_record))
                time.sleep(0.02)
        except Exception as exc:
            if not self.stop_event.is_set():
                LOGGER.exception("Ping1D acquisition failure")
                self._event("ping_error", str(exc), critical=True)
        finally:
            close_brping_device(self.device)
            self._event("ping_closed", None)

    def stop(self):
        self.stop_event.set()
        close_brping_device(self.device)


class CameraWorker(threading.Thread):
    """Single-ingest camera reader with PyAV remux/decode fan-out.

    PyAV demuxes each encoded packet once. The packet is offered to the
    session remuxer and then decoded frames are sent to the GUI. OpenCV is
    used only when PyAV is unavailable or cannot open the selected source.
    """

    def __init__(
        self, port=CAMERA_DEFAULT_PORT, source=None, events=None,
        preview_mailbox=None, metrics=None,
    ):
        super(CameraWorker, self).__init__(daemon=False, name="camera-ingest")
        self.port = int(port)
        self.source = source or default_camera_source(self.port)
        self.events = events if events is not None else queue.Queue(maxsize=CONTROL_QUEUE_MAX_ITEMS)
        self.metrics = metrics or MetricsRegistry()
        self.preview_mailbox = preview_mailbox
        self.stop_event = threading.Event()
        self.capture = None
        self.container = None
        self.packet_callback = None
        self.frame_callback = None
        self._callback_lock = threading.Lock()
        self.backend = "PENDING"

    def set_packet_callback(self, callback):
        """Atomically replace the session callback used by the ingest thread."""
        with self._callback_lock:
            self.packet_callback = callback

    def set_frame_callback(self, callback):
        with self._callback_lock:
            self.frame_callback = callback

    def _get_packet_callback(self):
        with self._callback_lock:
            return self.packet_callback

    def _get_frame_callback(self):
        with self._callback_lock:
            return self.frame_callback

    def _event(self, kind, data=None, critical=False):
        return publish_control_event(
            self.events, (kind, data), self.metrics, critical=critical,
        )

    def _publish_preview(self, image, frame_metadata):
        if self.preview_mailbox is not None:
            self.preview_mailbox.publish((image, frame_metadata))
        else:
            self._event("camera_frame", (image, frame_metadata))

    @staticmethod
    def _stream_metadata(stream, source, backend, recording_mode, reason=None):
        codec_context = getattr(stream, "codec_context", None)
        codec = getattr(codec_context, "name", None) or getattr(stream, "codec", None)
        if not isinstance(codec, str):
            codec = getattr(codec, "name", None)
        average_rate = getattr(stream, "average_rate", None)
        try:
            nominal_fps = float(average_rate) if average_rate is not None else None
        except (TypeError, ValueError):
            nominal_fps = None
        tb_num, tb_den = time_base_parts(getattr(stream, "time_base", None))
        result = {
            "backend": backend,
            "recording_mode": recording_mode,
            "source": str(source),
            "codec": codec,
            "width": int(getattr(codec_context, "width", 0) or 0) or None,
            "height": int(getattr(codec_context, "height", 0) or 0) or None,
            "nominal_fps": nominal_fps,
            "time_base": {"num": tb_num, "den": tb_den},
            "video_pts": "AVAILABLE" if tb_num is not None and tb_den is not None else "UNAVAILABLE",
            "input_status": "PYAV INPUT: OK" if backend == "PyAV" else "PYAV INPUT: FAILED / OPENCV FALLBACK",
            "remux_status": "READY" if backend == "PyAV" else "FAILED",
        }
        if reason:
            result["reason"] = str(reason)
        return result

    @staticmethod
    def _packet_frame_metadata(packet, stream, packet_index, frame=None, host_monotonic_ns=None, host_utc_ns=None, frame_index=None):
        time_base = getattr(frame, "time_base", None) or getattr(packet, "time_base", None) or getattr(stream, "time_base", None)
        tb_num, tb_den = time_base_parts(time_base)
        pts = getattr(frame, "pts", None) if frame is not None else None
        dts = getattr(frame, "dts", None) if frame is not None else None
        if pts is None:
            pts = getattr(packet, "pts", None)
        if dts is None:
            dts = getattr(packet, "dts", None)
        packet_size = getattr(packet, "size", None)
        if packet_size is None and packet is not None:
            try:
                packet_size = len(packet)
            except TypeError:
                packet_size = None
        return {
            "frame_index": frame_index,
            "packet_index": packet_index,
            "pts": int(pts) if pts is not None else None,
            "dts": int(dts) if dts is not None else None,
            "time_base_num": tb_num,
            "time_base_den": tb_den,
            "pts_seconds": timestamp_seconds(pts, time_base),
            "dts_seconds": timestamp_seconds(dts, time_base),
            "host_monotonic_ns": int(host_monotonic_ns or time.monotonic_ns()),
            "host_utc_ns": int(host_utc_ns or time.time_ns()),
            "session_time_s": None,
            "key_frame": bool(getattr(packet, "is_keyframe", False) or getattr(frame, "key_frame", False)),
            "packet_size": int(packet_size) if packet_size is not None else None,
        }

    def _emit_backend(self, metadata):
        self.backend = metadata.get("backend", "UNKNOWN")
        self._event("camera_backend", metadata)

    def _run_pyav(self):
        if av is None:
            raise RuntimeError("PyAV non installato")
        source_text = str(self.source)
        open_kwargs = {}
        if source_text.lower().startswith(("udp://", "rtp://")):
            open_kwargs["options"] = {"fflags": "nobuffer", "flags": "low_delay"}
        elif source_text.lower().endswith(".sdp"):
            open_kwargs["options"] = {
                "protocol_whitelist": "file,udp,rtp",
                "fflags": "nobuffer",
                "flags": "low_delay",
            }
        self.container = av.open(self.source, mode="r", **open_kwargs)
        streams = [stream for stream in self.container.streams if getattr(stream, "type", None) == "video"]
        if not streams:
            raise RuntimeError("la sorgente non contiene uno stream video")
        stream = streams[0]
        metadata = self._stream_metadata(stream, self.source, "PyAV", "READY")
        metadata["port"] = self.port
        self._emit_backend(metadata)
        self._event("camera_connected", metadata)
        packet_index = 0
        frame_index = 0
        last_preview_ns = 0
        preview_interval_ns = int(1_000_000_000 / CAMERA_PREVIEW_FPS)
        for packet in self.container.demux(stream):
            if self.stop_event.is_set():
                break
            if packet is None:
                continue
            host_mono = time.monotonic_ns()
            host_utc = time.time_ns()
            self.metrics.increment("camera_packets_received")
            callback = self._get_packet_callback()
            packet_recorded = False
            if callback is not None:
                packet_recorded = bool(callback(
                    packet, stream, packet_index, host_mono, host_utc,
                ))
                if packet_recorded:
                    self.metrics.increment("camera_packets_recorded")
            try:
                frames = list(packet.decode())
            except Exception as exc:
                frames = []
                # Count every failure but only publish sparse warnings.
                failures = self.metrics.increment("camera_decode_errors")
                if failures == 1 or int(failures) % 100 == 0:
                    self._event("camera_decode_warning", {
                        "error": str(exc), "occurrence_count": int(failures),
                    })
            if not frames:
                packet_index += 1
                continue
            for frame in frames:
                if self.stop_event.is_set():
                    break
                current_frame_index = frame_index
                frame_index += 1
                self.metrics.increment("camera_frames_decoded")
                frame_metadata = self._packet_frame_metadata(
                    packet, stream, packet_index, frame, host_mono,
                    host_utc, current_frame_index,
                )
                frame_callback = self._get_frame_callback()
                needs_fallback_frame = frame_callback is not None and callback is not None and not packet_recorded
                # Decoding/remuxing keeps running at the source rate, but the
                # Tk preview only needs a bounded refresh rate.  Converting and
                # queueing every 1080p frame can otherwise starve all other GUI
                # events, making healthy devices appear disconnected.
                if host_mono - last_preview_ns < preview_interval_ns:
                    if frame_callback is not None:
                        image = frame.to_ndarray(format="bgr24") if needs_fallback_frame else None
                        frame_callback(image, frame_metadata)
                    continue
                image = frame.to_ndarray(format="bgr24")
                if frame_callback is not None:
                    frame_callback(image if needs_fallback_frame else None, frame_metadata)
                self._publish_preview(image, frame_metadata)
                last_preview_ns = host_mono
            packet_index += 1

    def _run_opencv_fallback(self, reason):
        if cv2 is None:
            raise RuntimeError("PyAV non disponibile e OpenCV non installato: installare requirements.txt")
        self.capture = cv2.VideoCapture(self.source, getattr(cv2, "CAP_FFMPEG", 0))
        if not self.capture.isOpened():
            self.capture.release()
            self.capture = cv2.VideoCapture(self.source)
        if not self.capture.isOpened():
            raise RuntimeError("PyAV non ha aperto %s; OpenCV fallback non riesce ad aprire la sorgente: %s" % (self.source, reason))
        width = int(self.capture.get(getattr(cv2, "CAP_PROP_FRAME_WIDTH", 3)) or 0) or None
        height = int(self.capture.get(getattr(cv2, "CAP_PROP_FRAME_HEIGHT", 4)) or 0) or None
        fps = float(self.capture.get(getattr(cv2, "CAP_PROP_FPS", 5)) or 0.0) or None
        metadata = {
            "backend": "OpenCV fallback",
            "recording_mode": "DECODED/REENCODED FALLBACK",
            "source": str(self.source),
            "codec": None,
            "width": width,
            "height": height,
            "nominal_fps": fps,
            "time_base": {"num": None, "den": None},
            "video_pts": "UNAVAILABLE",
            "reason": str(reason),
            "port": self.port,
            "input_status": "PYAV INPUT: FAILED / OPENCV FALLBACK",
            "remux_status": "FAILED",
        }
        self._emit_backend(metadata)
        self._event("camera_connected", metadata)
        frame_index = 0
        last_preview_ns = 0
        while not self.stop_event.is_set():
            ok, frame = self.capture.read()
            if not ok or frame is None:
                source_text = str(self.source).lower()
                if not source_text.startswith(("udp://", "rtp://")):
                    break
                time.sleep(0.02)
                continue
            host_mono = time.monotonic_ns()
            host_utc = time.time_ns()
            self.metrics.increment("camera_frames_decoded")
            metadata = {
                "frame_index": frame_index,
                "packet_index": None,
                "pts": None,
                "dts": None,
                "time_base_num": None,
                "time_base_den": None,
                "pts_seconds": None,
                "dts_seconds": None,
                "host_monotonic_ns": host_mono,
                "host_utc_ns": host_utc,
                "session_time_s": None,
                "key_frame": None,
                "packet_size": None,
            }
            frame_callback = self._get_frame_callback()
            if frame_callback is not None:
                frame_callback(frame, metadata)
            if frame_index == 0 or host_mono - last_preview_ns >= int(1_000_000_000 / CAMERA_PREVIEW_FPS):
                self._publish_preview(frame, metadata)
                last_preview_ns = host_mono
            frame_index += 1

    def run(self):
        try:
            if av is not None:
                try:
                    self._run_pyav()
                    return
                except RecordingBackpressureError:
                    raise
                except Exception as exc:
                    if self.container is not None:
                        try:
                            self.container.close()
                        except Exception:
                            pass
                    self.container = None
                    self._run_opencv_fallback(exc)
            else:
                self._run_opencv_fallback("PyAV non installato")
        except Exception as exc:
            if not self.stop_event.is_set():
                LOGGER.exception("Camera acquisition failure")
                self._event("camera_error", str(exc), critical=True)
        finally:
            self.set_packet_callback(None)
            self.set_frame_callback(None)
            if self.capture is not None:
                self.capture.release()
            if self.container is not None:
                try:
                    self.container.close()
                except Exception:
                    pass
            self._event("camera_closed", None)

    def stop(self):
        self.stop_event.set()
        self.set_packet_callback(None)
        self.set_frame_callback(None)
        if self.capture is not None:
            try:
                self.capture.release()
            except Exception:
                pass
        if self.container is not None:
            try:
                self.container.close()
            except Exception:
                pass


class BlueOSWorker(threading.Thread):
    """Read-only reachability probe for BlueOS HTTP/mavlink2rest."""

    def __init__(self, host, events):
        super(BlueOSWorker, self).__init__(daemon=True)
        self.host = host
        self.events = events
        self.stop_event = threading.Event()

    @staticmethod
    def _check(host, port):
        try:
            sock = socket.create_connection((host, port), 1.0)
            sock.close()
            return True
        except OSError:
            return False

    def run(self):
        while not self.stop_event.is_set():
            online = self._check(self.host, 80) or self._check(self.host, 6040)
            self.events.put(("blueos", online))
            self.stop_event.wait(2.0)


class SessionRecorder:
    """Synchronized session backed by independent bounded writer pipelines."""

    CAMERA_TIMESTAMP_HEADER = [
        "session_id", "session_start_utc_ns", "session_start_monotonic_ns",
        "frame_index", "packet_index", "pts", "dts", "time_base_num",
        "time_base_den", "pts_seconds", "dts_seconds", "host_monotonic_ns",
        "host_utc_ns", "session_time_s", "key_frame", "packet_size",
    ]
    ROVL_TIMESTAMP_HEADER = [
        "line_index", "host_monotonic_ns", "host_utc_ns", "session_time_s",
        "message_type", "checksum_present", "checksum_ok", "byte_offset",
        "byte_length",
    ]

    def __init__(
        self,
        root=SESSION_ROOT,
        camera_port=CAMERA_DEFAULT_PORT,
        camera_source=None,
        surveyor_mode="live",
        surveyor_replay_source=None,
        rovl_connected=False,
        rovl_port=None,
        rovl_synthetic=False,
        surveyor_only=False,
        metrics=None,
        critical_callback=None,
        surveyor_tx="LOCKED",
        enable_surveyor=True,
    ):
        self.root = Path(root)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_id = "%s_%s" % (timestamp, uuid.uuid4().hex[:8])
        self.directory = self.root / self.session_id
        self.directory.mkdir(parents=True, exist_ok=False)
        self.session_start_utc_ns = time.time_ns()
        self.session_start_monotonic_ns = time.monotonic_ns()
        self.camera_port = int(camera_port)
        self.camera_source = camera_source or default_camera_source(self.camera_port)
        self.surveyor_mode = str(surveyor_mode)
        # The live recorder now passes enable_surveyor=False.  The legacy
        # default is retained only so old offline/session fixtures remain
        # readable; no live code path enables it anymore.
        self.enable_surveyor = bool(enable_surveyor)
        self.surveyor_only = bool(surveyor_only)
        self.surveyor_replay_source = str(surveyor_replay_source) if surveyor_replay_source else None
        self.metrics = metrics or MetricsRegistry()
        self.critical_callback = critical_callback
        self.state_lock = threading.Lock()
        self.metadata_file_lock = threading.Lock()
        self.camera_lock = threading.Lock()
        self.rovl_lock = threading.Lock()
        self.closed = False
        self.degraded = False
        self.degraded_reasons = []
        self.camera_writer = None
        self.camera_container = None
        self.camera_stream = None
        self.camera_size = None
        self.camera_frame_index = 0
        self.rovl_line_index = 0
        self.rovl_byte_offset = 0
        self.camera_backend = "PENDING"
        self.camera_recording_mode = "PENDING"
        self.remux_failed = False
        self.raw_writer = None
        self.rovl_raw_writer = None
        self.rovl_timestamp_writer = None
        self.rovl_position_writer = None
        self.camera_metadata = {
            "enabled": not self.surveyor_only,
            "connected": False,
            "backend": "PENDING", "recording_mode": "PENDING",
            "source": self.camera_source, "codec": None, "width": None,
            "height": None, "nominal_fps": None,
            "time_base": {"num": None, "den": None},
            "video_pts": "UNAVAILABLE", "input_status": "PENDING",
            "remux_status": "NOT STARTED",
        }
        self.session_metadata = {
            "session_id": self.session_id,
            "session_start_utc_ns": self.session_start_utc_ns,
            "session_start_monotonic_ns": self.session_start_monotonic_ns,
            "session_start_utc": utc_iso(self.session_start_utc_ns),
            "camera": self.camera_metadata,
            "surveyor_recording": "external / BlueOS SonarView",
            "ping1d": {
                "host": PING1D_HOST, "port": PING1D_PORT,
                "enabled": not self.surveyor_only, "connected": False,
            },
            "rovl": {
                "enabled": False, "connected": bool(rovl_connected),
                "port": rovl_port, "baud": 115200, "mode": "read-only",
                "synthetic": bool(rovl_synthetic),
            },
            "health": {"degraded": False, "reasons": []},
        }
        if self.enable_surveyor:
            self.session_metadata["surveyor"] = {
                "host": SURVEYOR_HOST, "port": SURVEYOR_PORT,
                "mode": self.surveyor_mode,
                "replay_source": self.surveyor_replay_source,
                "tx": str(surveyor_tx),
            }
        self.surveyor_ping_writer = (
            BufferedJsonlWriter(self.directory / "surveyor_pings.jsonl", self.metrics, "surveyor_decoded")
            if self.enable_surveyor else None
        )
        self.event_writer = BufferedJsonlWriter(
            self.directory / "events.jsonl", self.metrics, "events"
        )
        self.diagnostics_writer = BufferedJsonlWriter(
            self.directory / "diagnostics.jsonl", self.metrics, "diagnostics", max_items=2048
        )
        self.ping_writer = None
        self.camera_timestamp_writer = None
        if not self.surveyor_only:
            self.ping_writer = BufferedJsonlWriter(
                self.directory / "ping1d.jsonl", self.metrics, "ping1d"
            )
            self.camera_timestamp_writer = BufferedCsvWriter(
                self.directory / "camera_timestamps.csv",
                self.CAMERA_TIMESTAMP_HEADER, self.metrics, "camera_timestamps",
            )
            self.session_metadata["camera"]["port"] = self.camera_port
        if self.enable_surveyor and self.surveyor_mode != "skipped":
            self.raw_writer = BufferedBinaryWriter(
                self.directory / "surveyor_raw.svlog", self.metrics,
                "surveyor_raw", max_queue_bytes=32 * 1024 * 1024,
            )
            self.raw_writer.start()
        self._write_session(self.session_metadata)
        self._write_svlog_metadata()
        if rovl_connected and not rovl_synthetic:
            self.enable_rovl(rovl_port, synthetic=False)

    def _write_session(self, data):
        with self.metadata_file_lock:
            temporary = self.directory / "session.json.tmp"
            with temporary.open("w", encoding="utf-8") as stream:
                json.dump(data, stream, ensure_ascii=False, indent=2)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(str(temporary), str(self.directory / "session.json"))

    def mark_degraded(self, reason):
        reason = str(reason)
        added = False
        with self.state_lock:
            self.degraded = True
            if reason not in self.degraded_reasons:
                self.degraded_reasons.append(reason)
                added = True
            self.session_metadata["health"] = {
                "degraded": True, "reasons": list(self.degraded_reasons),
            }
            self._write_session(self.session_metadata)
        if added and self.critical_callback is not None:
            self.critical_callback(reason)

    def update_camera_metadata(self, metadata):
        with self.state_lock:
            if self.closed:
                return
            self.camera_metadata.update(dict(metadata or {}))
            self.camera_backend = self.camera_metadata.get("backend", self.camera_backend)
            self.camera_recording_mode = self.camera_metadata.get("recording_mode", self.camera_recording_mode)
            self.session_metadata["camera"] = dict(self.camera_metadata)
            self.session_metadata["camera"]["port"] = self.camera_port
            self._write_session(self.session_metadata)

    def _write_svlog_metadata(self):
        if self.raw_writer is None:
            return
        metadata = {
            "session_id": self.session_id, "session_uptime": 0.0,
            "session_devices": [{
                "url": "tcp://%s:%d" % (SURVEYOR_HOST, SURVEYOR_PORT),
                "product_id": "mbes24016",
            }],
            "is_recording": True, "sonarlink_version": "",
            "timestamp": utc_iso(self.session_start_utc_ns),
            "recorder": "BlueROV2 Multimodal Recorder",
            "mode": self.surveyor_mode,
            "replay_source": self.surveyor_replay_source,
            "session_start_utc_ns": self.session_start_utc_ns,
            "session_start_monotonic_ns": self.session_start_monotonic_ns,
        }
        self.raw_writer.submit(
            make_packet(MSG_JSON, json.dumps(metadata, indent=2).encode("utf-8")),
            units=0,
        )

    def write_surveyor_packet(self, raw_packet, host_monotonic_ns=None, host_utc_ns=None, packet_count=1):
        if self.closed or self.raw_writer is None:
            return
        try:
            self.raw_writer.submit(bytes(raw_packet), units=int(packet_count))
        except RecordingBackpressureError as exc:
            self.mark_degraded(str(exc))
            raise

    def write_surveyor_ping(self, record):
        if self.closed:
            return
        item = serializable_surveyor_record(record)
        item.update({
            "session_id": self.session_id,
            "session_start_utc_ns": self.session_start_utc_ns,
            "session_start_monotonic_ns": self.session_start_monotonic_ns,
        })
        try:
            self.surveyor_ping_writer.submit(item)
        except RecordingBackpressureError as exc:
            self.mark_degraded(str(exc))
            raise

    def write_ping1d(self, record):
        if self.closed or self.ping_writer is None:
            return
        item = dict(record)
        item.update({
            "session_id": self.session_id,
            "session_start_utc_ns": self.session_start_utc_ns,
            "session_start_monotonic_ns": self.session_start_monotonic_ns,
        })
        try:
            self.ping_writer.submit(item)
        except RecordingBackpressureError as exc:
            self.mark_degraded(str(exc))
            raise

    def enable_rovl(self, port, synthetic=False):
        """Create independent ROVL writers only for a physical connection."""
        with self.rovl_lock:
            if self.closed or synthetic:
                return False
            metadata = self.session_metadata["rovl"]
            metadata.update({
                "enabled": True, "connected": True, "port": port,
                "baud": 115200, "mode": "read-only", "synthetic": False,
            })
            if self.rovl_raw_writer is None:
                self.rovl_raw_writer = BufferedBinaryWriter(
                    self.directory / "rovl_raw.nmea", self.metrics,
                    "rovl_raw", max_queue_bytes=4 * 1024 * 1024,
                )
                self.rovl_raw_writer.start()
                self.rovl_timestamp_writer = BufferedCsvWriter(
                    self.directory / "rovl_timestamps.csv",
                    self.ROVL_TIMESTAMP_HEADER, self.metrics, "rovl_timestamps",
                )
                self.rovl_position_writer = BufferedJsonlWriter(
                    self.directory / "rovl_positions.jsonl", self.metrics,
                    "rovl_positions",
                )
            self._write_session(self.session_metadata)
            return True

    def mark_rovl_disconnected(self):
        with self.rovl_lock:
            if not self.closed:
                self.session_metadata["rovl"]["connected"] = False
                self._write_session(self.session_metadata)

    def write_rovl_sample(self, sample):
        """Queue exact serial bytes and decoded metadata without touching GUI."""
        if self.closed or sample.get("synthetic"):
            return False
        if self.rovl_raw_writer is None:
            self.enable_rovl(sample.get("port"), synthetic=False)
        if self.rovl_raw_writer is None:
            return False
        raw = bytes(sample.get("raw_bytes") or b"")
        with self.rovl_lock:
            line_index = self.rovl_line_index
            byte_offset = self.rovl_byte_offset
            self.rovl_line_index += 1
            self.rovl_byte_offset += len(raw)
        host_monotonic_ns = int(sample.get("host_monotonic_ns") or time.monotonic_ns())
        host_utc_ns = int(sample.get("host_utc_ns") or time.time_ns())
        session_time_s = (host_monotonic_ns - self.session_start_monotonic_ns) / 1_000_000_000.0
        parsed = dict(sample.get("parsed") or {})
        try:
            self.rovl_raw_writer.submit(raw)
            self.rovl_timestamp_writer.submit([
                line_index, host_monotonic_ns, host_utc_ns, session_time_s,
                parsed.get("message_type", "UNKNOWN"),
                parsed.get("checksum_present"), parsed.get("checksum_ok"),
                byte_offset, len(raw),
            ])
            item = dict(parsed)
            item.update({
                "session_id": self.session_id,
                "session_start_utc_ns": self.session_start_utc_ns,
                "session_start_monotonic_ns": self.session_start_monotonic_ns,
                "line_index": line_index,
                "host_monotonic_ns": host_monotonic_ns,
                "host_utc_ns": host_utc_ns,
                "session_time_s": session_time_s,
                "raw_reference": {
                    "file": "rovl_raw.nmea", "byte_offset": byte_offset,
                    "byte_length": len(raw),
                },
                "position": sample.get("position"), "synthetic": False,
            })
            self.rovl_position_writer.submit(item)
            return True
        except RecordingBackpressureError as exc:
            self.mark_degraded(str(exc))
            raise

    def write_event(self, kind, data=None):
        if self.closed:
            return
        item = {
            "timestamp": utc_iso(), "session_id": self.session_id,
            "session_start_utc_ns": self.session_start_utc_ns,
            "session_start_monotonic_ns": self.session_start_monotonic_ns,
            "kind": kind, "data": data,
        }
        self.event_writer.submit(item)

    def write_diagnostics(self, snapshot):
        if self.closed:
            return
        item = dict(snapshot)
        item.update({
            "timestamp": utc_iso(), "session_id": self.session_id,
            "session_time_s": (
                time.monotonic_ns() - self.session_start_monotonic_ns
            ) / 1_000_000_000.0,
        })
        self.diagnostics_writer.submit(item)

    def _set_camera_recording_mode(self, mode, reason=None, remux_status=None):
        new_mode = str(mode)
        new_remux_status = str(remux_status) if remux_status is not None else self.camera_metadata.get("remux_status")
        old_reason = self.camera_metadata.get("recording_reason")
        changed = self.camera_recording_mode != new_mode or self.camera_metadata.get("remux_status") != new_remux_status
        if reason is not None and str(reason) != old_reason:
            changed = True
        self.camera_recording_mode = new_mode
        self.camera_metadata["recording_mode"] = self.camera_recording_mode
        if remux_status is not None:
            self.camera_metadata["remux_status"] = new_remux_status
        if reason:
            self.camera_metadata["recording_reason"] = str(reason)
        if not changed:
            return
        self.session_metadata["camera"] = dict(self.camera_metadata)
        self.session_metadata["camera"]["port"] = self.camera_port
        self._write_session(self.session_metadata)

    def write_camera_packet(self, packet, stream, packet_index, host_monotonic_ns, host_utc_ns):
        """Remux one encoded packet into MKV without decoding/re-encoding."""
        with self.camera_lock:
            if self.closed or av is None:
                return False
            if self.remux_failed:
                return False
            if self.camera_container is None:
                try:
                    self.camera_container = av.open(str(self.directory / "camera_rgb.mkv"), mode="w", format="matroska")
                    try:
                        self.camera_stream = self.camera_container.add_stream(template=stream)
                    except TypeError:
                        codec_context = getattr(stream, "codec_context", None)
                        codec_name = getattr(codec_context, "name", None) or "h264"
                        self.camera_stream = self.camera_container.add_stream(codec_name)
                        self.camera_stream.time_base = getattr(stream, "time_base", None)
                        if codec_context is not None:
                            self.camera_stream.codec_context.extradata = getattr(codec_context, "extradata", None)
                    backend = self.camera_backend if self.camera_backend != "PENDING" else "PyAV"
                    metadata = CameraWorker._stream_metadata(stream, self.camera_source, backend, "READY")
                    metadata["port"] = self.camera_port
                    self.camera_metadata.update(metadata)
                    self.camera_backend = metadata.get("backend", "PyAV")
                    self._set_camera_recording_mode("READY", remux_status="READY")
                except Exception as exc:
                    if self.camera_container is not None:
                        try:
                            self.camera_container.close()
                        except Exception:
                            pass
                    self.camera_container = None
                    self.camera_stream = None
                    self.remux_failed = True
                    self._set_camera_recording_mode("DECODED/REENCODED FALLBACK", exc, remux_status="FAILED")
                    return False
            try:
                original_stream = getattr(packet, "stream", None)
                packet.stream = self.camera_stream
                self.camera_container.mux(packet)
                packet.stream = original_stream
                self._set_camera_recording_mode("REMUX H264", remux_status="ACTIVE")
                return True
            except Exception as exc:
                try:
                    packet.stream = original_stream
                except Exception:
                    pass
                self._set_camera_recording_mode("DECODED/REENCODED FALLBACK", exc, remux_status="FAILED")
                try:
                    self.camera_container.close()
                except Exception:
                    pass
                self.camera_container = None
                self.camera_stream = None
                self.remux_failed = True
                return False

    def write_camera_frame(self, frame, frame_metadata):
        """Record decoded frames only for the explicit fallback path."""
        with self.camera_lock:
            if self.closed:
                return
            item = dict(frame_metadata or {})
            host_mono = int(item.get("host_monotonic_ns") or time.monotonic_ns())
            item["session_time_s"] = (host_mono - self.session_start_monotonic_ns) / 1_000_000_000.0
            self._write_camera_timestamp(item)
            if self.camera_recording_mode == "REMUX H264":
                return
            if cv2 is None:
                return
            height, width = frame.shape[:2]
            if self.camera_writer is None:
                self.camera_size = (int(width), int(height))
                path = str(self.directory / "camera_rgb.mkv")
                self.camera_writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"XVID"), 30.0, self.camera_size)
                if not self.camera_writer.isOpened():
                    self.camera_writer.release()
                    self.camera_writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), 30.0, self.camera_size)
                self._set_camera_recording_mode("DECODED/REENCODED FALLBACK", self.camera_metadata.get("recording_reason"))
            if self.camera_writer is not None and self.camera_writer.isOpened():
                self.camera_writer.write(frame)

    def _write_camera_timestamp(self, item):
        values = [
            self.session_id, self.session_start_utc_ns, self.session_start_monotonic_ns,
            item.get("frame_index"), item.get("packet_index"), item.get("pts"),
            item.get("dts"), item.get("time_base_num"), item.get("time_base_den"),
            item.get("pts_seconds"), item.get("dts_seconds"), item.get("host_monotonic_ns"),
            item.get("host_utc_ns"), item.get("session_time_s"), item.get("key_frame"),
            item.get("packet_size"),
        ]
        if values[13] is None and item.get("host_monotonic_ns") is not None:
            values[13] = (int(item["host_monotonic_ns"]) - self.session_start_monotonic_ns) / 1_000_000_000.0
        if self.camera_timestamp_writer is not None:
            self.camera_timestamp_writer.submit(values)
        self.camera_frame_index = max(self.camera_frame_index, int(item.get("frame_index") or 0) + 1)

    def close(self):
        with self.state_lock:
            if self.closed:
                return
            self.closed = True
        close_errors = []
        with self.camera_lock:
            if self.camera_container is not None:
                try:
                    self.camera_container.close()
                except Exception as exc:
                    close_errors.append("camera container: %s" % exc)
            if self.camera_writer is not None:
                self.camera_writer.release()
        writers = [
            self.raw_writer, self.surveyor_ping_writer, self.ping_writer,
            self.event_writer, self.diagnostics_writer,
            self.camera_timestamp_writer, self.rovl_raw_writer,
            self.rovl_timestamp_writer, self.rovl_position_writer,
        ]
        for writer in writers:
            if writer is not None:
                try:
                    writer.close()
                except Exception as exc:
                    close_errors.append(str(exc))
        metadata = dict(self.session_metadata)
        metadata["session_end_utc_ns"] = time.time_ns()
        metadata["session_end_monotonic_ns"] = time.monotonic_ns()
        metadata["closed"] = True
        if close_errors:
            self.degraded = True
            self.degraded_reasons.extend(close_errors)
        metadata["health"] = {
            "degraded": bool(self.degraded),
            "reasons": list(dict.fromkeys(self.degraded_reasons)),
            "final_metrics": self.metrics.snapshot(),
        }
        self._write_session(metadata)


def nearest_sample(target_monotonic_ns: int, samples: Sequence[Dict[str, object]]) -> Optional[Dict[str, object]]:
    """Return the nearest timestamped stream sample for synchronizers."""
    if not samples:
        return None
    return min(samples, key=lambda item: abs(int(item.get("host_monotonic_ns", 0)) - int(target_monotonic_ns)))


def match_surveyor_ping(ping_record, rgb_frames, ping1d_samples):
    """Match one Surveyor ping with nearest RGB frame and Ping1D sample."""
    target = int(ping_record.get("host_monotonic_ns", 0))
    camera = nearest_sample(target, rgb_frames)
    ping1d = nearest_sample(target, ping1d_samples)

    def delta_ms(sample):
        if sample is None:
            return None
        return (int(sample.get("host_monotonic_ns", 0)) - target) / 1_000_000.0

    return {
        "surveyor": ping_record,
        "rgb_frame": camera,
        "ping1d": ping1d,
        "time_delta_camera_ms": delta_ms(camera),
        "time_delta_ping1d_ms": delta_ms(ping1d),
        "camera_pts": camera.get("pts") if camera is not None else None,
        "camera_pts_seconds": camera.get("pts_seconds") if camera is not None else None,
    }


def fan_image(record, width=720, height=560, brightness=1.0, contrast=1.0, show_atof=True):
    """Render a Surveyor intensity matrix as a polar ±40° fan."""
    image = Image.new("RGB", (width, height), "#06101d")
    draw = ImageDraw.Draw(image)
    matrix = record.get("matrix") or []
    if not matrix:
        draw.text((18, 18), "SURVEYOR FAN IMAGE — nessun channel data", fill="#dceaf2")
        return image
    try:
        return render_surveyor_fan(
            record, width, height, brightness, contrast, show_atof,
        )
    except (ValueError, TypeError):
        # Compatibility fallback for environments without NumPy/OpenCV or
        # malformed legacy preview records.
        pass
    rows = len(matrix)
    bins = len(matrix[0]) if rows else 0
    if not bins:
        return image
    flattened = [float(value) for row in matrix for value in row if math.isfinite(float(value))]
    if not flattened:
        return image
    flattened.sort()
    low = flattened[int(0.05 * (len(flattened) - 1))]
    high = flattened[int(0.99 * (len(flattened) - 1))]
    if high <= low:
        high = low + 1.0
    center_x = width // 2
    origin_y = height - 35
    max_radius = min(center_x - 25, height - 65)
    start_m = float(record.get("range_start_m", 0.0))
    end_m = max(float(record.get("range_end_m", 10.0)), start_m + 1e-6)

    def point(angle, radius):
        return center_x + math.sin(angle) * radius, origin_y - math.cos(angle) * radius

    for beam_index, row in enumerate(matrix):
        angle0 = math.radians(-40.0 + 80.0 * beam_index / max(1, rows))
        angle1 = math.radians(-40.0 + 80.0 * (beam_index + 1) / max(1, rows))
        for range_index, value in enumerate(row):
            radius0 = max_radius * (start_m + (end_m - start_m) * range_index / bins) / end_m
            radius1 = max_radius * (start_m + (end_m - start_m) * (range_index + 1) / bins) / end_m
            normalized = (float(value) - low) / (high - low)
            normalized = max(0.0, min(1.0, (normalized - 0.5) * float(contrast) + 0.5))
            normalized = max(0.0, min(1.0, normalized * float(brightness)))
            fill = heat_color(normalized)
            draw.polygon([point(angle0, radius0), point(angle1, radius0), point(angle1, radius1), point(angle0, radius1)], fill=fill)
    draw.arc((center_x - max_radius, origin_y - max_radius, center_x + max_radius, origin_y + max_radius), 50, 130, fill="#7e9aaa")
    draw.line(point(math.radians(-40), max_radius) + point(math.radians(40), max_radius), fill="#7e9aaa")
    draw.line((center_x, origin_y, center_x, origin_y - max_radius), fill="#526b7c")
    for distance in (end_m * 0.25, end_m * 0.5, end_m * 0.75, end_m):
        radius = max_radius * distance / end_m
        draw.ellipse((center_x - radius, origin_y - radius, center_x + radius, origin_y + radius), outline="#294352")
        draw.text((center_x + 5, origin_y - radius - 14), "%.1f m" % distance, fill="#b4c8d2")
    if show_atof:
        for point_data in record.get("points", []):
            angle = float(point_data.get("angle_rad", 0.0))
            distance = float(point_data.get("distance_m", 0.0))
            radius = max_radius * max(0.0, min(1.0, distance / end_m))
            x, y = point(angle, radius)
            draw.ellipse((x - 5, y - 5, x + 5, y + 5), outline="#ffcb4d", fill="#ff5b4d", width=2)
    draw.text((16, 14), "SURVEYOR FAN IMAGE · ±40° · %s" % record.get("channel_data_status", "--"), fill="#dceaf2")
    draw.text((16, height - 25), "ping %s · %d detections · %.2f..%.2f m" % (record.get("ping_number", "--"), record.get("detection_count", 0), start_m, end_m), fill="#b4c8d2")
    return image


def heat_color(value):
    x = max(0.0, min(1.0, float(value)))
    stops = ((0.0, (3, 10, 35)), (0.25, (10, 57, 130)), (0.5, (0, 180, 210)), (0.75, (255, 190, 50)), (1.0, (255, 250, 220)))
    for (lo, c0), (hi, c1) in zip(stops, stops[1:]):
        if x <= hi:
            fraction = (x - lo) / (hi - lo)
            return tuple(int(c0[i] + fraction * (c1[i] - c0[i])) for i in range(3))
    return stops[-1][1]


class _LegacySonarViewer(tk.Tk):
    """Three-panel live/replay GUI."""

    PROFILE_W = 560
    PROFILE_H = 230

    def __init__(self, offline=False, replay_path=None, wet_authorized=False, skip_surveyor=False):
        super(_LegacySonarViewer, self).__init__()
        self.title("BlueROV2 Multimodal Recorder")
        self.geometry("1600x950")
        self.minsize(1200, 760)
        self.metrics = MetricsRegistry()
        self.events = queue.Queue(maxsize=CONTROL_QUEUE_MAX_ITEMS)
        self.camera_preview_mailbox = LatestValueMailbox(
            self.metrics, "camera_preview_dropped", "camera_preview_published",
        )
        self.surveyor_preview_mailbox = LatestValueMailbox(
            self.metrics, "surveyor_preview_dropped", "surveyor_preview_published",
        )
        self.ping_preview_mailbox = LatestValueMailbox(
            self.metrics, "ping1d_preview_dropped", "ping1d_preview_published",
        )
        self.rovl_preview_mailbox = LatestValueMailbox(
            self.metrics, "rovl_preview_dropped", "rovl_preview_published",
        )
        self.attitude_mailbox = LatestValueMailbox(self.metrics)
        self._last_gui_poll_ns = time.monotonic_ns()
        self._metrics_previous = {}
        self._metrics_previous_ns = self._last_gui_poll_ns
        self.offline = bool(offline)
        self.replay_path = Path(replay_path) if replay_path else None
        self.wet_authorized = bool(wet_authorized)
        self.skip_surveyor = bool(skip_surveyor) and self.replay_path is None
        self.surveyor_worker = None
        self.ping_worker = None
        self.camera_worker = None
        self.blueos_worker = None
        self.session = None
        self._session_close_thread = None
        self._closing_session_path = None
        self.latest_surveyor = None
        self.latest_ping = None
        self.latest_ping_profile = None
        self.latest_camera = None
        self.latest_camera_pil = None
        self.camera_metadata = {
            "backend": "PENDING",
            "recording_mode": "PENDING",
            "video_pts": "UNAVAILABLE",
            "input_status": "PENDING",
            "remux_status": "NOT STARTED",
        }
        self.camera_frame_count = 0
        self.surveyor_ping_count = 0
        self.ping1d_sample_count = 0
        self._last_surveyor_preview_ns = 0
        self._last_ping_preview_ns = 0
        self._last_rovl_preview_ns = 0
        self._last_attitude_preview_ns = 0
        self.photos = {}
        self.show_atof = tk.BooleanVar(value=True)
        self.fan_brightness = tk.DoubleVar(value=1.0)
        self.fan_contrast = tk.DoubleVar(value=1.0)
        self.fan_threshold = tk.DoubleVar(value=0.0)
        self.camera_port = tk.IntVar(value=CAMERA_DEFAULT_PORT)
        self.camera_sdp = tk.StringVar(value="")
        self.status_text = tk.StringVar(value="Pronto — SURVEYOR TX: LOCKED / DRY MODE")
        self._build_ui()
        self.protocol("WM_DELETE_WINDOW", self.close)
        self.after(20, self._poll_events)
        self.after(20, self._poll_previews)
        self.after(1000, self._sample_diagnostics)
        self.after(250, self._refresh_status_age)

    def _build_ui(self):
        top = ttk.Frame(self, padding=8)
        top.pack(fill="x")
        self.blueos_badge = self._badge(top, "BlueOS", BLUEOS_HOST, 0)
        self.camera_badge = self._badge(top, "Camera", "UDP %d / %d" % (CAMERA_DEFAULT_PORT, CAMERA_ALTERNATE_PORT), 1)
        self.surveyor_badge = self._badge(top, "Surveyor Network", "%s:%d" % (SURVEYOR_HOST, SURVEYOR_PORT), 2)
        self.tx_badge = self._badge(top, "Surveyor TX", "manual authorization", 3)
        self.ping_badge = self._badge(top, "Ping1D", "%s:%d" % (PING1D_HOST, PING1D_PORT), 4)
        for column in range(5):
            top.columnconfigure(column, weight=1)
        self._set_badge(self.tx_badge, False, "LOCKED / DRY MODE")
        if self.replay_path is not None:
            self._set_badge(self.surveyor_badge, False, "REPLAY")
        elif self.skip_surveyor:
            self._set_badge(self.surveyor_badge, False, "SKIPPED")

        toolbar = ttk.Frame(self, padding=(8, 0, 8, 8))
        toolbar.pack(fill="x")
        ttk.Button(toolbar, text="Connect all", command=self.connect_all).pack(side="left", padx=2)
        ttk.Button(toolbar, text="Disconnect", command=self.disconnect_all).pack(side="left", padx=2)
        ttk.Button(toolbar, text="START SESSION", command=self.start_session).pack(side="left", padx=8)
        ttk.Button(toolbar, text="STOP SESSION", command=self.stop_session).pack(side="left", padx=2)
        self.start_surveyor_button = ttk.Button(toolbar, text="Start Surveyor (LOCKED)", command=self.start_surveyor)
        self.start_surveyor_button.pack(side="left", padx=8)
        if not self.wet_authorized:
            self.start_surveyor_button.configure(state="disabled")
        ttk.Button(toolbar, text="Screenshot", command=self.save_screenshot).pack(side="left", padx=2)
        ttk.Checkbutton(toolbar, text="Show ATOF", variable=self.show_atof, command=self._rerender_fan).pack(side="left", padx=10)
        ttk.Label(toolbar, textvariable=self.status_text).pack(side="right", padx=4)

        camera_controls = ttk.Frame(self, padding=(8, 0, 8, 5))
        camera_controls.pack(fill="x")
        ttk.Label(camera_controls, text="Camera port").pack(side="left")
        ttk.Combobox(camera_controls, textvariable=self.camera_port, values=(CAMERA_DEFAULT_PORT, CAMERA_ALTERNATE_PORT), width=7, state="readonly").pack(side="left", padx=4)
        ttk.Label(camera_controls, text="SDP locale/URL opzionale").pack(side="left", padx=(12, 2))
        ttk.Entry(camera_controls, textvariable=self.camera_sdp, width=48).pack(side="left")
        ttk.Button(camera_controls, text="Browse", command=self._choose_sdp).pack(side="left", padx=4)
        ttk.Label(camera_controls, text="Fan brightness").pack(side="left", padx=(18, 2))
        ttk.Scale(camera_controls, from_=0.2, to=3.0, variable=self.fan_brightness, command=lambda _value: self._rerender_fan()).pack(side="left", padx=2)
        ttk.Label(camera_controls, text="contrast").pack(side="left", padx=(8, 2))
        ttk.Scale(camera_controls, from_=0.2, to=3.0, variable=self.fan_contrast, command=lambda _value: self._rerender_fan()).pack(side="left", padx=2)
        ttk.Label(camera_controls, text="threshold %").pack(side="left", padx=(8, 2))
        ttk.Scale(camera_controls, from_=0.0, to=100.0, variable=self.fan_threshold, command=lambda _value: self._rerender_fan()).pack(side="left", padx=2)

        panes = ttk.Panedwindow(self, orient="horizontal")
        panes.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        left = ttk.Frame(panes, padding=8)
        center = ttk.Frame(panes, padding=8)
        right = ttk.Frame(panes, padding=8)
        panes.add(left, weight=1)
        panes.add(center, weight=1)
        panes.add(right, weight=1)
        self._build_camera_panel(left)
        self._build_surveyor_panel(center)
        self._build_ping_panel(right)

    def _choose_sdp(self):
        path = filedialog.askopenfilename(title="Seleziona SDP camera", filetypes=[("SDP", "*.sdp"), ("Tutti i file", "*.*")])
        if path:
            self.camera_sdp.set(path)

    def _badge(self, parent, title, address, column):
        frame = ttk.LabelFrame(parent, text=title, padding=6)
        frame.grid(row=0, column=column, sticky="ew", padx=3)
        state = tk.StringVar(value="OFFLINE")
        ttk.Label(frame, text=address).pack(side="left")
        label = ttk.Label(frame, textvariable=state, foreground="#b00020")
        label.pack(side="right")
        return {"state": state, "label": label}

    def _build_camera_panel(self, parent):
        ttk.Label(parent, text="RGB CAMERA LIVE", font=("Segoe UI", 14, "bold")).pack(anchor="w")
        self.camera_label = ttk.Label(parent, text="Nessun frame camera — selezionare UDP 5600/5602 o un SDP")
        self.camera_label.pack(fill="both", expand=True, pady=(8, 0))
        self.camera_stats = tk.StringVar(value="1920×1080 · 30 fps attesi · single ingest")
        ttk.Label(parent, textvariable=self.camera_stats, anchor="w").pack(fill="x", pady=(5, 0))

    def _build_surveyor_panel(self, parent):
        ttk.Label(parent, text="SURVEYOR FAN IMAGE", font=("Segoe UI", 14, "bold")).pack(anchor="w")
        self.fan_label = ttk.Label(parent, text="Nessun dato — Connect all oppure --replay-surveyor")
        self.fan_label.pack(fill="both", expand=True, pady=(8, 0))
        self.surveyor_stats = tk.StringVar(value="CHANNEL DATA: -- | ping: -- | rate: -- | range: -- | detections: --")
        ttk.Label(parent, textvariable=self.surveyor_stats, anchor="w").pack(fill="x", pady=(5, 0))
        self.attitude_stats = tk.StringVar(value="Attitude: --")
        ttk.Label(parent, textvariable=self.attitude_stats, anchor="w").pack(fill="x")

    def _build_ping_panel(self, parent):
        ttk.Label(parent, text="PING1D", font=("Segoe UI", 14, "bold")).pack(anchor="w")
        self.distance_label = ttk.Label(parent, text="DISTANCE: —", font=("Segoe UI", 25, "bold"))
        self.distance_label.pack(anchor="w", pady=(10, 0))
        self.confidence_label = ttk.Label(parent, text="CONFIDENCE: — %", font=("Segoe UI", 18, "bold"))
        self.confidence_label.pack(anchor="w")
        self.ping_profile_label = ttk.Label(parent, text="Profilo echi completo non ancora ricevuto")
        self.ping_profile_label.pack(fill="both", expand=True, pady=(15, 0))
        self.ping_stats = tk.StringVar(value="profile_data: -- | scan range: -- | gain: --")
        ttk.Label(parent, textvariable=self.ping_stats, anchor="w").pack(fill="x", pady=(5, 0))

    def _set_badge(self, badge, online, detail=None):
        text = "ONLINE" if online else "OFFLINE"
        if detail:
            text += " — " + str(detail)
        badge["state"].set(text)
        badge["label"].configure(foreground="#087f23" if online else "#b00020")

    def report_callback_exception(self, exc_type, exc_value, exc_traceback):
        """Keep Tk callback failures visible and persistent."""
        self.metrics.increment("tk_callback_exceptions")
        LOGGER.critical(
            "Unhandled Tk callback exception",
            exc_info=(exc_type, exc_value, exc_traceback),
        )
        try:
            self.status_text.set("Errore GUI: %s" % exc_value)
        except Exception:
            LOGGER.exception("Unable to display Tk callback failure")

    def _attach_camera_session(self):
        """Connect the active session to an existing or newly-created camera."""
        if self.camera_worker is not None and self.session is not None:
            self.camera_worker.set_packet_callback(self.session.write_camera_packet)
            self.camera_worker.set_frame_callback(self.session.write_camera_frame)

    def _detach_camera_session(self):
        """Detach before closing a session; CameraWorker makes this thread-safe."""
        if self.camera_worker is not None:
            self.camera_worker.set_packet_callback(None)
            self.camera_worker.set_frame_callback(None)

    def _attach_sensor_session(self):
        if self.session is None:
            return
        if getattr(self, "surveyor_worker", None) is not None:
            self.surveyor_worker.set_recording_callbacks(
                self.session.write_surveyor_packet,
                self.session.write_surveyor_ping,
            )
        if getattr(self, "ping_worker", None) is not None:
            self.ping_worker.set_record_callback(self.session.write_ping1d)
        if getattr(self, "rovl_worker", None) is not None and hasattr(self.rovl_worker, "set_record_callback"):
            self.rovl_worker.set_record_callback(self.session.write_rovl_sample)

    def _detach_sensor_session(self):
        if getattr(self, "surveyor_worker", None) is not None:
            self.surveyor_worker.set_recording_callbacks(None, None)
        if getattr(self, "ping_worker", None) is not None:
            self.ping_worker.set_record_callback(None)
        if getattr(self, "rovl_worker", None) is not None and hasattr(self.rovl_worker, "set_record_callback"):
            self.rovl_worker.set_record_callback(None)

    def connect_all(self):
        if self.offline:
            if getattr(self, "demo_rovl", False) and self.rovl_worker is None:
                self.rovl_worker = DemoROVLWorker(self.events)
                self.rovl_worker.start()
            if self.replay_path is not None and self.surveyor_worker is None:
                self.surveyor_worker = SurveyorWorker(
                    SURVEYOR_HOST, SURVEYOR_PORT, self.events, dry_mode=True,
                    wet_authorized=False, replay_path=self.replay_path,
                    preview_mailbox=self.surveyor_preview_mailbox,
                    attitude_mailbox=self.attitude_mailbox, metrics=self.metrics,
                    raw_callback=self.session.write_surveyor_packet if self.session else None,
                    decoded_callback=self.session.write_surveyor_ping if self.session else None,
                )
                self.surveyor_worker.start()
                self.status_text.set("Offline replay avviato — nessuna connessione hardware")
            else:
                self.status_text.set("Offline: nessuna connessione hardware avviata")
            if self.skip_surveyor:
                self._set_badge(self.surveyor_badge, False, "SKIPPED")
            self._attach_sensor_session()
            return
        if not getattr(self, "surveyor_only", False) and self.blueos_worker is None:
            self.blueos_worker = BlueOSWorker(BLUEOS_HOST, self.events)
            self.blueos_worker.start()
        if self.surveyor_worker is None and self.replay_path is not None:
            self.surveyor_worker = SurveyorWorker(
                SURVEYOR_HOST, SURVEYOR_PORT, self.events,
                dry_mode=not self.wet_authorized,
                wet_authorized=self.wet_authorized, replay_path=self.replay_path,
                preview_mailbox=self.surveyor_preview_mailbox,
                attitude_mailbox=self.attitude_mailbox, metrics=self.metrics,
                raw_callback=self.session.write_surveyor_packet if self.session else None,
                decoded_callback=self.session.write_surveyor_ping if self.session else None,
            )
            self.surveyor_worker.start()
        elif self.surveyor_worker is None and not self.skip_surveyor:
            self.surveyor_worker = SurveyorWorker(
                SURVEYOR_HOST, SURVEYOR_PORT, self.events,
                dry_mode=not self.wet_authorized,
                wet_authorized=self.wet_authorized,
                preview_mailbox=self.surveyor_preview_mailbox,
                attitude_mailbox=self.attitude_mailbox, metrics=self.metrics,
                raw_callback=self.session.write_surveyor_packet if self.session else None,
                decoded_callback=self.session.write_surveyor_ping if self.session else None,
            )
            self.surveyor_worker.start()
        if not getattr(self, "surveyor_only", False) and self.ping_worker is None:
            self.ping_worker = Ping1DWorker(
                PING1D_HOST, PING1D_PORT, self.events,
                preview_mailbox=self.ping_preview_mailbox,
                metrics=self.metrics,
                record_callback=self.session.write_ping1d if self.session else None,
            )
            self.ping_worker.start()
        if not getattr(self, "surveyor_only", False) and self.camera_worker is None:
            source = self.camera_sdp.get().strip() or None
            self.camera_worker = CameraWorker(
                self.camera_port.get(), source, self.events,
                preview_mailbox=self.camera_preview_mailbox,
                metrics=self.metrics,
            )
            self._attach_camera_session()
            self.camera_worker.start()
        if not getattr(self, "surveyor_only", False) and self.rovl_worker is None:
            self.rovl_worker = ROVLWorker(
                self.rovl_port.get().strip() or "Auto", self.events,
                preview_mailbox=self.rovl_preview_mailbox,
                metrics=self.metrics,
                record_callback=self.session.write_rovl_sample if self.session else None,
            )
            self.rovl_worker.start()
        self._attach_sensor_session()
        mode = "REPLAY" if self.replay_path is not None else ("SKIPPED" if self.skip_surveyor else ("TX: LOCKED / DRY MODE" if not self.wet_authorized else "LIVE"))
        self.status_text.set("Connessioni avviate — SURVEYOR %s" % mode)

    def start_surveyor(self):
        if not self.wet_authorized:
            messagebox.showwarning("Surveyor TX bloccato", "Il Surveyor è in DRY MODE: nessuna trasmissione acustica è autorizzata.")
            return
        if self.surveyor_worker is None:
            messagebox.showwarning("Surveyor non connesso", "Premere Connect all prima di avviare il Surveyor.")
            return
        try:
            self.surveyor_worker.request_start({"range_m": 20.0, "ping_rate_hz": 5.0, "n_range_steps": 400})
        except Exception as exc:
            messagebox.showerror("Surveyor", str(exc))

    def disconnect_all(self):
        self._detach_camera_session()
        if self.surveyor_worker is not None:
            self.surveyor_worker.disconnect()
        if self.ping_worker is not None:
            self.ping_worker.stop()
        if self.camera_worker is not None:
            self.camera_worker.stop()
        if self.blueos_worker is not None:
            self.blueos_worker.stop_event.set()
        if getattr(self, "rovl_worker", None) is not None:
            self.rovl_worker.stop()
        self.status_text.set("Disconnessione richiesta — TX rimane LOCKED")

    def start_session(self):
        if self.session is not None:
            return
        if self._session_close_thread is not None and self._session_close_thread.is_alive():
            self.status_text.set("Attendere la finalizzazione della sessione precedente")
            return
        if getattr(self, "demo_rovl", False):
            self.status_text.set("Synthetic demo is display-only; recording remains disabled")
            return
        try:
            surveyor_mode = "replay" if self.replay_path is not None else ("skipped" if self.skip_surveyor else "live")
            self.camera_frame_count = 0
            self.surveyor_ping_count = 0
            self.ping1d_sample_count = 0
            self.rovl_fix_count = 0
            rovl_port = getattr(getattr(self, "rovl_worker", None), "port", None) if getattr(self, "rovl_connected", False) else None
            self.session = SessionRecorder(
                SESSION_ROOT, self.camera_port.get(), self.camera_sdp.get().strip() or None,
                surveyor_mode, self.replay_path,
                rovl_connected=getattr(self, "rovl_connected", False),
                rovl_port=rovl_port,
                rovl_synthetic=getattr(self, "rovl_synthetic", False),
                surveyor_only=getattr(self, "surveyor_only", False),
                metrics=self.metrics,
                critical_callback=lambda reason: publish_control_event(
                    self.events, ("critical_data_loss", reason), self.metrics,
                    critical=True,
                ),
                surveyor_tx="AUTHORIZED" if self.wet_authorized else "LOCKED",
            )
            if not getattr(self, "surveyor_only", False):
                self._attach_camera_session()
            self._attach_sensor_session()
            if not getattr(self, "surveyor_only", False):
                self.session.update_camera_metadata(self.camera_metadata)
            self.session.write_event("session_started", {"tx": "LOCKED" if not self.wet_authorized else "manual-authorized"})
            if "record_status" in self.__dict__:
                self.record_status.set("●  RECORDING")
                self.record_label.configure(bg="#0d633d", fg="#eafff4")
            self.status_text.set("Registrazione sessione: %s" % self.session.directory)
        except Exception as exc:
            messagebox.showerror("Sessione", "Impossibile avviare la registrazione: %s" % exc)

    def stop_session(self):
        if self.session is None:
            return
        session = self.session
        self.session = None
        self._detach_camera_session()
        self._detach_sensor_session()
        session.write_event("session_stopped")
        self._closing_session_path = session.directory

        def finalize():
            error = None
            try:
                session.close()
            except Exception as exc:
                error = str(exc)
            publish_control_event(
                self.events, ("session_closed", {
                    "directory": str(session.directory), "error": error,
                }), self.metrics, critical=True,
            )

        self._session_close_thread = threading.Thread(
            target=finalize, name="session-finalizer", daemon=False,
        )
        self._session_close_thread.start()
        if "record_status" in self.__dict__:
            self.record_status.set("●  FINALIZING")
            self.record_label.configure(bg="#402126", fg="#ff777a")
        self.status_text.set("Finalizzazione sessione: %s" % session.directory)

    def _consume_rovl_sample(self, data):
        self.latest_rovl_sample = data
        parsed = data.get("parsed") or {}
        if parsed.get("message_type") == "USRTH":
            position = data.get("position") or {}
            if position.get("lock"):
                self.rovl_fix_count += 1
                self.rovl_trail.append(position)
                self.rovl_trail = self.rovl_trail[-60:]
            self.rovl_sample_times.append(int(data.get("host_monotonic_ns") or time.monotonic_ns()))
            self.rovl_sample_times = self.rovl_sample_times[-12:]
            self._update_rovl(data)

    def _poll_previews(self):
        """Consume only latest display values; never drain acquisition data."""
        now_ns = time.monotonic_ns()
        heartbeat_ms = (now_ns - self._last_gui_poll_ns) / 1_000_000.0
        self._last_gui_poll_ns = now_ns
        self.metrics.set("gui_heartbeat_ms", heartbeat_ms)
        self.metrics.set("gui_heartbeat_lag_ms", max(0.0, heartbeat_ms - 20.0))
        try:
            item = self.camera_preview_mailbox.take()
            if item is not None:
                _sequence, (frame, frame_metadata) = item
                self.latest_camera = frame
                self.metrics.increment("camera_frames_previewed")
                self._update_camera(frame)
                self.camera_metadata["last_frame"] = frame_metadata
            item = self.surveyor_preview_mailbox.take() if now_ns - self._last_surveyor_preview_ns >= 200_000_000 else None
            if item is not None:
                _sequence, record = item
                self.latest_surveyor = record
                self.surveyor_ping_count += 1
                self.metrics.increment("surveyor_gui_frames")
                self._update_surveyor(record)
                self._last_surveyor_preview_ns = now_ns
            item = self.attitude_mailbox.take() if now_ns - self._last_attitude_preview_ns >= 100_000_000 else None
            if item is not None:
                _sequence, data = item
                if data:
                    self.attitude_stats.set(
                        "Attitude up vector: %.3f, %.3f, %.3f" % (
                            data.get("up_vec_x", 0), data.get("up_vec_y", 0),
                            data.get("up_vec_z", 0),
                        )
                    )
                    self._last_attitude_preview_ns = now_ns
            item = self.ping_preview_mailbox.take() if now_ns - self._last_ping_preview_ns >= 100_000_000 else None
            if item is not None:
                _sequence, (distance, profile) = item
                self.ping1d_sample_count += 1
                self.latest_ping = distance
                self.latest_ping_profile = profile or self.latest_ping_profile
                self.metrics.increment("ping1d_gui_frames")
                self._update_ping(distance, profile)
                self._last_ping_preview_ns = now_ns
            item = self.rovl_preview_mailbox.take() if now_ns - self._last_rovl_preview_ns >= 100_000_000 else None
            if item is not None:
                _sequence, data = item
                self.metrics.increment("rovl_gui_frames")
                self._consume_rovl_sample(data)
                self._last_rovl_preview_ns = now_ns
        except Exception as exc:
            self.metrics.increment("gui_preview_errors")
            LOGGER.exception("Preview update failure")
            publish_control_event(self.events, ("gui_preview_error", str(exc)), self.metrics)
        finally:
            self.after(20, self._poll_previews)

    def _sample_diagnostics(self):
        now_ns = time.monotonic_ns()
        snapshot = self.metrics.snapshot()
        elapsed = max(1e-6, (now_ns - self._metrics_previous_ns) / 1_000_000_000.0)
        rate_names = (
            "camera_packets_received", "camera_frames_decoded",
            "camera_frames_previewed", "surveyor_packets_received",
            "surveyor_ping_records", "surveyor_complete_channel_pings",
            "ping1d_samples_received", "rovl_samples_received",
        )
        for name in rate_names:
            current = float(snapshot.get(name, 0.0))
            previous = float(self._metrics_previous.get(name, 0.0))
            snapshot[name + "_per_s"] = (current - previous) / elapsed
        try:
            import psutil
            process = psutil.Process(os.getpid())
            snapshot["process_cpu_percent"] = process.cpu_percent(None)
            snapshot["process_rss_mb"] = process.memory_info().rss / (1024.0 * 1024.0)
        except Exception:
            snapshot.setdefault("process_cpu_percent", None)
            snapshot.setdefault("process_rss_mb", None)
        self._metrics_previous = dict(snapshot)
        self._metrics_previous_ns = now_ns
        if self.session is not None:
            try:
                self.session.write_diagnostics(snapshot)
            except Exception as exc:
                publish_control_event(
                    self.events, ("critical_data_loss", str(exc)), self.metrics,
                    critical=True,
                )
        if hasattr(self, "performance_status"):
            self.performance_status.set(
                "CAM %.1f pkt/s | decode %.1f fps | GUI %.1f fps | drops %d    "
                "SV %.1f pkt/s | complete %.1f/s | raw q %.1f MB | app drops %d    "
                "CPU %s | RAM %s MB | GUI lag %.1f ms" % (
                    snapshot.get("camera_packets_received_per_s", 0.0),
                    snapshot.get("camera_frames_decoded_per_s", 0.0),
                    snapshot.get("camera_frames_previewed_per_s", 0.0),
                    int(snapshot.get("camera_preview_dropped", 0)),
                    snapshot.get("surveyor_packets_received_per_s", 0.0),
                    snapshot.get("surveyor_complete_channel_pings_per_s", 0.0),
                    float(snapshot.get("surveyor_raw_queue_bytes", 0.0)) / (1024 * 1024),
                    int(snapshot.get("surveyor_raw_app_raw_drop", 0)),
                    "--" if snapshot.get("process_cpu_percent") is None else "%.0f%%" % snapshot["process_cpu_percent"],
                    "--" if snapshot.get("process_rss_mb") is None else "%.0f" % snapshot["process_rss_mb"],
                    float(snapshot.get("gui_heartbeat_lag_ms", 0.0)),
                )
            )
        self.after(1000, self._sample_diagnostics)

    def _poll_events(self):
        processed = 0
        deadline_ns = time.perf_counter_ns() + 5_000_000
        try:
            while processed < 64 and time.perf_counter_ns() < deadline_ns:
                kind, data = self.events.get_nowait()
                processed += 1
                if self.session is not None and kind not in ("camera_frame", "camera_frame_metadata", "surveyor_raw_packet", "rovl_sample"):
                    try:
                        self.session.write_event(kind, data if kind not in ("surveyor_ping", "ping_sample") else None)
                    except Exception as exc:
                        self.metrics.increment("event_recording_errors")
                        self.status_text.set("Errore registrazione evento: %s" % exc)
                if kind == "blueos":
                    self._set_badge(self.blueos_badge, data)
                elif kind == "camera_connected":
                    self._set_badge(self.camera_badge, True, "%s · UDP %s" % (data.get("backend", "camera"), data.get("port", "?")))
                elif kind == "camera_backend":
                    self.camera_metadata = dict(data or {})
                    self._set_badge(self.camera_badge, True, self.camera_metadata.get("backend", "UNKNOWN"))
                    if self.session is not None:
                        self.session.update_camera_metadata(self.camera_metadata)
                elif kind == "camera_frame":
                    frame, frame_metadata = data
                    self.latest_camera = frame
                    if self.session is not None:
                        self.session.write_camera_frame(frame, frame_metadata)
                        self.camera_metadata.update(self.session.camera_metadata)
                    self._update_camera(frame)
                elif kind == "camera_frame_metadata":
                    if self.session is not None:
                        self.session.write_camera_frame(None, data)
                elif kind == "camera_error":
                    self._set_badge(self.camera_badge, False, data)
                elif kind == "camera_closed":
                    if self.camera_worker is not None:
                        self.camera_worker.set_packet_callback(None)
                        self.camera_worker.set_frame_callback(None)
                    self.camera_worker = None
                    self._set_badge(self.camera_badge, False)
                elif kind == "surveyor_connected":
                    self._set_badge(self.surveyor_badge, True, "REPLAY" if data.get("replay") else "PASSIVE")
                elif kind == "surveyor_ping":
                    self.latest_surveyor = data
                    self.surveyor_ping_count += 1
                    if self.session is not None:
                        self.session.write_surveyor_ping(data)
                    now_ns = time.monotonic_ns()
                    preview_interval_ns = int(1_000_000_000 / SURVEYOR_PREVIEW_FPS)
                    if now_ns - self._last_surveyor_preview_ns >= preview_interval_ns:
                        self._last_surveyor_preview_ns = now_ns
                        self._update_surveyor(data)
                elif kind == "surveyor_attitude":
                    if data:
                        self.attitude_stats.set("Attitude up vector: %.3f, %.3f, %.3f" % (data.get("up_vec_x", 0), data.get("up_vec_y", 0), data.get("up_vec_z", 0)))
                elif kind == "surveyor_started":
                    self._set_badge(self.tx_badge, True, "ACTIVE")
                elif kind == "surveyor_stopped":
                    self._set_badge(
                        self.tx_badge, False,
                        "AUTHORIZED / READY" if self.wet_authorized else "LOCKED (DRY MODE)",
                    )
                elif kind == "surveyor_error":
                    self._set_badge(self.surveyor_badge, False, data)
                    self.status_text.set("Errore Surveyor: %s" % data)
                elif kind in ("surveyor_closed", "surveyor_replay_closed"):
                    self.surveyor_worker = None
                    self._set_badge(self.surveyor_badge, False)
                elif kind == "ping_connected":
                    self._set_badge(self.ping_badge, True)
                elif kind == "ping_sample":
                    distance, profile = data
                    self.ping1d_sample_count += 1
                    self.latest_ping = distance
                    self.latest_ping_profile = profile or self.latest_ping_profile
                    if self.session is not None:
                        self.session.write_ping1d({"distance": distance, "profile": profile})
                    self._update_ping(distance, profile)
                elif kind == "ping_warning":
                    self._set_badge(self.ping_badge, True, "no data")
                elif kind == "ping_error":
                    self._set_badge(self.ping_badge, False, data)
                elif kind == "ping_closed":
                    self.ping_worker = None
                    self._set_badge(self.ping_badge, False)
                elif kind == "rovl_connected":
                    self.rovl_connected = True
                    self.rovl_synthetic = bool(data.get("synthetic"))
                    self.rovl_badge["address"].set(str(data.get("port") or "USB COM"))
                    self._set_badge(self.rovl_badge, True, "SYNTHETIC" if self.rovl_synthetic else "CONNECTED")
                    if self.session is not None and not self.rovl_synthetic:
                        self.session.enable_rovl(data.get("port"), synthetic=False)
                elif kind == "rovl_sample":
                    if self.session is not None and not data.get("synthetic"):
                        self.session.enable_rovl(getattr(self.rovl_worker, "port", None), synthetic=False)
                        self.session.write_rovl_sample(data)
                    self._consume_rovl_sample(data)
                elif kind == "rovl_lock_acquired":
                    self.rovl_lock = True
                elif kind == "rovl_lock_lost":
                    self.rovl_lock = False
                elif kind == "rovl_unavailable":
                    self.rovl_worker = None
                    self.rovl_connected = False
                    self._set_badge(self.rovl_badge, False, "NOT AVAILABLE")
                    self.rovl_status.set("NOT AVAILABLE (optional)")
                elif kind in ("rovl_checksum_error", "rovl_parser_error"):
                    self.rovl_status.set("RECEIVING · DATA WARNING")
                elif kind == "rovl_serial_error":
                    self.rovl_connected = False
                    self._set_badge(self.rovl_badge, False, "SERIAL ERROR")
                    self.rovl_status.set("SERIAL ERROR")
                elif kind == "rovl_disconnected":
                    self.rovl_worker = None
                    self.rovl_connected = False
                    self.rovl_lock = False
                    self._set_badge(self.rovl_badge, False, "DISCONNECTED")
                    if self.session is not None:
                        self.session.mark_rovl_disconnected()
                elif kind == "critical_data_loss":
                    self.metrics.increment("critical_data_loss_events")
                    self.status_text.set("CRITICAL DATA LOSS — sessione degradata: %s" % data)
                    if self.session is not None:
                        self.session.mark_degraded(str(data))
                        self.stop_session()
                elif kind == "session_closed":
                    if "record_status" in self.__dict__:
                        self.record_status.set("●  NOT RECORDING")
                    if data.get("error"):
                        self.status_text.set("Sessione chiusa con errori: %s" % data["error"])
                    else:
                        self.status_text.set("Sessione salvata: %s" % data["directory"])
                elif kind in ("performance_warning", "surveyor_processing_error", "gui_preview_error"):
                    self.status_text.set("Warning prestazioni: %s" % data)
        except queue.Empty:
            pass
        except Exception as exc:
            self.metrics.increment("gui_control_errors")
            LOGGER.exception("Control event handler failure")
            self.status_text.set("Errore GUI control queue: %s" % exc)
        finally:
            self.metrics.set("control_events_processed_last_tick", processed)
            self.metrics.set("control_queue_depth", self.events.qsize())
            self.after(20, self._poll_events)

    def _update_camera(self, frame):
        if cv2 is None:
            return
        try:
            image = Image.fromarray(frame[:, :, ::-1])
            self.latest_camera_pil = image.copy()
            image.thumbnail((560, 520), Image.Resampling.LANCZOS)
            self.photos["camera"] = ImageTk.PhotoImage(image)
            self.camera_label.configure(image=self.photos["camera"], text="")
            backend = self.camera_metadata.get("backend", "PENDING")
            recording = self.camera_metadata.get("recording_mode", "PENDING")
            remux = self.camera_metadata.get("remux_status", "NOT STARTED")
            input_status = self.camera_metadata.get("input_status", backend)
            pts = self.camera_metadata.get("video_pts", "UNAVAILABLE")
            self.camera_frame_count += 1
            self.camera_stats.set("%d×%d · %s · %s · %s · REMUX: %s · Video PTS: %s" % (frame.shape[1], frame.shape[0], backend, input_status, recording, remux, pts))
        except Exception:
            self.metrics.increment("camera_gui_errors")
            LOGGER.exception("Camera GUI conversion failure")

    def _update_surveyor(self, record):
        self._rerender_fan()
        self.surveyor_stats.set("CHANNEL DATA: %s | ping: %s | rate: %.2f Hz | range: %.2f..%.2f m | detections: %d" % (record.get("channel_data_status", "--"), record.get("ping_number", "--"), record.get("ping_rate_hz") or 0.0, record.get("range_start_m", 0.0), record.get("range_end_m", 0.0), record.get("detection_count", 0)))

    def _rerender_fan(self):
        if self.latest_surveyor is None:
            return
        record = dict(self.latest_surveyor)
        image = fan_image(record, brightness=self.fan_brightness.get(), contrast=self.fan_contrast.get(), show_atof=self.show_atof.get())
        image.thumbnail((720, 650), Image.Resampling.LANCZOS)
        self.photos["fan"] = ImageTk.PhotoImage(image)
        self.fan_label.configure(image=self.photos["fan"], text="")

    def _update_ping(self, distance, profile):
        self.distance_label.configure(text="DISTANCE: %.2f m" % distance["distance_m"], foreground="#087f23")
        self.confidence_label.configure(text="CONFIDENCE: %d %%" % distance["confidence"])
        if not profile:
            return
        plot = self._plot_profile(profile["profile"], profile["scan_start_mm"] / 1000.0, (profile["scan_start_mm"] + profile["scan_length_mm"]) / 1000.0, profile["distance_m"], "Ping1D full profile_data", "return strength")
        self.photos["ping"] = ImageTk.PhotoImage(plot)
        self.ping_profile_label.configure(image=self.photos["ping"], text="")
        self.ping_stats.set("profile_data: %d campioni | scan range: %.2f..%.2f m | gain: %d" % (len(profile["profile"]), profile["scan_start_mm"] / 1000.0, (profile["scan_start_mm"] + profile["scan_length_mm"]) / 1000.0, profile["gain"]))

    def _plot_profile(self, values, x_min, x_max, marker, title, y_label):
        image = Image.new("RGB", (self.PROFILE_W, self.PROFILE_H), "#101820")
        draw = ImageDraw.Draw(image)
        left, top, right, bottom = 52, 26, self.PROFILE_W - 12, self.PROFILE_H - 28
        draw.text((8, 5), title, fill="white")
        draw.line((left, top, left, bottom), fill="#aaaaaa")
        draw.line((left, bottom, right, bottom), fill="#aaaaaa")
        values = list(values or [])
        if not values:
            draw.text((left + 10, top + 10), "nessun profilo", fill="#dddddd")
            return image
        low, high = min(values), max(values)
        if high <= low:
            high = low + 1.0
        points = []
        for index, value in enumerate(values):
            fraction = 0.0 if len(values) == 1 else float(index) / (len(values) - 1)
            points.append((left + fraction * (right - left), bottom - (float(value) - low) / (high - low) * (bottom - top)))
        if len(points) > 1:
            draw.line(points, fill="#38d9ff", width=2)
        if marker is not None and x_max > x_min:
            marker_x = left + max(0.0, min(1.0, (marker - x_min) / (x_max - x_min))) * (right - left)
            draw.line((marker_x, top, marker_x, bottom), fill="#ffcc33", width=2)
            draw.text((max(left, marker_x - 24), top + 3), "%.2fm" % marker, fill="#ffcc33")
        draw.text((left, bottom + 5), "%.2f m" % x_min, fill="#cccccc")
        draw.text((right - 48, bottom + 5), "%.2f m" % x_max, fill="#cccccc")
        draw.text((4, bottom - 12), y_label, fill="#cccccc")
        return image

    def _refresh_status_age(self):
        if self.latest_surveyor:
            age = max(0.0, (time.monotonic_ns() - int(self.latest_surveyor.get("host_monotonic_ns", time.monotonic_ns()))) / 1_000_000_000.0)
            mode = "REPLAY" if self.replay_path is not None else ("SKIPPED" if self.skip_surveyor else ("ACTIVE" if self.wet_authorized else "LOCKED / DRY MODE"))
            session = ""
            if self.session is not None:
                elapsed = (time.monotonic_ns() - self.session.session_start_monotonic_ns) / 1_000_000_000.0
                session = " · session %s %.1fs · frames %d · Surveyor pings %d · Ping1D %d" % (self.session.session_id, elapsed, self.camera_frame_count, self.surveyor_ping_count, self.ping1d_sample_count)
            self.status_text.set("SURVEYOR: %s · last ping %.1fs ago%s" % (mode, age, session))
        elif self.session is not None:
            elapsed = (time.monotonic_ns() - self.session.session_start_monotonic_ns) / 1_000_000_000.0
            self.status_text.set("SESSION %s %.1fs · frames %d · Surveyor pings %d · Ping1D %d" % (self.session.session_id, elapsed, self.camera_frame_count, self.surveyor_ping_count, self.ping1d_sample_count))
        self.after(250, self._refresh_status_age)

    def _snapshot_image(self):
        canvas = Image.new("RGB", (1500, 850), "#202830")
        draw = ImageDraw.Draw(canvas)
        draw.text((20, 12), "BlueROV2 Multimodal Recorder · %s" % utc_iso(), fill="white")
        if self.latest_camera_pil is not None:
            camera = self.latest_camera_pil.copy()
            camera.thumbnail((470, 500), Image.Resampling.LANCZOS)
            canvas.paste(camera, (20, 45))
        if self.latest_surveyor is not None:
            fan = fan_image(self.latest_surveyor, 650, 600, self.fan_brightness.get(), self.fan_contrast.get(), self.show_atof.get())
            canvas.paste(fan, (510, 45))
        if self.latest_ping_profile:
            canvas.paste(self._plot_profile(self.latest_ping_profile["profile"], self.latest_ping_profile["scan_start_mm"] / 1000.0, (self.latest_ping_profile["scan_start_mm"] + self.latest_ping_profile["scan_length_mm"]) / 1000.0, self.latest_ping_profile["distance_m"], "Ping1D profile", "return"), (1170, 45))
        return canvas

    def save_screenshot(self):
        path = filedialog.asksaveasfilename(title="Save multimodal screenshot", defaultextension=".png", filetypes=[("PNG", "*.png")], initialfile="multimodal_%s.png" % datetime.now().strftime("%Y%m%d_%H%M%S"))
        if path:
            self._snapshot_image().save(path)
            self.status_text.set("Screenshot salvato: %s" % os.path.basename(path))

    def close(self):
        self.stop_session()
        self.disconnect_all()
        stuck = []
        for worker, timeout in ((self.surveyor_worker, 10.0), (self.ping_worker, 5.0), (self.camera_worker, 5.0), (self.blueos_worker, 2.0)):
            if worker is not None:
                worker.join(timeout=timeout)
                if worker.is_alive():
                    stuck.append(worker.name)
        if self._session_close_thread is not None:
            self._session_close_thread.join(timeout=15.0)
            if self._session_close_thread.is_alive():
                stuck.append(self._session_close_thread.name)
        if stuck:
            self.metrics.increment("shutdown_workers_stuck", len(stuck))
        self.destroy()

    def destroy(self):
        """Dispose Tk-owned objects on the UI thread before later GC cycles."""
        photos = self.__dict__.get("photos")
        if isinstance(photos, dict):
            photos.clear()
        super().destroy()

        def detach(value):
            if isinstance(value, tk.Variable):
                value._tk = None
            elif isinstance(value, dict):
                for nested in value.values():
                    detach(nested)
            elif isinstance(value, (list, tuple)):
                for nested in value:
                    detach(nested)

        for value in tuple(self.__dict__.values()):
            detach(value)
        gc.collect()


class SonarViewer(_LegacySonarViewer):
    """Dark four-panel dashboard with optional passive ROVL tracking."""

    BG = "#06141c"
    BAR = "#071a24"
    CARD = "#071922"
    CARD_ALT = "#091e28"
    BORDER = "#285368"
    TEXT = "#e9f2f7"
    MUTED = "#a9bdd0"
    CYAN = "#35aef3"
    GREEN = "#42e59a"
    AMBER = "#ffbd3a"
    RED = "#ff6268"

    def __init__(self, offline=False, replay_path=None, wet_authorized=False, skip_surveyor=False, demo_rovl=False, rovl_port="Auto", surveyor_only=False):
        self.surveyor_only = bool(surveyor_only)
        self.demo_rovl = bool(demo_rovl)
        self.initial_rovl_port = rovl_port or "Auto"
        self.rovl_worker = None
        self.rovl_connected = False
        self.rovl_synthetic = False
        self.rovl_lock = False
        self.latest_rovl_sample = None
        self.rovl_trail = []
        self.rovl_fix_count = 0
        self.rovl_sample_times = []
        self._demo_tick = 0
        super().__init__(offline=offline or self.demo_rovl, replay_path=replay_path, wet_authorized=wet_authorized, skip_surveyor=skip_surveyor)
        self.title("BlueROV2 Surveyor Recorder" if self.surveyor_only else "BlueROV2 Multimodal Recorder")
        self.geometry("1200x900" if self.surveyor_only else "1600x1000")
        self.rovl_port.set(self.initial_rovl_port)
        if not self.surveyor_only:
            self._refresh_rovl_ports()
        if self.demo_rovl:
            self.start_session_button.configure(state="disabled")
            self.after(80, self._prime_demo_dashboard)
            self.after(180, self.connect_all)

    def _build_ui(self):
        self.configure(bg=self.BG)
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("Dashboard.TCombobox", fieldbackground="#091c26", background="#112a38", foreground=self.TEXT, arrowcolor=self.TEXT)
        style.map("Dashboard.TCombobox", fieldbackground=[("readonly", "#091c26")], foreground=[("readonly", self.TEXT)])
        style.configure("Dashboard.Horizontal.TScale", background=self.CARD, troughcolor="#1a3543")

        self.rovl_port = tk.StringVar(value=self.initial_rovl_port)
        self.rovl_status = tk.StringVar(value="NOT CONNECTED")
        self.rovl_connection_text = tk.StringVar(value="COM port       --\nStatus         NOT CONNECTED\nAcoustic lock  ○ NO")
        self.rovl_position_text = tk.StringVar(value="Slant range        --\nHorizontal range   --\nBearing            --\nElevation          --")
        self.rovl_attitude_text = tk.StringVar(value="Heading     --\nRoll        --\nPitch       --")
        self.rovl_health_text = tk.StringVar(value="IMU          --\nGain         --\nIDs          --\nMethod       --\nUpdate age   --\nUpdate rate  --")
        self.record_status = tk.StringVar(value="●  NOT RECORDING")
        self.session_bar = tk.StringVar(value="Session:   –")
        self.counter_bar = tk.StringVar(value="Frames (cam):  0     Surveyor pings:  0     Ping1D samples:  0     ROVL fixes:  0")
        self.disk_bar = tk.StringVar(value="Disk:  -- GB free")
        self.clock_bar = tk.StringVar(value="")
        self.performance_status = tk.StringVar(
            value="CAM -- | SV -- | queues -- | CPU -- | RAM -- | GUI lag --"
        )
        self.ping_history = []

        titlebar = tk.Frame(self, bg="#081b25", height=38)
        titlebar.pack(fill="x")
        titlebar.pack_propagate(False)
        tk.Label(titlebar, text="◉", bg="#081b25", fg=self.CYAN, font=("Segoe UI", 15, "bold")).pack(side="left", padx=(12, 7))
        tk.Label(titlebar, text="BlueROV2 Multimodal Recorder", bg="#081b25", fg=self.TEXT, font=("Segoe UI", 12, "bold")).pack(side="left")
        tk.Label(titlebar, text="v0.1.0", bg="#081b25", fg=self.MUTED, font=("Segoe UI", 9)).pack(side="left", padx=12)
        if self.demo_rovl:
            tk.Label(titlebar, text="DEMO / SYNTHETIC DATA", bg="#e55358", fg="#160708", font=("Segoe UI", 9, "bold"), padx=10, pady=2).pack(side="right", padx=14)

        connection_bar = tk.Frame(self, bg=self.BAR, padx=10, pady=7)
        connection_bar.pack(fill="x")
        badges = tk.Frame(connection_bar, bg=self.BAR)
        badges.pack(side="left", fill="x", expand=True)
        if self.surveyor_only:
            self.blueos_badge = self.camera_badge = self.ping_badge = self.rovl_badge = None
            self.surveyor_badge = self._badge(badges, "Surveyor 240-16", SURVEYOR_HOST, 0)
            self.tx_badge = self._badge(badges, "Surveyor TX", "sonar-only", 1)
            badge_columns = 2
        else:
            self.blueos_badge = self._badge(badges, "BlueOS", BLUEOS_HOST, 0)
            self.camera_badge = self._badge(badges, "Camera (5600)", "30 FPS", 1)
            self.surveyor_badge = self._badge(badges, "Surveyor 240-16", SURVEYOR_HOST, 2)
            self.tx_badge = self._badge(badges, "Surveyor TX", "safe", 3)
            self.ping_badge = self._badge(badges, "Ping1D", "10.0 Hz", 4)
            self.rovl_badge = self._badge(badges, "ROVL Mk III", "USB COM", 5)
            badge_columns = 6
        for column in range(badge_columns):
            badges.columnconfigure(column, weight=1)
        if self.wet_authorized:
            self._set_badge(self.tx_badge, False, "AUTHORIZED / READY")
        else:
            self._set_badge(self.tx_badge, False, "LOCKED (DRY MODE)")
        if self.replay_path is not None:
            self._set_badge(self.surveyor_badge, False, "REPLAY")
        elif self.skip_surveyor:
            self._set_badge(self.surveyor_badge, False, "SKIPPED")

        controls = tk.Frame(connection_bar, bg=self.BAR, padx=12)
        controls.pack(side="right")
        if self.surveyor_only:
            self.rovl_combo = None
            self._button(controls, "Connect Surveyor", self.connect_all, "#176fd1", 14).grid(row=0, column=0, padx=4)
            self._button(controls, "Disconnect", self.disconnect_all, "#243847", 10).grid(row=0, column=1, padx=4)
            session_column = 2
        else:
            tk.Label(controls, text="ROVL Port", bg=self.BAR, fg=self.MUTED, font=("Segoe UI", 9)).grid(row=0, column=0, padx=(0, 5))
            self.rovl_combo = ttk.Combobox(controls, textvariable=self.rovl_port, values=("Auto",), width=9, state="readonly", style="Dashboard.TCombobox")
            self.rovl_combo.grid(row=0, column=1, padx=4)
            self._button(controls, "Auto", self._select_auto, "#203746", 6).grid(row=0, column=2, padx=4)
            self._button(controls, "Connect all", self.connect_all, "#176fd1", 11).grid(row=0, column=3, padx=(12, 4))
            self._button(controls, "Disconnect", self.disconnect_all, "#243847", 10).grid(row=0, column=4, padx=4)
            session_column = 5
        self.start_session_button = self._button(controls, "START SESSION", self.start_session, "#078b4c", 13)
        self.start_session_button.grid(row=0, column=session_column, padx=(12, 4))
        self.stop_session_button = self._button(controls, "STOP SESSION", self.stop_session, "#6b2c32", 12)
        self.stop_session_button.grid(row=0, column=session_column + 1, padx=4)
        self.start_surveyor_button = ttk.Button(
            controls,
            text="Start Surveyor" if self.wet_authorized else "Start Surveyor (LOCKED)",
            command=self.start_surveyor,
        )
        self.start_surveyor_button.grid(row=0, column=session_column + 2, padx=(10, 0))
        if not self.wet_authorized:
            self.start_surveyor_button.configure(state="disabled")

        body = tk.Frame(self, bg=self.BG, padx=9, pady=4)
        if self.surveyor_only:
            body.columnconfigure(0, weight=1)
            body.rowconfigure(0, weight=1)
            surveyor = self._card(body)
            surveyor.grid(row=0, column=0, sticky="nsew")
            self._build_surveyor_panel(surveyor)
        else:
            for column in range(2):
                body.columnconfigure(column, weight=1, uniform="column")
            for row in range(2):
                body.rowconfigure(row, weight=1, uniform="row")
            camera, surveyor, ping, rovl = (self._card(body) for _ in range(4))
            camera.grid(row=0, column=0, sticky="nsew", padx=(0, 5), pady=(0, 5))
            surveyor.grid(row=0, column=1, sticky="nsew", padx=(5, 0), pady=(0, 5))
            ping.grid(row=1, column=0, sticky="nsew", padx=(0, 5), pady=(5, 0))
            rovl.grid(row=1, column=1, sticky="nsew", padx=(5, 0), pady=(5, 0))
            self._build_camera_panel(camera)
            self._build_surveyor_panel(surveyor)
            self._build_ping_panel(ping)
            self._build_rovl_panel(rovl)

        status = tk.Frame(self, bg="#071923", padx=12, pady=7, highlightthickness=1, highlightbackground="#163a4c")
        status.pack(side="bottom", fill="x", padx=9, pady=(3, 8))
        self.record_label = tk.Label(status, textvariable=self.record_status, bg="#402126", fg="#ff777a", font=("Segoe UI", 9, "bold"), padx=12, pady=6)
        self.record_label.pack(side="left")
        tk.Label(status, textvariable=self.session_bar, bg="#071923", fg=self.MUTED, font=("Segoe UI", 9)).pack(side="left", padx=25)
        tk.Label(status, textvariable=self.counter_bar, bg="#071923", fg="#d6e2ea", font=("Segoe UI", 9)).pack(side="left", padx=8)
        tk.Label(status, textvariable=self.disk_bar, bg="#071923", fg=self.MUTED, font=("Segoe UI", 9)).pack(side="right", padx=18)
        tk.Label(status, textvariable=self.clock_bar, bg="#071923", fg="#d6e2ea", font=("Consolas", 9)).pack(side="right")
        tk.Label(
            self, textvariable=self.performance_status, bg="#091b25",
            fg="#9fc4d8", font=("Consolas", 8), anchor="w", padx=12,
            pady=4,
        ).pack(side="bottom", fill="x", padx=9)
        body.pack(fill="both", expand=True)

    def _button(self, parent, text, command, color, width):
        return tk.Button(parent, text=text, command=command, bg=color, activebackground=color, fg=self.TEXT, activeforeground=self.TEXT, relief="flat", bd=0, padx=8, pady=7, width=width, cursor="hand2", font=("Segoe UI", 9, "bold"))

    def _card(self, parent):
        return tk.Frame(parent, bg=self.CARD, highlightthickness=1, highlightbackground=self.BORDER)

    def _panel_header(self, parent, icon, title, stats=""):
        frame = tk.Frame(parent, bg=self.CARD, padx=11, pady=6)
        frame.pack(fill="x")
        tk.Label(frame, text=icon, bg=self.CARD, fg=self.CYAN, font=("Segoe UI Symbol", 14, "bold")).pack(side="left")
        tk.Label(frame, text=title, bg=self.CARD, fg=self.TEXT, font=("Segoe UI", 13, "bold")).pack(side="left", padx=8)
        if stats:
            tk.Label(frame, text=stats, bg=self.CARD, fg=self.MUTED, font=("Segoe UI", 9)).pack(side="right")
        return frame

    def _badge(self, parent, title, address, column):
        frame = tk.Frame(parent, bg="#081b25", highlightthickness=1, highlightbackground="#21475b", padx=7, pady=5)
        frame.grid(row=0, column=column, sticky="ew", padx=3)
        top = tk.Frame(frame, bg="#081b25")
        top.pack(fill="x")
        dot = tk.Label(top, text="●", bg="#081b25", fg="#6b7f8a", font=("Segoe UI", 9))
        dot.pack(side="left")
        tk.Label(top, text=title, bg="#081b25", fg=self.TEXT, font=("Segoe UI", 8)).pack(side="left", padx=3)
        state = tk.StringVar(value="DISCONNECTED")
        label = tk.Label(frame, textvariable=state, bg="#081b25", fg="#6b7f8a", font=("Segoe UI", 8, "bold"))
        label.pack(pady=(2, 0))
        address_var = tk.StringVar(value=str(address))
        tk.Label(frame, textvariable=address_var, bg="#081b25", fg=self.MUTED, font=("Segoe UI", 8)).pack()
        return {"state": state, "label": label, "dot": dot, "address": address_var}

    def _set_badge(self, badge, online, detail=None):
        text = str(detail) if detail else ("CONNECTED" if online else "DISCONNECTED")
        upper = text.upper()
        color = self.GREEN if online else (self.AMBER if "LOCKED" in upper or "REPLAY" in upper else (self.MUTED if "SKIPPED" in upper or "NOT AVAILABLE" in upper else self.RED))
        badge["state"].set(text)
        badge["label"].configure(fg=color)
        badge["dot"].configure(fg=color)

    def _build_camera_panel(self, parent):
        self._panel_header(parent, "▣", "RGB CAMERA LIVE", "1920 × 1080     30.1 FPS     H.264 (UDP)")
        holder = tk.Frame(parent, bg="#041019", padx=9, pady=3)
        holder.pack(fill="both", expand=True)
        self.camera_label = tk.Label(holder, text="No RGB frames", bg="#06151e", fg=self.MUTED)
        self.camera_label.pack(fill="both", expand=True)
        self.camera_stats = tk.StringVar(value="Awaiting camera stream")

    def _build_surveyor_panel(self, parent):
        self._panel_header(parent, "◈", "SURVEYOR 240-16", "Range: 25 m     FOV: 80° (±40°)     16 beams")
        content = tk.Frame(parent, bg=self.CARD, padx=8, pady=3)
        content.pack(fill="both", expand=True)
        self.fan_label = tk.Label(content, text="No Surveyor data", bg="#041019", fg=self.MUTED)
        self.fan_label.pack(side="left", fill="both", expand=True)
        side = tk.Frame(content, bg=self.CARD_ALT, width=168, padx=9, pady=7, highlightthickness=1, highlightbackground="#224556")
        side.pack(side="right", fill="y", padx=(8, 0))
        side.pack_propagate(False)
        tk.Label(side, text="Display", bg=self.CARD_ALT, fg="#c8d8e4", font=("Segoe UI", 9, "bold")).pack(anchor="w")
        for title, variable in (("Intensity", self.fan_brightness), ("Contrast", self.fan_contrast)):
            tk.Label(side, text=title, bg=self.CARD_ALT, fg=self.MUTED, font=("Segoe UI", 8)).pack(anchor="w", pady=(6, 0))
            ttk.Scale(side, from_=0.2, to=3.0, variable=variable, command=lambda _v: self._rerender_fan(), style="Dashboard.Horizontal.TScale").pack(fill="x")
        tk.Checkbutton(side, text="Show detections", variable=self.show_atof, command=self._rerender_fan, bg=self.CARD_ALT, fg=self.TEXT, selectcolor="#143649", activebackground=self.CARD_ALT, activeforeground=self.TEXT).pack(anchor="w", pady=(7, 2))
        self.surveyor_stats = tk.StringVar(value="Ping rate    --\nDetections   --\nStatus       WAITING")
        tk.Label(side, textvariable=self.surveyor_stats, justify="left", anchor="nw", bg=self.CARD_ALT, fg=self.MUTED, font=("Consolas", 8), pady=8).pack(fill="x")
        self.attitude_stats = tk.StringVar(value="")

    def _build_ping_panel(self, parent):
        self._panel_header(parent, "⌁", "PING1D")
        content = tk.Frame(parent, bg=self.CARD, padx=9, pady=4)
        content.pack(fill="both", expand=True)
        plot = tk.Frame(content, bg="#041019")
        plot.pack(side="left", fill="both", expand=True)
        tk.Label(plot, text="Distance / Altitude", bg="#041019", fg=self.TEXT, font=("Segoe UI", 10)).pack(anchor="w", padx=48, pady=(3, 0))
        self.ping_profile_label = tk.Label(plot, text="No Ping1D samples", bg="#041019", fg=self.MUTED)
        self.ping_profile_label.pack(fill="both", expand=True)
        side = tk.Frame(content, bg=self.CARD_ALT, width=188, padx=10, pady=7, highlightthickness=1, highlightbackground="#224556")
        side.pack(side="right", fill="y", padx=(8, 0))
        side.pack_propagate(False)
        tk.Label(side, text="Altitude / Range", bg=self.CARD_ALT, fg=self.TEXT, font=("Segoe UI", 10)).pack(anchor="w")
        self.distance_label = tk.Label(side, text="— m", bg=self.CARD_ALT, fg=self.CYAN, font=("Segoe UI", 27, "bold"))
        self.distance_label.pack(anchor="w", pady=(3, 7))
        tk.Label(side, text="Confidence", bg=self.CARD_ALT, fg=self.MUTED, font=("Segoe UI", 9)).pack(anchor="w")
        self.confidence_label = tk.Label(side, text="—", bg=self.CARD_ALT, fg=self.GREEN, font=("Segoe UI", 10, "bold"))
        self.confidence_label.pack(anchor="w", pady=(2, 8))
        self.ping_stats = tk.StringVar(value="Ping rate   --\nGain        --\nMode        Bottom Track\nStatus      WAITING")
        tk.Label(side, textvariable=self.ping_stats, justify="left", bg=self.CARD_ALT, fg=self.MUTED, font=("Consolas", 8)).pack(anchor="w")

    def _build_rovl_panel(self, parent):
        header = self._panel_header(parent, "◎", "ROV LOCATOR Mk III")
        tk.Label(header, text="DEMO / SYNTHETIC DATA" if self.demo_rovl else "READ-ONLY USB", bg="#ff6268" if self.demo_rovl else "#153848", fg="#170708" if self.demo_rovl else self.CYAN, font=("Segoe UI", 9, "bold"), padx=10, pady=2).pack(side="right")
        content = tk.Frame(parent, bg=self.CARD, padx=8, pady=3)
        content.pack(fill="both", expand=True)
        self.rovl_label = tk.Label(content, text="No ROVL position", bg="#041019", fg=self.MUTED)
        self.rovl_label.pack(side="left", fill="both", expand=True)
        side = tk.Frame(content, bg=self.CARD_ALT, width=270, padx=9, pady=5, highlightthickness=1, highlightbackground="#224556")
        side.pack(side="right", fill="y", padx=(8, 0))
        side.pack_propagate(False)
        for title, variable in (("Connection", self.rovl_connection_text), ("Position (relative to topside)", self.rovl_position_text), ("Receiver attitude (topside)", self.rovl_attitude_text), ("Status", self.rovl_health_text)):
            tk.Label(side, text=title, bg=self.CARD_ALT, fg="#b8d2e6", font=("Segoe UI", 8, "bold")).pack(anchor="w", pady=(0, 1))
            tk.Label(side, textvariable=variable, bg=self.CARD_ALT, fg="#d7e5ee", justify="left", anchor="nw", font=("Consolas", 8)).pack(fill="x", pady=(0, 3))

    def _select_auto(self):
        self.rovl_port.set("Auto")
        self._refresh_rovl_ports()

    def _refresh_rovl_ports(self):
        current = self.rovl_port.get()
        values = ["Auto"] + [str(item["device"]) for item in list_serial_devices()]
        self.rovl_combo.configure(values=values)
        self.rovl_port.set(current if current in values else "Auto")

    def _prime_demo_dashboard(self):
        self._set_badge(self.blueos_badge, True, "CONNECTED")
        self._set_badge(self.camera_badge, True, "DEMO STREAM")
        self._set_badge(self.surveyor_badge, True, "DEMO NETWORK")
        self._set_badge(self.ping_badge, True, "DEMO STREAM")
        self._show_camera_image(self._demo_underwater_image())
        self.latest_surveyor = self._demo_surveyor_record()
        self.camera_frame_count, self.surveyor_ping_count, self.ping1d_sample_count = 12420, 5832, 18441
        self._update_surveyor(self.latest_surveyor)
        self._animate_demo_streams()

    def _animate_demo_streams(self):
        if not self.demo_rovl or not self.winfo_exists():
            return
        phase = self._demo_tick * 0.11
        value = 8.4 + 0.65 * math.sin(phase) + 0.22 * math.sin(phase * 2.7)
        self._update_ping({"distance_m": value, "confidence": 96, "host_monotonic_ns": time.monotonic_ns()}, {"gain": "Auto"})
        self._demo_tick += 1
        self.after(400, self._animate_demo_streams)

    def _demo_underwater_image(self):
        width, height = 920, 450
        image = Image.new("RGB", (width, height), "#17697e")
        draw = ImageDraw.Draw(image)
        for y in range(height):
            fraction = y / (height - 1)
            draw.line((0, y, width, y), fill=(int(22 + 20 * fraction), int(112 - 42 * fraction), int(136 - 55 * fraction)))
        draw.polygon([(0, 320), (170, 295), (350, 335), (530, 307), (720, 329), (920, 285), (920, 450), (0, 450)], fill="#9a9b7c")
        for index in range(42):
            x, y = (index * 83) % width, 330 + ((index * 47) % 105)
            radius = 6 + (index * 7) % 19
            draw.ellipse((x - radius, y - radius // 2, x + radius, y + radius // 2), fill="#5d675c", outline="#788176")
        for x, y, radius in ((65, 285, 80), (770, 280, 100), (860, 325, 70)):
            draw.ellipse((x - radius, y - radius // 2, x + radius, y + radius), fill="#465b52", outline="#627269")
        return image

    def _demo_surveyor_record(self):
        matrix = []
        for row in range(81):
            angle = -40.0 + row
            values = []
            for column in range(120):
                distance = 25.0 * column / 120.0
                echo = sum(strength * math.exp(-((angle - target_angle) / 1.5) ** 2 - ((distance - target_range) / 0.45) ** 2) for target_angle, target_range, strength in ((-18, 8.2, 8), (-4, 14.8, 11), (14, 9.2, 9), (27, 7.0, 8)))
                bottom = 4.5 * math.exp(-((distance - (5.1 + 0.002 * angle * angle)) / 0.7) ** 2)
                values.append(max(0.01, 0.8 + 0.12 * math.sin(angle * 0.23) + 0.25 * math.sin(distance * 0.8) + echo + bottom))
            matrix.append(values)
        return {"matrix": matrix, "range_start_m": 0.0, "range_end_m": 25.0, "ping_number": 5832, "ping_rate_hz": 4.8, "detection_count": 12, "points": [{"angle_rad": math.radians(a), "distance_m": d} for a, d in ((-28, 8.3), (-18, 7.2), (-9, 14.8), (3, 9.0), (14, 8.3), (26, 7.0))], "channel_data_status": "AVAILABLE", "host_monotonic_ns": time.monotonic_ns()}

    def _show_camera_image(self, image):
        self.latest_camera_pil = image.copy()
        shown = image.copy()
        shown.thumbnail((730, 365), Image.Resampling.LANCZOS)
        self.photos["camera"] = ImageTk.PhotoImage(shown)
        self.camera_label.configure(image=self.photos["camera"], text="")

    def _update_camera(self, frame):
        if cv2 is not None:
            try:
                self._show_camera_image(Image.fromarray(frame[:, :, ::-1]))
                self.camera_frame_count += 1
            except Exception:
                self.metrics.increment("camera_gui_errors")
                LOGGER.exception("Camera GUI conversion failure")

    def _update_surveyor(self, record):
        self._rerender_fan()
        self.surveyor_stats.set("Ping rate    %4.1f Hz\nDetections   %4d\nStatus       RECEIVING" % (record.get("ping_rate_hz") or 0.0, record.get("detection_count", 0)))

    def _rerender_fan(self):
        if self.latest_surveyor is None:
            return
        record = dict(self.latest_surveyor)
        image = fan_image(record, 620, 365, self.fan_brightness.get(), self.fan_contrast.get(), self.show_atof.get())
        self.photos["fan"] = ImageTk.PhotoImage(image)
        self.fan_label.configure(image=self.photos["fan"], text="")

    def _update_ping(self, distance, profile):
        value = float(distance.get("distance_m", 0.0))
        confidence = int(distance.get("confidence", 0))
        self.ping_history.append((int(distance.get("host_monotonic_ns") or time.monotonic_ns()), value))
        self.ping_history = self.ping_history[-150:]
        self.distance_label.configure(text="%.1f m" % value)
        self.confidence_label.configure(text="●  %d%% · VALID" % confidence)
        self.ping_stats.set("Ping rate   10.0 Hz\nGain        %s\nMode        Bottom Track\nStatus      RECEIVING" % ((profile or {}).get("gain", "Auto")))
        self._draw_ping_history()

    def _draw_ping_history(self):
        width, height = 570, 285
        image = Image.new("RGB", (width, height), "#041019")
        draw = ImageDraw.Draw(image)
        left, top, right, bottom = 48, 15, width - 15, height - 34
        for value in range(0, 21, 5):
            y = bottom - value / 20.0 * (bottom - top)
            draw.line((left, y, right, y), fill="#264452")
            draw.text((14, y - 6), str(value), fill="#9db2c1")
        for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
            x = left + fraction * (right - left)
            draw.line((x, top, x, bottom), fill="#1c3948")
            draw.text((x - 10, bottom + 8), "%d" % int(-60 + fraction * 60), fill="#9db2c1")
        samples = self.ping_history[-60:]
        points = [(left + index / max(1, len(samples) - 1) * (right - left), bottom - max(0.0, min(20.0, item[1])) / 20.0 * (bottom - top)) for index, item in enumerate(samples)]
        if len(points) > 1:
            draw.line(points, fill=self.CYAN, width=2)
        draw.text((width // 2 - 20, height - 16), "Time (s)", fill="#9db2c1")
        draw.text((3, 3), "Distance (m)", fill="#cbdbe5")
        self.photos["ping"] = ImageTk.PhotoImage(image)
        self.ping_profile_label.configure(image=self.photos["ping"], text="")

    def _update_rovl(self, sample):
        parsed, position = sample.get("parsed") or {}, sample.get("position") or {}
        self.rovl_lock = bool(position.get("lock"))
        port = getattr(self.rovl_worker, "port", None) or ("DEMO" if sample.get("synthetic") else "--")
        status = "RECEIVING" if parsed.get("parse_ok") else "DATA WARNING"
        self.rovl_connection_text.set("COM port       %s\nStatus         %s\nAcoustic lock  %s" % (port, status, "● YES" if self.rovl_lock else "○ NO"))
        if self.rovl_lock:
            self.rovl_position_text.set("Slant range       %4.1f m\nHorizontal range  %4.1f m\nBearing          %5.1f°\nElevation        %5.1f°" % (position.get("slant_range_m") or 0.0, position.get("horizontal_range_m") or 0.0, position.get("bearing_deg") or 0.0, position.get("elevation_deg") or 0.0))
        else:
            self.rovl_position_text.set("Slant range        --\nHorizontal range   --\nBearing            --\nElevation          --")
        self.rovl_attitude_text.set("Heading     %s\nRoll        %s\nPitch       %s" % ("%.1f°" % parsed["ch"] if parsed.get("ch") is not None else "--", "%.1f°" % parsed["er"] if parsed.get("er") is not None else "--", "%.1f°" % parsed["ep"] if parsed.get("ep") is not None else "--"))
        if len(self.rovl_sample_times) > 1:
            span = (self.rovl_sample_times[-1] - self.rovl_sample_times[0]) / 1e9
            rate = (len(self.rovl_sample_times) - 1) / span if span > 0 else 0.0
        else:
            rate = 0.0
        self.rovl_health_text.set(
            "IMU          %s\nGain         %s\nIDs          %s / %s\nMethod       %s\nUpdate age   0.0 s\nUpdate rate  %.1f Hz" % (
                parsed.get("im") or "--",
                "%.0f dB" % parsed["db"] if parsed.get("db") is not None else "--",
                parsed.get("idx") if parsed.get("idx") is not None else "--",
                parsed.get("idq") if parsed.get("idq") is not None else "--",
                position.get("method") or "--",
                rate,
            )
        )
        self._draw_rovl_position(position)

    def _draw_rovl_position(self, position):
        width, height = 500, 315
        image = Image.new("RGB", (width, height), "#041019")
        draw = ImageDraw.Draw(image)
        cx, cy, radial = width * 0.50, height * 0.52, 132.0
        ranges = [float(item.get("horizontal_range_m")) for item in self.rovl_trail if item.get("horizontal_range_m") is not None]
        scale = max(30.0, math.ceil(max(ranges or [0.0]) / 10.0) * 10.0)
        draw.text((12, 8), "⌄  Top-Down View (%s)" % ("North / East" if position.get("north_m") is not None else "Receiver-relative"), fill="#c8d9e5")
        for fraction in (1 / 3, 2 / 3, 1.0):
            radius = radial * fraction
            draw.ellipse((cx - radius, cy - radius, cx + radius, cy + radius), outline="#295066")
            draw.text((cx + 4, cy - radius - 10), "%.0f m" % (scale * fraction), fill="#a9bdca")
        draw.line((cx - radial, cy, cx + radial, cy), fill="#294d60")
        draw.line((cx, cy - radial, cx, cy + radial), fill="#294d60")
        if position.get("north_m") is not None:
            draw.text((cx - 5, cy - radial - 28), "N", fill=self.TEXT)
        def project(item):
            if item.get("north_m") is not None:
                east, north = float(item["east_m"]), float(item["north_m"])
            else:
                east, north = -float(item.get("relative_y_m") or 0.0), float(item.get("relative_x_m") or 0.0)
            return cx + east / scale * radial, cy - north / scale * radial
        points = [project(item) for item in self.rovl_trail if item.get("lock")]
        if len(points) > 1:
            draw.line(points, fill="#1f91d5", width=2)
        for point in points[:-1]:
            draw.ellipse((point[0] - 2, point[1] - 2, point[0] + 2, point[1] + 2), fill="#2ea8ed")
        draw.ellipse((cx - 7, cy - 7, cx + 7, cy + 7), fill="#ff5f60")
        draw.text((cx - 28, cy + 12), "Topside", fill=self.TEXT)
        if position.get("lock"):
            x, y = project(position)
            draw.polygon(((x, y - 9), (x - 8, y + 8), (x + 8, y + 8)), fill=self.CYAN)
            draw.line((cx, cy, x, y), fill="#2e9dd9")
            draw.text((x + 12, y - 18), "BlueROV2\n(%.1f m)" % float(position.get("slant_range_m") or 0.0), fill=self.CYAN)
        draw.text((width - 85, height - 18), "Scale: %.0f m" % scale, fill="#a9bdca")
        self.photos["rovl"] = ImageTk.PhotoImage(image)
        self.rovl_label.configure(image=self.photos["rovl"], text="")

    def _refresh_status_age(self):
        try:
            target = SESSION_ROOT.parent if SESSION_ROOT.parent.exists() else APP_ROOT
            free_gb = shutil.disk_usage(str(target)).free / (1024 ** 3)
            self.disk_bar.set("Disk:  %.0f GB free" % free_gb)
        except Exception:
            self.disk_bar.set("Disk:  -- GB free")
        self.clock_bar.set(datetime.now().strftime("%Y-%m-%d  %H:%M:%S"))
        if self.session is None:
            self.session_bar.set("Session:   –")
        else:
            elapsed = (time.monotonic_ns() - self.session.session_start_monotonic_ns) / 1_000_000_000.0
            self.session_bar.set("Session:   %s   %.1fs" % (self.session.session_id, elapsed))
        if self.surveyor_only:
            self.counter_bar.set("Surveyor pings:  %s" % f"{self.surveyor_ping_count:,}")
        else:
            self.counter_bar.set(
                "Frames (cam):  %s     Surveyor pings:  %s     Ping1D samples:  %s     ROVL fixes:  %s" %
                (f"{self.camera_frame_count:,}", f"{self.surveyor_ping_count:,}", f"{self.ping1d_sample_count:,}", f"{self.rovl_fix_count:,}")
            )
        if self.latest_rovl_sample is not None:
            parsed = self.latest_rovl_sample.get("parsed") or {}
            age = max(0.0, (time.monotonic_ns() - int(self.latest_rovl_sample.get("host_monotonic_ns") or time.monotonic_ns())) / 1_000_000_000.0)
            lines = self.rovl_health_text.get().splitlines()
            if len(lines) >= 2:
                lines[-2] = "Update age   %.1f s" % age
                self.rovl_health_text.set("\n".join(lines))
        self.after(250, self._refresh_status_age)

    def close(self):
        self.stop_session()
        self.disconnect_all()
        stuck = []
        for worker, timeout in (
            (self.surveyor_worker, 10.0), (self.ping_worker, 5.0),
            (self.camera_worker, 5.0), (self.blueos_worker, 2.0),
            (self.rovl_worker, 5.0),
        ):
            if worker is not None:
                worker.join(timeout=timeout)
                if worker.is_alive():
                    stuck.append(worker.name)
        if self._session_close_thread is not None:
            self._session_close_thread.join(timeout=15.0)
            if self._session_close_thread.is_alive():
                stuck.append(self._session_close_thread.name)
        if stuck:
            self.metrics.increment("shutdown_workers_stuck", len(stuck))
        self.destroy()


def main():
    configure_diagnostics(APP_ROOT)
    parser = argparse.ArgumentParser(description="BlueROV2 Camera/Ping1D/ROVL Recorder")
    parser.add_argument("--offline", action="store_true", help="build the GUI without connecting to hardware")
    parser.add_argument("--rovl-port", default="Auto", help="ROVL COM port or Auto (default)")
    parser.add_argument("--demo", "--demo-rovl", action="store_true", dest="demo", help="hardware-free synthetic camera/Ping1D/ROVL dashboard")
    args = parser.parse_args()
    from .live_app import CameraPingROVLViewer
    app = CameraPingROVLViewer(offline=args.offline, demo=args.demo, rovl_port=args.rovl_port)
    if not args.offline and not args.demo:
        app.connect_all()
    app.mainloop()


if __name__ == "__main__":
    main()
