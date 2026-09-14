"""BlueROV2 Multimodal Recorder.

The application combines three independent, timestamped streams:

* Cerulean Surveyor 240-16 on TCP ``192.168.2.86:62312``;
* Blue Robotics Ping1D through BlueOS PingProxy UDP ``192.168.2.2:9090``;
* the BlueROV2 RGB camera stream, normally UDP port 5600 (or a separately
  configured 5602 output).

Surveyor transmission is safety-locked by default. In the default dry mode
the app may perform read-only connection/replay work, but it never calls the
Surveyor ping-parameter command and never enables acoustic transmission. A
future wet test requires the explicit command-line flag ``--wet-authorized``.
That flag is intentionally not used by the normal launcher or offline tests.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import queue
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
    from .processing import (
        MSG_ATOF, MSG_ATTITUDE, MSG_END_PING, MSG_JSON, MSG_RAW_PROFILE,
        PACKET_CHECKSUM, PACKET_HEADER, SurveyorChannelAccumulator,
        beamform_surveyor_channels, decode_atof_payload,
        decode_attitude_payload, decode_end_ping_payload, iter_svlog_packets,
        make_packet,
    )
except ImportError:  # Support direct execution from the package directory too.
    from processing import (
        MSG_ATOF, MSG_ATTITUDE, MSG_END_PING, MSG_JSON, MSG_RAW_PROFILE,
        PACKET_CHECKSUM, PACKET_HEADER, SurveyorChannelAccumulator,
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

APP_ROOT = Path(__file__).resolve().parents[2]
SESSION_ROOT = APP_ROOT / "records" / "real_sessions"

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


def iter_live_packets(device, stop_event: threading.Event):
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

    def __init__(self, host, port, events, dry_mode=True, wet_authorized=False, replay_path=None, raw_callback=None):
        super(SurveyorWorker, self).__init__(daemon=True)
        self.host = host
        self.port = int(port)
        self.events = events
        self.dry_mode = bool(dry_mode)
        self.wet_authorized = bool(wet_authorized)
        self.replay_path = Path(replay_path) if replay_path else None
        self.raw_callback = raw_callback
        self.stop_event = threading.Event()
        self.commands = queue.Queue()
        self.device = None
        self.started_by_app = False

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
        self.started_by_app = True
        self.events.put(("surveyor_started", None))

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
                self.events.put(("surveyor_stopped", None))

    def _process_packets(self, packet_iterator):
        accumulator = SurveyorChannelAccumulator()
        atof_by_ping = {}
        attitude = None
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
                self.events.put(("surveyor_attitude", attitude))
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
        try:
            if self.replay_path is not None:
                self.events.put(("surveyor_connected", {"replay": str(self.replay_path)}))
                replay_surveyor(self.replay_path, self.events, self.stop_event, self.raw_callback)
                return
            if Surveyor240 is None:
                raise RuntimeError("bluerobotics-ping non installato")
            self.device = Surveyor240()
            self.device.connect_tcp(self.host, self.port)
            if self.device.initialize() is False:
                raise RuntimeError("Surveyor240 initialize() fallita")
            self.events.put(("surveyor_connected", {"replay": None, "tx_locked": self.tx_locked}))
            self._process_packets(iter_live_packets(self.device, self.stop_event))
        except Exception as exc:
            if not self.stop_event.is_set():
                self.events.put(("surveyor_error", str(exc)))
        finally:
            if self.started_by_app and self.device is not None and not self.tx_locked:
                try:
                    self.device.control_set_ping_parameters(ping_enable=False)
                except Exception:
                    pass
                self.started_by_app = False
            close_brping_device(self.device)
            self.events.put(("surveyor_closed", None))


class Ping1DWorker(threading.Thread):
    """Official Ping1D worker through BlueOS PingProxy UDP."""

    def __init__(self, host, port, events):
        super(Ping1DWorker, self).__init__(daemon=True)
        self.host = host
        self.port = int(port)
        self.events = events
        self.stop_event = threading.Event()
        self.device = None

    def run(self):
        try:
            if Ping1D is None:
                raise RuntimeError("bluerobotics-ping non installato")
            self.device = Ping1D()
            self.device.connect_udp(self.host, self.port)
            if self.device.initialize() is False:
                raise RuntimeError("Ping1D initialize() fallita")
            self.events.put(("ping_connected", None))
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
                    self.events.put(("ping_warning", "nessuna risposta Ping1D"))
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
                self.events.put(("ping_sample", (record, profile_record)))
                time.sleep(0.02)
        except Exception as exc:
            if not self.stop_event.is_set():
                self.events.put(("ping_error", str(exc)))
        finally:
            close_brping_device(self.device)
            self.events.put(("ping_closed", None))

    def stop(self):
        self.stop_event.set()
        close_brping_device(self.device)


class CameraWorker(threading.Thread):
    """Single-ingest camera reader with PyAV remux/decode fan-out.

    PyAV demuxes each encoded packet once. The packet is offered to the
    session remuxer and then decoded frames are sent to the GUI. OpenCV is
    used only when PyAV is unavailable or cannot open the selected source.
    """

    def __init__(self, port=CAMERA_DEFAULT_PORT, source=None, events=None):
        super(CameraWorker, self).__init__(daemon=True)
        self.port = int(port)
        self.source = source or "udp://0.0.0.0:%d" % self.port
        self.events = events if events is not None else queue.Queue()
        self.stop_event = threading.Event()
        self.capture = None
        self.container = None
        self.packet_callback = None
        self._callback_lock = threading.Lock()
        self.backend = "PENDING"

    def set_packet_callback(self, callback):
        """Atomically replace the session callback used by the ingest thread."""
        with self._callback_lock:
            self.packet_callback = callback

    def _get_packet_callback(self):
        with self._callback_lock:
            return self.packet_callback

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
        self.events.put(("camera_backend", metadata))

    def _run_pyav(self):
        if av is None:
            raise RuntimeError("PyAV non installato")
        source_text = str(self.source)
        open_kwargs = {}
        if source_text.lower().startswith(("udp://", "rtp://")):
            open_kwargs["options"] = {"fflags": "nobuffer", "flags": "low_delay"}
        self.container = av.open(self.source, mode="r", **open_kwargs)
        streams = [stream for stream in self.container.streams if getattr(stream, "type", None) == "video"]
        if not streams:
            raise RuntimeError("la sorgente non contiene uno stream video")
        stream = streams[0]
        metadata = self._stream_metadata(stream, self.source, "PyAV", "READY")
        metadata["port"] = self.port
        self._emit_backend(metadata)
        self.events.put(("camera_connected", metadata))
        packet_index = 0
        frame_index = 0
        for packet in self.container.demux(stream):
            if self.stop_event.is_set():
                break
            if packet is None:
                continue
            host_mono = time.monotonic_ns()
            host_utc = time.time_ns()
            try:
                frames = list(packet.decode())
            except Exception as exc:
                frames = []
                self.events.put(("camera_decode_warning", str(exc)))
            callback = self._get_packet_callback()
            if callback is not None:
                callback(packet, stream, packet_index, host_mono, host_utc)
            if not frames:
                packet_index += 1
                continue
            for frame in frames:
                if self.stop_event.is_set():
                    break
                image = frame.to_ndarray(format="bgr24")
                frame_metadata = self._packet_frame_metadata(packet, stream, packet_index, frame, host_mono, host_utc, frame_index)
                self.events.put(("camera_frame", (image, frame_metadata)))
                frame_index += 1
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
        self.events.put(("camera_connected", metadata))
        frame_index = 0
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
            self.events.put(("camera_frame", (frame, {
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
            })))
            frame_index += 1

    def run(self):
        try:
            if av is not None:
                try:
                    self._run_pyav()
                    return
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
                self.events.put(("camera_error", str(exc)))
        finally:
            self.set_packet_callback(None)
            if self.capture is not None:
                self.capture.release()
            if self.container is not None:
                try:
                    self.container.close()
                except Exception:
                    pass
            self.events.put(("camera_closed", None))

    def stop(self):
        self.stop_event.set()
        self.set_packet_callback(None)
        if self.capture is not None:
            try:
                self.capture.release()
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
    """Write a local synchronized session without changing source files."""

    def __init__(self, root=SESSION_ROOT, camera_port=CAMERA_DEFAULT_PORT, camera_source=None, surveyor_mode="live", surveyor_replay_source=None):
        self.root = Path(root)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_id = "%s_%s" % (timestamp, uuid.uuid4().hex[:8])
        self.directory = self.root / self.session_id
        self.directory.mkdir(parents=True, exist_ok=False)
        self.session_start_utc_ns = time.time_ns()
        self.session_start_monotonic_ns = time.monotonic_ns()
        self.camera_port = int(camera_port)
        self.camera_source = camera_source or "udp://0.0.0.0:%d" % self.camera_port
        self.surveyor_mode = str(surveyor_mode)
        self.surveyor_replay_source = str(surveyor_replay_source) if surveyor_replay_source else None
        self.lock = threading.Lock()
        self.closed = False
        self.files = {}
        self.camera_writer = None
        self.camera_container = None
        self.camera_stream = None
        self.camera_size = None
        self.camera_frame_index = 0
        self.camera_backend = "PENDING"
        self.camera_recording_mode = "PENDING"
        self.remux_failed = False
        self.camera_metadata = {
            "backend": "PENDING",
            "recording_mode": "PENDING",
            "source": self.camera_source,
            "codec": None,
            "width": None,
            "height": None,
            "nominal_fps": None,
            "time_base": {"num": None, "den": None},
            "video_pts": "UNAVAILABLE",
            "input_status": "PENDING",
            "remux_status": "NOT STARTED",
        }
        self.session_metadata = {
            "session_id": self.session_id,
            "session_start_utc_ns": self.session_start_utc_ns,
            "session_start_monotonic_ns": self.session_start_monotonic_ns,
            "session_start_utc": utc_iso(self.session_start_utc_ns),
            "camera": self.camera_metadata,
            "surveyor": {"host": SURVEYOR_HOST, "port": SURVEYOR_PORT, "mode": self.surveyor_mode, "replay_source": self.surveyor_replay_source, "tx": "LOCKED"},
            "ping1d": {"host": PING1D_HOST, "port": PING1D_PORT},
        }
        self._open_files()
        self.session_metadata["camera"].update({"port": self.camera_port})
        self._write_session(self.session_metadata)
        self._write_svlog_metadata()

    def _open_files(self):
        self.files["surveyor_pings"] = open(str(self.directory / "surveyor_pings.jsonl"), "w", encoding="utf-8")
        self.files["ping1d"] = open(str(self.directory / "ping1d.jsonl"), "w", encoding="utf-8")
        self.files["events"] = open(str(self.directory / "events.jsonl"), "w", encoding="utf-8")
        self.files["camera_timestamps"] = open(str(self.directory / "camera_timestamps.csv"), "w", newline="", encoding="utf-8")
        self.camera_csv = csv.writer(self.files["camera_timestamps"])
        self.camera_csv.writerow([
            "session_id", "session_start_utc_ns", "session_start_monotonic_ns",
            "frame_index", "packet_index", "pts", "dts", "time_base_num",
            "time_base_den", "pts_seconds", "dts_seconds", "host_monotonic_ns",
            "host_utc_ns", "session_time_s", "key_frame", "packet_size",
        ])
        if self.surveyor_mode != "skipped":
            self.files["svlog"] = open(str(self.directory / "surveyor_raw.svlog"), "wb")

    def _write_session(self, data):
        with open(str(self.directory / "session.json"), "w", encoding="utf-8") as stream:
            json.dump(data, stream, ensure_ascii=False, indent=2)

    def update_camera_metadata(self, metadata):
        with self.lock:
            if self.closed:
                return
            self.camera_metadata.update(dict(metadata or {}))
            self.camera_backend = self.camera_metadata.get("backend", self.camera_backend)
            self.camera_recording_mode = self.camera_metadata.get("recording_mode", self.camera_recording_mode)
            self.session_metadata["camera"] = dict(self.camera_metadata)
            self.session_metadata["camera"]["port"] = self.camera_port
            self._write_session(self.session_metadata)

    def _write_svlog_metadata(self):
        metadata = {
            "session_id": self.session_id,
            "session_uptime": 0.0,
            "session_devices": [{"url": "tcp://%s:%d" % (SURVEYOR_HOST, SURVEYOR_PORT), "product_id": "mbes24016"}],
            "is_recording": True,
            "sonarlink_version": "",
            "timestamp": utc_iso(self.session_start_utc_ns),
            "recorder": "BlueROV2 Multimodal Recorder",
            "mode": self.surveyor_mode,
            "replay_source": self.surveyor_replay_source,
            "session_start_utc_ns": self.session_start_utc_ns,
            "session_start_monotonic_ns": self.session_start_monotonic_ns,
        }
        if "svlog" in self.files:
            self.files["svlog"].write(make_packet(MSG_JSON, json.dumps(metadata, indent=2).encode("utf-8")))
            self.files["svlog"].flush()

    def write_surveyor_packet(self, raw_packet, host_monotonic_ns=None, host_utc_ns=None):
        with self.lock:
            if not self.closed and "svlog" in self.files:
                self.files["svlog"].write(bytes(raw_packet))
                self.files["svlog"].flush()

    def write_surveyor_ping(self, record):
        with self.lock:
            if not self.closed:
                item = serializable_surveyor_record(record)
                item.update({"session_id": self.session_id, "session_start_utc_ns": self.session_start_utc_ns, "session_start_monotonic_ns": self.session_start_monotonic_ns})
                self.files["surveyor_pings"].write(json.dumps(item, ensure_ascii=False) + "\n")
                self.files["surveyor_pings"].flush()

    def write_ping1d(self, record):
        with self.lock:
            if not self.closed:
                item = dict(record)
                item.update({"session_id": self.session_id, "session_start_utc_ns": self.session_start_utc_ns, "session_start_monotonic_ns": self.session_start_monotonic_ns})
                self.files["ping1d"].write(json.dumps(item, ensure_ascii=False) + "\n")
                self.files["ping1d"].flush()

    def write_event(self, kind, data=None):
        with self.lock:
            if not self.closed:
                self.files["events"].write(json.dumps({"timestamp": utc_iso(), "session_id": self.session_id, "session_start_utc_ns": self.session_start_utc_ns, "session_start_monotonic_ns": self.session_start_monotonic_ns, "kind": kind, "data": data}, ensure_ascii=False) + "\n")
                self.files["events"].flush()

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
        with self.lock:
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
        with self.lock:
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
        self.camera_csv.writerow(values)
        self.files["camera_timestamps"].flush()
        self.camera_frame_index = max(self.camera_frame_index, int(item.get("frame_index") or 0) + 1)

    def close(self):
        with self.lock:
            if self.closed:
                return
            self.closed = True
            if self.camera_container is not None:
                try:
                    self.camera_container.close()
                except Exception:
                    pass
            if self.camera_writer is not None:
                self.camera_writer.release()
            for stream in self.files.values():
                try:
                    stream.close()
                except Exception:
                    pass
        with open(str(self.directory / "session.json"), "r", encoding="utf-8") as stream:
            metadata = json.load(stream)
        metadata["session_end_utc_ns"] = time.time_ns()
        metadata["session_end_monotonic_ns"] = time.monotonic_ns()
        metadata["closed"] = True
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


class SonarViewer(tk.Tk):
    """Three-panel live/replay GUI."""

    PROFILE_W = 560
    PROFILE_H = 230

    def __init__(self, offline=False, replay_path=None, wet_authorized=False, skip_surveyor=False):
        super(SonarViewer, self).__init__()
        self.title("BlueROV2 Multimodal Recorder")
        self.geometry("1600x950")
        self.minsize(1200, 760)
        self.events = queue.Queue()
        self.offline = bool(offline)
        self.replay_path = Path(replay_path) if replay_path else None
        self.wet_authorized = bool(wet_authorized)
        self.skip_surveyor = bool(skip_surveyor) and self.replay_path is None
        self.surveyor_worker = None
        self.ping_worker = None
        self.camera_worker = None
        self.blueos_worker = None
        self.session = None
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
        self.after(50, self._poll_events)
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

    def _attach_camera_session(self):
        """Connect the active session to an existing or newly-created camera."""
        if self.camera_worker is not None and self.session is not None:
            self.camera_worker.set_packet_callback(self.session.write_camera_packet)

    def _detach_camera_session(self):
        """Detach before closing a session; CameraWorker makes this thread-safe."""
        if self.camera_worker is not None:
            self.camera_worker.set_packet_callback(None)

    def connect_all(self):
        if self.offline:
            if self.replay_path is not None and self.surveyor_worker is None:
                self.surveyor_worker = SurveyorWorker(SURVEYOR_HOST, SURVEYOR_PORT, self.events, dry_mode=True, wet_authorized=False, replay_path=self.replay_path)
                if self.session is not None:
                    self.surveyor_worker.raw_callback = self.session.write_surveyor_packet
                self.surveyor_worker.start()
                self.status_text.set("Offline replay avviato — nessuna connessione hardware")
            else:
                self.status_text.set("Offline: nessuna connessione hardware avviata")
            return
        if self.blueos_worker is None:
            self.blueos_worker = BlueOSWorker(BLUEOS_HOST, self.events)
            self.blueos_worker.start()
        if self.surveyor_worker is None and self.replay_path is not None:
            self.surveyor_worker = SurveyorWorker(SURVEYOR_HOST, SURVEYOR_PORT, self.events, dry_mode=not self.wet_authorized, wet_authorized=self.wet_authorized, replay_path=self.replay_path)
            if self.session is not None:
                self.surveyor_worker.raw_callback = self.session.write_surveyor_packet
            self.surveyor_worker.start()
        elif self.surveyor_worker is None and not self.skip_surveyor:
            self.surveyor_worker = SurveyorWorker(SURVEYOR_HOST, SURVEYOR_PORT, self.events, dry_mode=not self.wet_authorized, wet_authorized=self.wet_authorized)
            if self.session is not None:
                self.surveyor_worker.raw_callback = self.session.write_surveyor_packet
            self.surveyor_worker.start()
        if self.ping_worker is None:
            self.ping_worker = Ping1DWorker(PING1D_HOST, PING1D_PORT, self.events)
            self.ping_worker.start()
        if self.camera_worker is None:
            source = self.camera_sdp.get().strip() or None
            self.camera_worker = CameraWorker(self.camera_port.get(), source, self.events)
            self._attach_camera_session()
            self.camera_worker.start()
        mode = "REPLAY" if self.replay_path is not None else ("SKIPPED" if self.skip_surveyor else ("TX: LOCKED / DRY MODE" if not self.wet_authorized else "LIVE"))
        self.status_text.set("Connessioni avviate — SURVEYOR %s" % mode)

    def start_surveyor(self):
        if not self.wet_authorized or self.surveyor_worker is None:
            messagebox.showwarning("Surveyor TX bloccato", "Il Surveyor è in DRY MODE: nessuna trasmissione acustica è autorizzata.")
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
        self.status_text.set("Disconnessione richiesta — TX rimane LOCKED")

    def start_session(self):
        if self.session is not None:
            return
        try:
            surveyor_mode = "replay" if self.replay_path is not None else ("skipped" if self.skip_surveyor else "live")
            self.camera_frame_count = 0
            self.surveyor_ping_count = 0
            self.ping1d_sample_count = 0
            self.session = SessionRecorder(SESSION_ROOT, self.camera_port.get(), self.camera_sdp.get().strip() or None, surveyor_mode, self.replay_path)
            self._attach_camera_session()
            if self.surveyor_worker is not None:
                self.surveyor_worker.raw_callback = self.session.write_surveyor_packet
            self.session.update_camera_metadata(self.camera_metadata)
            self.session.write_event("session_started", {"tx": "LOCKED" if not self.wet_authorized else "manual-authorized"})
            self.status_text.set("Registrazione sessione: %s" % self.session.directory)
        except Exception as exc:
            messagebox.showerror("Sessione", "Impossibile avviare la registrazione: %s" % exc)

    def stop_session(self):
        if self.session is None:
            return
        session = self.session
        self._detach_camera_session()
        if self.surveyor_worker is not None:
            self.surveyor_worker.raw_callback = None
        session.write_event("session_stopped")
        session.close()
        self.session = None
        self.status_text.set("Sessione salvata: %s" % session.directory)

    def _poll_events(self):
        try:
            while True:
                kind, data = self.events.get_nowait()
                if self.session is not None and kind not in ("camera_frame", "surveyor_raw_packet"):
                    try:
                        self.session.write_event(kind, data if kind not in ("surveyor_ping", "ping_sample") else None)
                    except Exception:
                        pass
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
                elif kind == "camera_error":
                    self._set_badge(self.camera_badge, False, data)
                elif kind == "camera_closed":
                    if self.camera_worker is not None:
                        self.camera_worker.set_packet_callback(None)
                    self.camera_worker = None
                    self._set_badge(self.camera_badge, False)
                elif kind == "surveyor_connected":
                    self._set_badge(self.surveyor_badge, True, "REPLAY" if data.get("replay") else "PASSIVE")
                elif kind == "surveyor_ping":
                    self.latest_surveyor = data
                    self.surveyor_ping_count += 1
                    if self.session is not None:
                        self.session.write_surveyor_ping(data)
                    self._update_surveyor(data)
                elif kind == "surveyor_attitude":
                    if data:
                        self.attitude_stats.set("Attitude up vector: %.3f, %.3f, %.3f" % (data.get("up_vec_x", 0), data.get("up_vec_y", 0), data.get("up_vec_z", 0)))
                elif kind == "surveyor_started":
                    self._set_badge(self.tx_badge, True, "ACTIVE")
                elif kind == "surveyor_stopped":
                    self._set_badge(self.tx_badge, False, "LOCKED")
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
        except queue.Empty:
            pass
        self.after(50, self._poll_events)

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
            pass

    def _update_surveyor(self, record):
        self._rerender_fan()
        self.surveyor_stats.set("CHANNEL DATA: %s | ping: %s | rate: %.2f Hz | range: %.2f..%.2f m | detections: %d" % (record.get("channel_data_status", "--"), record.get("ping_number", "--"), record.get("ping_rate_hz", 0.0), record.get("range_start_m", 0.0), record.get("range_end_m", 0.0), record.get("detection_count", 0)))

    def _rerender_fan(self):
        if self.latest_surveyor is None:
            return
        record = dict(self.latest_surveyor)
        if record.get("channel_signals"):
            try:
                record["matrix"] = beamform_surveyor_channels(record["channel_signals"], record.get("range_start_m", 0.0), record.get("range_end_m", 10.0), record.get("sos_mps", 1500.0), threshold_percent=self.fan_threshold.get())
            except ValueError:
                pass
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
        for worker, timeout in ((self.surveyor_worker, 1.5), (self.ping_worker, 1.0), (self.camera_worker, 1.0), (self.blueos_worker, 0.5)):
            if worker is not None:
                worker.join(timeout=timeout)
        self.destroy()


def main():
    parser = argparse.ArgumentParser(description="BlueROV2 Multimodal Recorder — Surveyor/Ping1D/RGB")
    parser.add_argument("--offline", action="store_true", help="build the GUI without connecting to hardware")
    parser.add_argument("--replay-surveyor", metavar="FILE.svlog", help="replay a local Surveyor log without transmitting")
    parser.add_argument("--skip-surveyor", action="store_true", help="do not instantiate or connect the hardware Surveyor")
    parser.add_argument("--wet-authorized", action="store_true", help="manual future authorization for Surveyor ping transmission")
    args = parser.parse_args()
    app = SonarViewer(offline=args.offline, replay_path=args.replay_surveyor, wet_authorized=args.wet_authorized, skip_surveyor=args.skip_surveyor)
    app.mainloop()


if __name__ == "__main__":
    main()
