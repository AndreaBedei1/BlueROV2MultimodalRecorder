"""Passive Cerulean ROV Locator serial acquisition.

The module deliberately exposes no serial-write API.  It only discovers and
reads the USB COM stream emitted by a ROVL receiver/transceiver.
"""

from __future__ import annotations

import math
import queue
import threading
import time
from datetime import datetime, timezone
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple

try:  # Keep the rest of the recorder importable without hardware extras.
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - exercised by import-isolation tests.
    serial = None
    list_ports = None


ROVL_BAUD = 115_200
USRTH_FIELD_NAMES: Tuple[str, ...] = (
    "ab", "ac", "ae", "sr", "tb", "cb", "te", "er", "ep", "ey",
    "ch", "db", "ah", "ag", "ls", "im", "oc", "idx", "idq",
)
USRTH_NUMERIC_FIELDS = {
    "ab", "ac", "ae", "sr", "tb", "cb", "te", "er", "ep", "ey",
    "ch", "db", "ls", "idx", "idq",
}
USRTH_EXPANDED_NAMES = {
    "ab": "apparent_bearing_math_deg",
    "ac": "apparent_bearing_compass_deg",
    "ae": "apparent_elevation_deg",
    "sr": "slant_range_m",
    "tb": "true_bearing_math_deg",
    "cb": "true_bearing_compass_deg",
    "te": "true_elevation_deg",
    "er": "receiver_roll_deg",
    "ep": "receiver_pitch_deg",
    "ey": "receiver_yaw_math_deg",
    "ch": "receiver_heading_compass_deg",
    "db": "analog_agc_gain_db",
    "ah": "cpu_autosync_capable",
    "ag": "gnss_autosync_capable",
    "ls": "seconds_since_sync",
    "im": "imu_status",
    "oc": "operating_channel",
    "idx": "transponder_id_decoded",
    "idq": "transponder_id_queried",
}
ROVL_MESSAGE_TYPES = {"USRTH", "USINF", "USTXT", "USERR", "USDEB"}


def nmea_checksum(payload: str | bytes) -> int:
    """Return the NMEA XOR checksum for bytes between ``$`` and ``*``."""
    raw = payload.encode("ascii", "replace") if isinstance(payload, str) else bytes(payload)
    if raw.startswith(b"$"):
        raw = raw[1:]
    if b"*" in raw:
        raw = raw.split(b"*", 1)[0]
    value = 0
    for byte in raw:
        value ^= byte
    return value


def build_nmea_sentence(body: str, line_ending: bytes = b"\r\n") -> bytes:
    """Build a checksummed sentence; used by the deterministic demo/tests."""
    body = body[1:] if body.startswith("$") else body
    body = body.split("*", 1)[0]
    return ("$%s*%02X" % (body, nmea_checksum(body))).encode("ascii") + line_ending


def _number(value: str):
    if value == "":
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _gnss_metadata(message_type: str, fields: Sequence[str]) -> Dict[str, object]:
    result: Dict[str, object] = {}
    if not fields:
        return result
    if message_type.endswith("RMC"):
        result["device_time_utc"] = fields[0] or None
        result["device_date_utc"] = fields[8] if len(fields) > 8 and fields[8] else None
        if result["device_time_utc"] and result["device_date_utc"]:
            try:
                stamp = str(result["device_date_utc"]) + str(result["device_time_utc"]).split(".", 1)[0]
                dt = datetime.strptime(stamp, "%d%m%y%H%M%S").replace(tzinfo=timezone.utc)
                result["gnss_utc_ns"] = int(dt.timestamp() * 1_000_000_000)
            except (TypeError, ValueError, OverflowError):
                result["gnss_utc_ns"] = None
    elif message_type.endswith("GGA"):
        result["device_time_utc"] = fields[0] or None
    return result


def parse_sentence(raw: str | bytes) -> Dict[str, object]:
    """Parse one NMEA-like line without rejecting partial or future messages."""
    if isinstance(raw, bytes):
        decoded = raw.decode("ascii", "replace")
    else:
        decoded = str(raw)
    sentence = decoded.rstrip("\r\n")
    line_ending = decoded[len(sentence):]
    result: Dict[str, object] = {
        "raw_sentence": sentence,
        "line_ending": line_ending,
        "message_type": "UNKNOWN",
        "checksum_present": False,
        "checksum_text": None,
        "checksum_calculated": None,
        "checksum_ok": None,
        "parse_ok": False,
        "parse_error": None,
        "fields": {},
        "extra_fields": [],
    }
    if not sentence.startswith("$"):
        result["parse_error"] = "sentence does not start with '$'"
        return result

    content = sentence[1:]
    checksum_text = None
    if "*" in content:
        content, checksum_text = content.split("*", 1)
        result["checksum_present"] = True
        result["checksum_text"] = checksum_text
        calculated = nmea_checksum(content)
        result["checksum_calculated"] = calculated
        if len(checksum_text) >= 2:
            try:
                expected = int(checksum_text[:2], 16)
                result["checksum_expected"] = expected
                result["checksum_ok"] = expected == calculated
            except ValueError:
                result["checksum_ok"] = False
        else:
            result["checksum_ok"] = False

    parts = content.split(",")
    message_type = parts[0].upper() if parts else "UNKNOWN"
    raw_fields = parts[1:]
    result["message_type"] = message_type
    result["raw_fields"] = raw_fields

    if message_type == "USRTH":
        known: Dict[str, object] = {}
        raw_known: Dict[str, Optional[str]] = {}
        for index, name in enumerate(USRTH_FIELD_NAMES):
            text = raw_fields[index] if index < len(raw_fields) else ""
            raw_known[name] = text if text != "" else None
            value = _number(text) if name in USRTH_NUMERIC_FIELDS else (text or None)
            known[name] = value
            result[name] = value
            result[USRTH_EXPANDED_NAMES[name]] = value
        result["fields"] = known
        result["raw_known_fields"] = raw_known
        result["extra_fields"] = list(raw_fields[len(USRTH_FIELD_NAMES):])
    elif message_type in {"USINF", "USTXT", "USERR", "USDEB"}:
        result["text"] = ",".join(raw_fields)
        result["fields"] = {"text": result["text"]}
    elif message_type.startswith(("GP", "GN")):
        result["source"] = "forwarded_gnss"
        result["fields"] = {"values": list(raw_fields)}
        result.update(_gnss_metadata(message_type, raw_fields))
    else:
        result["fields"] = {"values": list(raw_fields)}

    if result["checksum_present"] and result["checksum_ok"] is False:
        result["parse_error"] = "checksum mismatch or malformed checksum"
    result["parse_ok"] = result["parse_error"] is None
    return result


def imu_status_valid(value: object) -> Optional[bool]:
    """Conservatively interpret published Mk II/CIMU status values."""
    if value is None or str(value).strip() == "":
        return None
    text = str(value).strip().upper()
    if text == "CIMU":
        return True
    if len(text) == 4 and text.isdigit():
        return text[0] != "0" and any(char != "0" for char in text[1:])
    if text in {"UNKNOWN", "INVALID", "VOID", "NONE", "OFF", "0000"}:
        return False
    return None


def compute_local_position(record: Dict[str, object]) -> Dict[str, object]:
    """Derive a local position without mixing Compass/NED and Math/ENU.

    True Compass values become North/East.  If true values are unavailable,
    apparent Math values are represented as receiver-relative X/Y instead.
    """
    slant = _number(str(record.get("sr"))) if record.get("sr") is not None else None
    base: Dict[str, object] = {
        "lock": False,
        "coordinate_frame": None,
        "method": None,
        "slant_range_m": slant,
        "horizontal_range_m": None,
        "vertical_up_m": None,
        "north_m": None,
        "east_m": None,
        "relative_x_m": None,
        "relative_y_m": None,
        "bearing_deg": None,
        "elevation_deg": None,
        "imu_valid": imu_status_valid(record.get("im")),
    }
    if slant is None or slant <= 0.0:
        return base

    cb, te = record.get("cb"), record.get("te")
    if cb is not None and te is not None and base["imu_valid"] is not False:
        bearing, elevation = float(cb), float(te)
        horizontal = slant * math.cos(math.radians(elevation))
        base.update({
            "lock": True,
            "coordinate_frame": "NED_HORIZONTAL_UP_VERTICAL",
            "method": "true_compass_cb_te",
            "horizontal_range_m": horizontal,
            "vertical_up_m": slant * math.sin(math.radians(elevation)),
            "north_m": horizontal * math.cos(math.radians(bearing)),
            "east_m": horizontal * math.sin(math.radians(bearing)),
            "bearing_deg": bearing,
            "elevation_deg": elevation,
        })
        return base

    ab, ae = record.get("ab"), record.get("ae")
    if ab is None or ae is None:
        return base
    bearing, elevation = float(ab), float(ae)
    horizontal = slant * math.cos(math.radians(elevation))
    base.update({
        "lock": True,
        "coordinate_frame": "RECEIVER_RELATIVE_MATH",
        "method": "apparent_math_ab_ae",
        "horizontal_range_m": horizontal,
        "vertical_up_m": slant * math.sin(math.radians(elevation)),
        "relative_x_m": horizontal * math.cos(math.radians(bearing)),
        "relative_y_m": horizontal * math.sin(math.radians(bearing)),
        "bearing_deg": bearing,
        "elevation_deg": elevation,
    })
    return base


class NMEALineFramer:
    """Incrementally frame LF/CRLF serial chunks while preserving bytes."""

    def __init__(self, max_buffer: int = 65_536):
        self.buffer = bytearray()
        self.max_buffer = int(max_buffer)

    def feed(self, chunk: bytes) -> List[bytes]:
        self.buffer.extend(bytes(chunk))
        frames: List[bytes] = []
        while True:
            try:
                end = self.buffer.index(0x0A) + 1
            except ValueError:
                break
            frames.append(bytes(self.buffer[:end]))
            del self.buffer[:end]
        if len(self.buffer) > self.max_buffer:
            # Keep only a plausible partial sentence after serial garbage.
            start = self.buffer.rfind(b"$")
            self.buffer[:] = self.buffer[start:] if start >= 0 else b""
        return frames

    def flush(self) -> Optional[bytes]:
        if not self.buffer:
            return None
        frame = bytes(self.buffer)
        self.buffer.clear()
        return frame


def list_serial_devices() -> List[Dict[str, object]]:
    """List current serial ports without opening or changing any device."""
    if list_ports is None:
        return []
    devices = []
    for item in list_ports.comports():
        devices.append({
            "device": item.device,
            "description": getattr(item, "description", "") or "",
            "manufacturer": getattr(item, "manufacturer", None),
            "serial_number": getattr(item, "serial_number", None),
            "vid": getattr(item, "vid", None),
            "pid": getattr(item, "pid", None),
        })
    return devices


def _open_serial(port: str, timeout: float, serial_factory=None):
    if serial_factory is not None:
        return serial_factory(
            port=port, baudrate=ROVL_BAUD, bytesize=8, parity="N",
            stopbits=1, timeout=timeout,
        )
    if serial is None:
        raise RuntimeError("pyserial is not installed")
    return serial.Serial(
        port=port, baudrate=ROVL_BAUD, bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
        timeout=timeout, write_timeout=0,
    )


def discover_rovl_port(
    devices: Optional[Iterable[object]] = None,
    serial_factory=None,
    probe_seconds: float = 0.45,
) -> Optional[str]:
    """Passively identify a ROVL by briefly reading each enumerated port."""
    candidates = list(devices) if devices is not None else list_serial_devices()
    for item in candidates:
        port = item if isinstance(item, str) else getattr(item, "device", None)
        if port is None and isinstance(item, dict):
            port = item.get("device")
        if not port:
            continue
        handle = None
        try:
            handle = _open_serial(str(port), min(0.1, probe_seconds), serial_factory)
            framer = NMEALineFramer()
            deadline = time.monotonic() + max(0.05, probe_seconds)
            while time.monotonic() < deadline:
                for frame in framer.feed(handle.read(4096) or b""):
                    parsed = parse_sentence(frame)
                    if parsed["message_type"] in ROVL_MESSAGE_TYPES:
                        return str(port)
        except Exception:
            continue
        finally:
            if handle is not None:
                try:
                    handle.close()
                except Exception:
                    pass
    return None


class _LimitedEvents:
    def __init__(self, events: queue.Queue):
        self.events = events
        self.counts: Dict[str, int] = {}

    def emit(self, kind: str, data: object, every: int = 50) -> None:
        count = self.counts.get(kind, 0) + 1
        self.counts[kind] = count
        if count == 1 or count % every == 0:
            payload = dict(data) if isinstance(data, dict) else {"detail": data}
            payload["occurrence_count"] = count
            self.events.put((kind, payload))


class ROVLWorker(threading.Thread):
    """Read-only serial worker for a physical ROVL receiver/transceiver."""

    def __init__(
        self,
        port: Optional[str],
        events: queue.Queue,
        serial_factory=None,
        devices: Optional[Iterable[object]] = None,
        probe_seconds: float = 0.45,
    ):
        super().__init__(daemon=True, name="rovl-read-only")
        self.requested_port = None if not port or str(port).lower() == "auto" else str(port)
        self.events = events
        self.serial_factory = serial_factory
        self.devices = devices
        self.probe_seconds = float(probe_seconds)
        self.stop_event = threading.Event()
        self.handle = None
        self.port: Optional[str] = None
        self._source_offset = 0
        self._last_lock = False

    def stop(self) -> None:
        self.stop_event.set()
        handle = self.handle
        if handle is not None:
            try:
                handle.close()
            except Exception:
                pass

    def _sample(self, frame: bytes) -> Dict[str, object]:
        host_monotonic_ns = time.monotonic_ns()
        host_utc_ns = time.time_ns()
        parsed = parse_sentence(frame)
        position = compute_local_position(parsed) if parsed["message_type"] == "USRTH" else None
        sample = {
            "host_monotonic_ns": host_monotonic_ns,
            "host_utc_ns": host_utc_ns,
            "raw_bytes": frame,
            "source_byte_offset": self._source_offset,
            "byte_length": len(frame),
            "parsed": parsed,
            "position": position,
            "synthetic": False,
        }
        self._source_offset += len(frame)
        return sample

    def run(self) -> None:
        limited = _LimitedEvents(self.events)
        try:
            self.port = self.requested_port or discover_rovl_port(
                self.devices, self.serial_factory, self.probe_seconds,
            )
            if not self.port:
                self.events.put(("rovl_unavailable", {"reason": "no passive ROVL stream detected"}))
                return
            self.handle = _open_serial(self.port, 0.2, self.serial_factory)
            self.events.put(("rovl_connected", {
                "port": self.port, "baud": ROVL_BAUD, "mode": "read-only", "synthetic": False,
            }))
            framer = NMEALineFramer()
            while not self.stop_event.is_set():
                chunk = self.handle.read(4096) or b""
                if not chunk:
                    continue
                for frame in framer.feed(chunk):
                    sample = self._sample(frame)
                    parsed = sample["parsed"]
                    self.events.put(("rovl_sample", sample))
                    if parsed["checksum_present"] and parsed["checksum_ok"] is False:
                        limited.emit("rovl_checksum_error", {"raw_sentence": parsed["raw_sentence"]})
                    elif not parsed["parse_ok"]:
                        limited.emit("rovl_parser_error", {"raw_sentence": parsed["raw_sentence"], "error": parsed["parse_error"]})
                    position = sample.get("position")
                    locked = bool(position and position.get("lock"))
                    if locked != self._last_lock:
                        self.events.put(("rovl_lock_acquired" if locked else "rovl_lock_lost", {
                            "port": self.port, "method": position.get("method") if position else None,
                        }))
                        self._last_lock = locked
        except Exception as exc:
            if not self.stop_event.is_set():
                self.events.put(("rovl_serial_error", {"port": self.port, "error": str(exc)}))
        finally:
            if self.handle is not None:
                try:
                    self.handle.close()
                except Exception:
                    pass
            self.handle = None
            self.events.put(("rovl_disconnected", {"port": self.port, "synthetic": False}))


class DemoROVLWorker(threading.Thread):
    """Deterministic ~1 Hz ROVL source for hardware-free UI validation."""

    def __init__(self, events: queue.Queue, rate_hz: float = 1.0):
        super().__init__(daemon=True, name="rovl-synthetic-demo")
        self.events = events
        self.rate_hz = max(0.1, float(rate_hz))
        self.stop_event = threading.Event()
        self.port = "DEMO"
        self.index = 0
        self._offset = 0

    def stop(self) -> None:
        self.stop_event.set()

    def run(self) -> None:
        self.events.put(("rovl_connected", {
            "port": self.port, "baud": ROVL_BAUD, "mode": "synthetic", "synthetic": True,
        }))
        self.events.put(("rovl_lock_acquired", {"port": self.port, "method": "true_compass_cb_te"}))
        period = 1.0 / self.rate_hz
        while not self.stop_event.is_set():
            phase = self.index * 0.16
            slant = 10.0 + 1.2 * math.sin(phase * 0.7)
            compass = (35.0 + 18.0 * math.sin(phase)) % 360.0
            elevation = -18.0 - 5.0 * math.cos(phase * 0.55)
            math_bearing = (90.0 - compass) % 360.0
            fields = [
                math_bearing, compass, elevation, slant, math_bearing,
                compass, elevation, 1.2, -0.7, 78.2, 11.8, 24,
                "T", "T", 0.2, "CIMU", "B", 3, 3,
            ]
            body = "USRTH," + ",".join(str(round(value, 3)) if isinstance(value, float) else str(value) for value in fields)
            frame = build_nmea_sentence(body)
            parsed = parse_sentence(frame)
            host_monotonic_ns = time.monotonic_ns()
            sample = {
                "host_monotonic_ns": host_monotonic_ns,
                "host_utc_ns": time.time_ns(),
                "raw_bytes": frame,
                "source_byte_offset": self._offset,
                "byte_length": len(frame),
                "parsed": parsed,
                "position": compute_local_position(parsed),
                "synthetic": True,
            }
            self._offset += len(frame)
            self.events.put(("rovl_sample", sample))
            self.index += 1
            self.stop_event.wait(period)
        self.events.put(("rovl_disconnected", {"port": self.port, "synthetic": True}))


__all__ = [
    "DemoROVLWorker", "NMEALineFramer", "ROVLWorker", "ROVL_BAUD",
    "USRTH_FIELD_NAMES", "build_nmea_sentence", "compute_local_position",
    "discover_rovl_port", "imu_status_valid", "list_serial_devices",
    "nmea_checksum", "parse_sentence",
]
