"""Hardware-free coverage for the optional passive ROV Locator stream."""

import csv
import importlib.util
import json
import queue
import sys
import tempfile
import threading
import time
from pathlib import Path

import pytest

from bluerov_recorder.app import SessionRecorder
from bluerov_recorder.rovl import (
    DemoROVLWorker,
    NMEALineFramer,
    ROVLWorker,
    build_nmea_sentence,
    compute_local_position,
    discover_rovl_port,
    nmea_checksum,
    parse_sentence,
)


def usrth_bytes(extra=""):
    body = "USRTH,358.5,1.5,-20.0,10.8,52.8,43.2,-20.4,1.2,-0.7,78.1,11.8,24,T,T,0.2,CIMU,B,3,3"
    if extra:
        body += "," + extra
    return build_nmea_sentence(body)


def make_sample(raw=None, host_monotonic_ns=2_000_000_000, host_utc_ns=4_000_000_000):
    raw = raw or usrth_bytes()
    parsed = parse_sentence(raw)
    return {
        "raw_bytes": raw,
        "host_monotonic_ns": host_monotonic_ns,
        "host_utc_ns": host_utc_ns,
        "parsed": parsed,
        "position": compute_local_position(parsed),
        "synthetic": False,
    }


def test_nmea_checksum_valid_invalid_absent_and_lowercase():
    raw = usrth_bytes()
    parsed = parse_sentence(raw)
    assert parsed["checksum_present"] is True
    assert parsed["checksum_ok"] is True
    assert nmea_checksum(raw) == int(parsed["checksum_text"], 16)
    lowercase = raw[:-4] + raw[-4:-2].lower() + b"\r\n"
    assert parse_sentence(lowercase)["checksum_ok"] is True
    damaged = raw.replace(b"10.8", b"10.9")
    assert parse_sentence(damaged)["checksum_ok"] is False
    absent = parse_sentence(b"$USINF,hello\r\n")
    assert absent["checksum_present"] is False
    assert absent["checksum_ok"] is None


def test_usrth_full_empty_short_and_future_trailing_fields():
    parsed = parse_sentence(usrth_bytes("future,42"))
    assert parsed["message_type"] == "USRTH"
    assert parsed["slant_range_m"] == pytest.approx(10.8)
    assert parsed["true_bearing_compass_deg"] == pytest.approx(43.2)
    assert parsed["imu_status"] == "CIMU"
    assert parsed["extra_fields"] == ["future", "42"]

    short = parse_sentence(build_nmea_sentence("USRTH,12,78,-4,8"))
    assert short["sr"] == pytest.approx(8.0)
    assert short["cb"] is None
    assert short["extra_fields"] == []

    empty = parse_sentence(build_nmea_sentence("USRTH,,,,,,,,,,,,,,,,,,"))
    assert empty["ab"] is None
    assert empty["sr"] is None
    assert compute_local_position(empty)["lock"] is False


@pytest.mark.parametrize("body,message_type", [
    ("USINF,IMU status All OK", "USINF"),
    ("USTXT,free form text", "USTXT"),
    ("USERR,example error", "USERR"),
    ("USDEB,debug,field", "USDEB"),
])
def test_generic_cerulean_messages(body, message_type):
    parsed = parse_sentence(build_nmea_sentence(body))
    assert parsed["message_type"] == message_type
    assert parsed["parse_ok"] is True
    assert parsed["text"]


def test_forwarded_gnss_sentence_is_preserved_separately():
    parsed = parse_sentence(build_nmea_sentence("GPRMC,123519,A,4807.038,N,01131.000,E,0.0,0.0,230394,,,A"))
    assert parsed["message_type"] == "GPRMC"
    assert parsed["source"] == "forwarded_gnss"
    assert parsed["device_time_utc"] == "123519"
    assert parsed["gnss_utc_ns"] is not None


def test_true_compass_coordinate_conversion_uses_radians_and_correct_axes():
    position = compute_local_position({"sr": 10.0, "cb": 90.0, "te": 0.0, "im": "CIMU", "ab": None, "ae": None})
    assert position["lock"] is True
    assert position["coordinate_frame"] == "NED_HORIZONTAL_UP_VERTICAL"
    assert position["north_m"] == pytest.approx(0.0, abs=1e-9)
    assert position["east_m"] == pytest.approx(10.0)
    assert position["vertical_up_m"] == pytest.approx(0.0)

    elevated = compute_local_position({"sr": 10.0, "cb": 0.0, "te": -30.0, "im": "3333", "ab": None, "ae": None})
    assert elevated["horizontal_range_m"] == pytest.approx(8.660254, rel=1e-6)
    assert elevated["vertical_up_m"] == pytest.approx(-5.0)


def test_apparent_fallback_is_relative_and_no_lock_when_incomplete():
    relative = compute_local_position({"sr": 8.0, "cb": None, "te": None, "im": "0000", "ab": 90.0, "ae": 0.0})
    assert relative["lock"] is True
    assert relative["coordinate_frame"] == "RECEIVER_RELATIVE_MATH"
    assert relative["north_m"] is None
    assert relative["relative_x_m"] == pytest.approx(0.0, abs=1e-9)
    assert relative["relative_y_m"] == pytest.approx(8.0)
    assert compute_local_position({"sr": 8.0, "ab": None, "ae": 0.0})["lock"] is False
    assert compute_local_position({"sr": None, "ab": 2.0, "ae": 0.0})["lock"] is False


def test_incremental_framing_handles_fragmented_and_multiple_packets_exactly():
    first = build_nmea_sentence("USINF,one")
    second = build_nmea_sentence("USERR,two")
    framer = NMEALineFramer()
    assert framer.feed(first[:5]) == []
    assert framer.feed(first[5:] + second[:4]) == [first]
    assert framer.feed(second[4:]) == [second]
    assert framer.flush() is None


class FakeSerial:
    def __init__(self, chunks):
        self.chunks = list(chunks)
        self.closed = False
        self.write_calls = 0

    def read(self, _size):
        if self.chunks:
            return self.chunks.pop(0)
        time.sleep(0.005)
        return b""

    def write(self, _data):
        self.write_calls += 1
        raise AssertionError("ROVL acquisition must never write to serial")

    def close(self):
        self.closed = True


def test_serial_worker_timestamps_every_line_and_never_writes():
    handle = FakeSerial([usrth_bytes()[:13], usrth_bytes()[13:]])
    events = queue.Queue()
    worker = ROVLWorker("COM9", events, serial_factory=lambda **_kwargs: handle)
    worker.start()
    deadline = time.time() + 2.0
    sample = None
    while time.time() < deadline and sample is None:
        try:
            kind, data = events.get(timeout=0.1)
        except queue.Empty:
            continue
        if kind == "rovl_sample":
            sample = data
    worker.stop()
    worker.join(timeout=1.0)
    assert sample is not None
    assert sample["host_monotonic_ns"] > 0
    assert sample["host_utc_ns"] > 0
    assert sample["raw_bytes"] == usrth_bytes()
    assert handle.write_calls == 0
    assert handle.closed is True


def test_auto_discovery_uses_passive_content_not_vid_pid_guesses():
    handles = {
        "COM1": FakeSerial([b"unrelated\r\n"]),
        "COM7": FakeSerial([build_nmea_sentence("USINF,Cerulean Sonar RTH ROV Locator")]),
    }
    selected = discover_rovl_port(
        devices=[{"device": "COM1"}, {"device": "COM7"}],
        serial_factory=lambda port, **_kwargs: handles[port],
        probe_seconds=0.03,
    )
    assert selected == "COM7"
    assert all(handle.write_calls == 0 for handle in handles.values())


def test_worker_without_any_com_port_is_optional_and_terminates_cleanly():
    events = queue.Queue()
    worker = ROVLWorker("Auto", events, devices=[])
    worker.start()
    worker.join(timeout=1.0)
    assert not worker.is_alive()
    kinds = []
    while not events.empty():
        kinds.append(events.get_nowait()[0])
    assert "rovl_unavailable" in kinds
    assert "rovl_disconnected" in kinds


def test_session_without_rovl_has_no_rovl_artifacts():
    with tempfile.TemporaryDirectory() as directory:
        session = SessionRecorder(Path(directory))
        session.close()
        metadata = json.loads((session.directory / "session.json").read_text(encoding="utf-8"))
        assert metadata["rovl"]["enabled"] is False
        assert not (session.directory / "rovl_raw.nmea").exists()
        assert not (session.directory / "rovl_timestamps.csv").exists()
        assert not (session.directory / "rovl_positions.jsonl").exists()


def test_session_with_rovl_preserves_raw_offsets_timestamps_and_nulls():
    raw = usrth_bytes()
    with tempfile.TemporaryDirectory() as directory:
        session = SessionRecorder(Path(directory), rovl_connected=True, rovl_port="COM6")
        sample = make_sample(raw, session.session_start_monotonic_ns + 1_250_000_000, session.session_start_utc_ns + 1_250_000_000)
        assert session.write_rovl_sample(sample) is True
        session.close()
        assert (session.directory / "rovl_raw.nmea").read_bytes() == raw
        with (session.directory / "rovl_timestamps.csv").open(newline="", encoding="utf-8") as stream:
            row = list(csv.DictReader(stream))[0]
        assert row["line_index"] == "0"
        assert row["session_time_s"] == "1.25"
        assert row["checksum_ok"] == "True"
        assert row["byte_offset"] == "0"
        assert row["byte_length"] == str(len(raw))
        record = json.loads((session.directory / "rovl_positions.jsonl").read_text(encoding="utf-8"))
        assert record["raw_reference"]["byte_offset"] == 0
        assert record["position"]["lock"] is True
        assert record["fields"]["ab"] == pytest.approx(358.5)


def test_demo_stream_is_obvious_valid_and_in_expected_operating_envelope():
    events = queue.Queue()
    worker = DemoROVLWorker(events, rate_hz=30.0)
    worker.start()
    sample = None
    deadline = time.time() + 1.0
    while time.time() < deadline and sample is None:
        kind, data = events.get(timeout=0.2)
        if kind == "rovl_sample":
            sample = data
    worker.stop()
    worker.join(timeout=1.0)
    assert sample is not None and sample["synthetic"] is True
    assert sample["parsed"]["checksum_ok"] is True
    assert 8.0 <= sample["position"]["slant_range_m"] <= 12.0
    assert sample["position"]["elevation_deg"] < 0.0


def test_module_executes_when_pyserial_import_is_unavailable(monkeypatch):
    import builtins
    import bluerov_recorder.rovl as installed_module

    original_import = builtins.__import__

    def blocked_import(name, *args, **kwargs):
        if name == "serial" or name.startswith("serial."):
            raise ImportError("blocked for test")
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", blocked_import)
    spec = importlib.util.spec_from_file_location("rovl_without_serial", Path(installed_module.__file__))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module.serial is None
    assert module.list_serial_devices() == []
