"""Offline checks for the Surveyor log viewer."""

from pathlib import Path
import struct
import tempfile
import unittest

from bluerov_recorder.log_viewer import (
    CHANNEL_DATA_HEADER,
    END_PING,
    PACKET_HEADER,
    cross_track_depth,
    load_raw_intensity,
    load_rovl_positions,
    nearest_rovl_sample,
    rovl_trajectory,
    scan_svlog,
)


ROOT = Path(__file__).resolve().parents[1]
SAMPLE_DIR = ROOT / "records" / "sonar" / "rover_20260910_SonarView"


class SonarLogViewerTests(unittest.TestCase):
    @staticmethod
    def _packet(message_id, payload):
        return PACKET_HEADER.pack(66, 82, len(payload), message_id, 0, 255) + payload + b"\0\0"

    def _synthetic_channel_log(self, packet_count=8, trailing_bytes=0):
        parts = []
        for ch in range(0, packet_count * 2, 2):
            header = CHANNEL_DATA_HEADER.pack(123, 60.54, 0, 0, 40288, ch, ch + 1, 200)
            samples = [1.0, 0.0] * 200 + [1.0, 0.0] * 200
            payload = header + struct.pack("<800f", *samples) + (b"\0" * trailing_bytes)
            parts.append(self._packet(3009, payload))
        end = END_PING.pack(0, 0.5, 10.0, 0, 0, 1, 123, 0, 0, 0, 0, 0, 0, 100.0, 0, 200, 200, 0, 0, 0, 0, 1789042766364)
        parts.append(self._packet(3010, end))
        return b"".join(parts)

    def test_confirmed_channel_data_is_beamformed(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "synthetic.svlog"
            path.write_bytes(self._synthetic_channel_log(trailing_bytes=9600))
            log = scan_svlog(path)
            record = log.pings[0]
            self.assertTrue(record.has_channel_data)
            self.assertEqual(record.bins, 200)
            self.assertIn("trailing bytes", record.channel_data_note)
            decoded = load_raw_intensity(log, record)
            self.assertIsNotNone(decoded)
            matrix, start_m, end_m = decoded
            self.assertEqual((len(matrix), len(matrix[0])), (81, 200))
            self.assertLess(start_m, end_m)
            self.assertGreater(max(max(row) for row in matrix), 0.0)

    def test_incomplete_channel_data_is_not_claimed_available(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "incomplete.svlog"
            path.write_bytes(self._synthetic_channel_log(packet_count=7))
            log = scan_svlog(path)
            self.assertFalse(log.pings[0].has_channel_data)
            self.assertIn("incomplete channel set", log.pings[0].channel_data_note)

    def test_surveyor_log_has_points_and_channel_data(self):
        path = SAMPLE_DIR / "2026-09-10-12-19.svlog"
        if not path.exists():
            self.skipTest("downloaded rover sample is not present")
        log = scan_svlog(path)
        self.assertGreater(len(log.pings), 700)
        self.assertGreater(log.total_points, 9000)
        self.assertTrue(log.pings[0].has_channel_data)

    def test_cross_track_projection_uses_radians(self):
        path = SAMPLE_DIR / "2026-09-10-12-19.svlog"
        if not path.exists():
            self.skipTest("downloaded rover sample is not present")
        log = scan_svlog(path)
        point = log.pings[0].points[0]
        projected = cross_track_depth(log.pings[0])[0]
        self.assertAlmostEqual(projected[0], point[1] * __import__("math").sin(point[0]), places=6)
        self.assertAlmostEqual(projected[1], -point[1] * __import__("math").cos(point[0]), places=6)

    def test_beamformed_fan_is_lazy_and_rectangular(self):
        path = SAMPLE_DIR / "2026-09-10-12-19.svlog"
        if not path.exists():
            self.skipTest("downloaded rover sample is not present")
        log = scan_svlog(path)
        decoded = load_raw_intensity(log, log.pings[0])
        self.assertIsNotNone(decoded)
        matrix, range_start, range_end = decoded
        self.assertEqual(len(matrix), 81)
        self.assertEqual(len(matrix[0]), 200)
        self.assertLess(range_start, range_end)

    def test_detections_only_log_has_no_raw_intensity(self):
        path = SAMPLE_DIR / "2026-09-10-13-14.svlog"
        if not path.exists():
            self.skipTest("downloaded rover sample is not present")
        log = scan_svlog(path)
        self.assertTrue(log.pings)
        self.assertFalse(log.pings[0].has_raw_intensity)

    def test_rovl_fixture_loads_trajectory_and_nearest_sample(self):
        fixture = ROOT / "tests" / "fixtures" / "rovl_positions.jsonl"
        samples = load_rovl_positions(fixture)
        self.assertEqual(len(samples), 2)
        nearest = nearest_rovl_sample(samples, 1_800_000_000)
        self.assertEqual(nearest["line_index"], 1)
        track = rovl_trajectory(samples)
        self.assertEqual(track[0], (2.0, 8.0, "NED_HORIZONTAL_UP_VERTICAL"))

    def test_legacy_session_without_rovl_remains_readable(self):
        with tempfile.TemporaryDirectory() as folder:
            self.assertEqual(load_rovl_positions(Path(folder)), [])
            self.assertIsNone(nearest_rovl_sample([], 123))


if __name__ == "__main__":
    unittest.main()
