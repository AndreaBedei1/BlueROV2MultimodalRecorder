"""Hardware-free acceptance soak for the current live streams.

This deliberately contains no Surveyor imports or network endpoints.
"""
from __future__ import annotations
import argparse, json, queue, sys, tempfile, threading, time
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[1]; sys.path.insert(0, str(ROOT / "src"))
from bluerov_recorder.runtime import BufferedJsonlWriter, LatestValueMailbox, MetricsRegistry

def run(duration=180.0):
    with tempfile.TemporaryDirectory(prefix="bluerov_live_soak_") as tmp:
        out = Path(tmp); m = MetricsRegistry(); cam = LatestValueMailbox(m, "camera_preview_dropped", "camera_preview_published"); ping = LatestValueMailbox(m, "ping1d_preview_dropped", "ping1d_preview_published"); rovl = LatestValueMailbox(m, "rovl_preview_dropped", "rovl_preview_published")
        pw, rw = BufferedJsonlWriter(out / "ping1d.jsonl", m, "ping1d"), BufferedJsonlWriter(out / "rovl_positions.jsonl", m, "rovl"); stop = threading.Event(); start = time.monotonic(); rss = []
        def camera():
            tick = time.perf_counter(); i = 0
            while not stop.is_set():
                cam.publish((np.empty((180, 320, 3), dtype=np.uint8), {"frame_index": i})); m.increment("camera_frames_decoded"); i += 1; tick += 1 / 30; stop.wait(max(0, tick - time.perf_counter()))
        def sensors():
            tick = time.perf_counter(); i = 0
            while not stop.is_set():
                now = time.monotonic_ns(); sample = {"host_monotonic_ns": now, "distance_m": 2.0, "confidence": 95, "profile": [i % 255] * 256}; pw.submit(sample); ping.publish(sample); m.increment("ping1d_samples_received"); rw.submit({"host_monotonic_ns": now, "lock": True, "slant_range_m": 5.0}); rovl.publish({"host_monotonic_ns": now, "lock": True}); m.increment("rovl_samples_received"); i += 1; tick += .1; stop.wait(max(0, tick - time.perf_counter()))
        threads = [threading.Thread(target=camera), threading.Thread(target=sensors)]
        for t in threads: t.start()
        next_cam = time.perf_counter(); next_rss = next_cam
        try:
            while time.monotonic() - start < duration:
                now = time.perf_counter()
                if now >= next_cam:
                    if cam.take() is not None: m.increment("camera_frames_previewed")
                    next_cam += 1 / 20
                ping.take(); rovl.take()
                if now >= next_rss:
                    try:
                        import psutil; rss.append(psutil.Process().memory_info().rss)
                    except ImportError: pass
                    next_rss += 1
                time.sleep(.002)
        finally:
            active = time.monotonic() - start; stop.set(); [t.join(5) for t in threads]; pw.close(); rw.close()
        s = m.snapshot(); s.update(active_duration_s=active, camera_preview_fps=s.get("camera_frames_previewed", 0) / active, rss_min_mb=min(rss) / 1048576 if rss else None, rss_max_mb=max(rss) / 1048576 if rss else None, rss_growth_mb=(rss[-1] - rss[0]) / 1048576 if len(rss) > 1 else None)
        s["success"] = s.get("camera_preview_fps", 0) >= 15 and s.get("ping1d_samples_received", 0) == s.get("ping1d_records_written", 0) and s.get("rovl_samples_received", 0) == s.get("rovl_records_written", 0) and s.get("camera_preview_dropped", 0) >= 0
        return s

if __name__ == "__main__":
    p = argparse.ArgumentParser(); p.add_argument("--duration", type=float, default=180); a = p.parse_args(); result = run(a.duration); print(json.dumps(result, indent=2, sort_keys=True)); raise SystemExit(0 if result["success"] else 1)
