"""Safe live dashboard: RGB camera, Ping1D and Cerulean ROVL only.

The Surveyor is intentionally not imported here.  Its acquisition and .svlog
recording belong to BlueOS/SonarView on the vehicle.
"""
from __future__ import annotations
import math, os, queue, time
import tkinter as tk
from pathlib import Path
from PIL import Image, ImageDraw, ImageTk

from .offline_app import (APP_ROOT, BLUEOS_HOST, CAMERA_DEFAULT_PORT, CONTROL_QUEUE_MAX_ITEMS,
                  PING1D_HOST, PING1D_PORT, SESSION_ROOT, BlueOSWorker, CameraWorker,
                  Ping1DWorker, SessionRecorder, default_camera_source)
from .rovl import DemoROVLWorker, ROVLWorker
from .runtime import LatestValueMailbox, MetricsRegistry


class CameraPingROVLViewer(tk.Tk):
    """Three-stream recorder with bounded preview mailboxes and no Surveyor I/O."""
    def __init__(self, offline=False, demo=False, rovl_port="Auto"):
        super().__init__(); self.title("BlueROV2 Camera · Ping1D · ROVL Recorder"); self.geometry("1500x900")
        self.offline, self.demo = bool(offline), bool(demo); self.metrics = MetricsRegistry(); self.events = queue.Queue(maxsize=CONTROL_QUEUE_MAX_ITEMS)
        self.camera_preview = LatestValueMailbox(self.metrics, "camera_preview_dropped", "camera_preview_published")
        self.ping_preview = LatestValueMailbox(self.metrics, "ping1d_preview_dropped", "ping1d_preview_published")
        self.rovl_preview = LatestValueMailbox(self.metrics, "rovl_preview_dropped", "rovl_preview_published")
        self.camera_worker = self.ping_worker = self.rovl_worker = self.blueos_worker = None; self.session = None
        self.latest_rovl = None; self.photos = {}; self.rovl_port = tk.StringVar(value=rovl_port or "Auto")
        self.status = tk.StringVar(value="Pronto — Surveyor esterno via BlueOS / SonarView"); self.record_status = tk.StringVar(value="NOT RECORDING")
        self.performance = tk.StringVar(value="CPU -- | RAM -- | GUI -- | Camera -- | Ping1D -- | ROVL --"); self._previous = {}; self._previous_ns = time.monotonic_ns()
        self._build_ui(); self.protocol("WM_DELETE_WINDOW", self.close); self.after(20, self._poll_events); self.after(20, self._poll_previews); self.after(1000, self._diagnostics)
        if self.demo: self.after(100, self._start_demo)

    def _build_ui(self):
        self.configure(bg="#06141c"); top = tk.Frame(self, bg="#071a24", padx=10, pady=8); top.pack(fill="x")
        self.badges = {}
        items = (("blueos", "BlueOS", BLUEOS_HOST), ("camera", "Camera", "UDP 5600 / 5602"), ("ping", "Ping1D", "%s:%d" % (PING1D_HOST, PING1D_PORT)), ("rovl", "ROVL Mk III", "USB COM"), ("surveyor", "Surveyor", "EXTERNAL / SonarView"))
        for col, (key, title, address) in enumerate(items):
            frame = tk.Frame(top, bg="#081b25", highlightthickness=1, highlightbackground="#21475b", padx=8, pady=4); frame.grid(row=0, column=col, sticky="ew", padx=3); top.columnconfigure(col, weight=1)
            tk.Label(frame, text=title, bg="#081b25", fg="#e9f2f7", font=("Segoe UI", 9, "bold")).pack(); state = tk.StringVar(value="EXTERNAL" if key == "surveyor" else "DISCONNECTED"); tk.Label(frame, textvariable=state, bg="#081b25", fg="#a9bdd0").pack(); tk.Label(frame, text=address, bg="#081b25", fg="#a9bdd0").pack(); self.badges[key] = state
        controls = tk.Frame(self, bg="#071a24", padx=12, pady=5); controls.pack(fill="x")
        for label, command in (("Connect all", self.connect_all), ("Disconnect", self.disconnect_all), ("START SESSION", self.start_session), ("STOP SESSION", self.stop_session)):
            tk.Button(controls, text=label, command=command, bg="#176fd1" if label in ("Connect all", "START SESSION") else "#243847", fg="white", relief="flat", padx=10, pady=5).pack(side="left", padx=3)
        tk.Label(controls, textvariable=self.record_status, bg="#071a24", fg="#ffbd3a").pack(side="left", padx=15); tk.Label(controls, textvariable=self.status, bg="#071a24", fg="#a9bdd0").pack(side="right")
        body = tk.Frame(self, bg="#06141c", padx=8, pady=5); body.pack(fill="both", expand=True); body.columnconfigure(0, weight=3); body.columnconfigure(1, weight=2); body.rowconfigure(0, weight=3); body.rowconfigure(1, weight=2)
        cam = self._panel(body, "RGB CAMERA LIVE", 0, 0, rowspan=2); self.camera_label = tk.Label(cam, text="No RGB frames", bg="#041019", fg="#a9bdd0"); self.camera_label.pack(fill="both", expand=True, padx=8); self.camera_stats = tk.StringVar(value="single ingest · latest-only preview · target ≥15 FPS"); tk.Label(cam, textvariable=self.camera_stats, bg="#071922", fg="#a9bdd0").pack(anchor="w", padx=10, pady=5)
        rov = self._panel(body, "ROV LOCATOR Mk III · TOP-DOWN", 1, 0); self.rovl_label = tk.Label(rov, text="No ROVL position\n(relative frame until true bearing/IMU is available)", bg="#041019", fg="#a9bdd0"); self.rovl_label.pack(fill="both", expand=True, padx=8); self.rovl_stats = tk.StringVar(value="Disconnected"); tk.Label(rov, textvariable=self.rovl_stats, bg="#071922", fg="#a9bdd0", justify="left").pack(anchor="w", padx=10, pady=5)
        ping = self._panel(body, "PING1D · FULL ECHO PROFILE", 1, 1); self.ping_label = tk.Label(ping, text="No Ping1D samples", bg="#041019", fg="#a9bdd0"); self.ping_label.pack(fill="both", expand=True, padx=8); self.ping_stats = tk.StringVar(value="Distance -- · Confidence -- · Profile --"); tk.Label(ping, textvariable=self.ping_stats, bg="#071922", fg="#a9bdd0").pack(anchor="w", padx=10, pady=5)
        tk.Label(self, textvariable=self.performance, bg="#091b25", fg="#9fc4d8", anchor="w", padx=12, pady=4, font=("Consolas", 8)).pack(fill="x", padx=8)

    def _panel(self, parent, title, col, row, rowspan=1):
        frame = tk.Frame(parent, bg="#071922", highlightthickness=1, highlightbackground="#285368"); frame.grid(row=row, column=col, rowspan=rowspan, sticky="nsew", padx=3, pady=3); tk.Label(frame, text=title, bg="#071922", fg="#e9f2f7", font=("Segoe UI", 13, "bold")).pack(anchor="w", padx=10, pady=8); return frame
    def _badge(self, key, value):
        if key in self.badges: self.badges[key].set(str(value))

    def _start_demo(self):
        for key in ("blueos", "camera", "ping", "rovl"): self._badge(key, "DEMO")
        self.rovl_worker = DemoROVLWorker(self.events); self.rovl_worker.start(); self.after(33, self._demo_camera); self.after(100, self._demo_ping)
    def _demo_camera(self):
        if not self.demo or not self.winfo_exists(): return
        try:
            import numpy as np; t = time.monotonic(); frame = np.zeros((360, 640, 3), dtype=np.uint8); frame[:, :, 0] = int(30 + 20 * math.sin(t)); frame[:, :, 1] = 100; frame[:, :, 2] = 160; self.camera_preview.publish((frame, {"host_monotonic_ns": time.monotonic_ns(), "host_utc_ns": time.time_ns(), "frame_index": int(t * 30)})); self.metrics.increment("camera_frames_decoded")
        except Exception: pass
        self.after(33, self._demo_camera)
    def _demo_ping(self):
        if not self.demo or not self.winfo_exists(): return
        now = time.monotonic_ns(); d = {"distance_m": 8.0 + math.sin(now / 1e9), "distance_mm": 8000, "confidence": 95, "host_monotonic_ns": now, "host_utc_ns": time.time_ns()}; p = {"profile": [int(120 + 80 * math.sin(i / 8.0)) for i in range(160)], "scan_start_mm": 300, "scan_length_mm": 8000, "gain": 0, "distance_m": d["distance_m"]}; self.ping_preview.publish((d, p)); self.metrics.increment("ping1d_samples_received"); self.after(100, self._demo_ping)

    def connect_all(self):
        if self.offline or self.demo: return
        self.blueos_worker = self.blueos_worker or BlueOSWorker(BLUEOS_HOST, self.events); self.blueos_worker.start() if not self.blueos_worker.is_alive() else None
        self.ping_worker = self.ping_worker or Ping1DWorker(PING1D_HOST, PING1D_PORT, self.events, self.ping_preview, self.metrics)
        if self.session is not None: self.ping_worker.set_record_callback(self.session.write_ping1d)
        self.ping_worker.start() if not self.ping_worker.is_alive() else None
        self.camera_worker = self.camera_worker or CameraWorker(CAMERA_DEFAULT_PORT, default_camera_source(CAMERA_DEFAULT_PORT), self.events, self.camera_preview, self.metrics)
        if self.session is not None: self.camera_worker.set_packet_callback(self.session.write_camera_packet); self.camera_worker.set_frame_callback(self.session.write_camera_frame)
        self.camera_worker.start() if not self.camera_worker.is_alive() else None
        self.rovl_worker = self.rovl_worker or ROVLWorker(self.rovl_port.get(), self.events, preview_mailbox=self.rovl_preview, metrics=self.metrics)
        if self.session is not None: self.rovl_worker.set_record_callback(self.session.write_rovl_sample)
        self.rovl_worker.start() if not self.rovl_worker.is_alive() else None
        self.status.set("Connessioni avviate — Surveyor esterno, nessun accesso dalla app")

    def start_session(self):
        if self.session is not None: return
        self.session = SessionRecorder(SESSION_ROOT, CAMERA_DEFAULT_PORT, default_camera_source(CAMERA_DEFAULT_PORT), rovl_connected=False, metrics=self.metrics, enable_surveyor=False)
        if self.camera_worker: self.camera_worker.set_packet_callback(self.session.write_camera_packet); self.camera_worker.set_frame_callback(self.session.write_camera_frame)
        if self.ping_worker: self.ping_worker.set_record_callback(self.session.write_ping1d)
        if self.rovl_worker and hasattr(self.rovl_worker, "set_record_callback"): self.rovl_worker.set_record_callback(self.session.write_rovl_sample)
        self.session.write_event("session_started", {"surveyor_recording": "external / BlueOS SonarView"}); self.record_status.set("RECORDING"); self.status.set("Sessione avviata")
    def stop_session(self):
        if self.session is None: return
        session = self.session; self.session = None
        if self.camera_worker: self.camera_worker.set_packet_callback(None); self.camera_worker.set_frame_callback(None)
        if self.ping_worker: self.ping_worker.set_record_callback(None)
        if self.rovl_worker and hasattr(self.rovl_worker, "set_record_callback"): self.rovl_worker.set_record_callback(None)
        session.close(); self.record_status.set("NOT RECORDING"); self.status.set("Sessione finalizzata: %s" % session.directory)
    def disconnect_all(self):
        for worker in (self.camera_worker, self.ping_worker, self.rovl_worker):
            if worker is not None and hasattr(worker, "stop"): worker.stop()
        if self.blueos_worker is not None: self.blueos_worker.stop_event.set()

    def _poll_events(self):
        try:
            for _ in range(64):
                kind, data = self.events.get_nowait()
                if kind == "blueos": self._badge("blueos", "CONNECTED" if data else "DISCONNECTED")
                elif kind == "camera_connected":
                    self._badge("camera", "CONNECTED")
                    if self.session is not None: self.session.update_camera_metadata(dict(data or {}, connected=True, enabled=True))
                elif kind in ("camera_error",): self._badge("camera", "ERROR")
                elif kind == "ping_connected":
                    self._badge("ping", "CONNECTED")
                    if self.session is not None:
                        self.session.session_metadata["ping1d"]["connected"] = True
                        self.session._write_session(self.session.session_metadata)
                elif kind == "ping_error": self._badge("ping", "ERROR")
                elif kind == "rovl_connected": self._badge("rovl", "CONNECTED")
                elif kind in ("rovl_unavailable", "rovl_serial_error"): self._badge("rovl", "OPTIONAL / OFFLINE")
                elif kind == "critical_data_loss": self.status.set("CRITICAL DATA LOSS: %s" % data); self.stop_session()
        except queue.Empty: pass
        self.metrics.set("gui_heartbeat", time.monotonic_ns()); self.after(20, self._poll_events)

    def _poll_previews(self):
        item = self.camera_preview.take()
        if item is not None:
            frame, meta = item
            try:
                image = Image.fromarray(frame[:, :, ::-1]); image.thumbnail((900, 700), Image.Resampling.LANCZOS); self.photos["camera"] = ImageTk.PhotoImage(image); self.camera_label.configure(image=self.photos["camera"], text=""); self.camera_stats.set("%dx%d · decoded frames · preview drops %d" % (frame.shape[1], frame.shape[0], self.metrics.snapshot().get("camera_preview_dropped", 0))); self.metrics.increment("camera_frames_previewed")
                # The ingest worker already sends frame metadata to the
                # session writer.  Preview consumption must never duplicate
                # timestamps or perform disk I/O on the GUI thread.
            except Exception: pass
        item = self.ping_preview.take()
        if item is not None:
            distance, profile = item; self.ping_stats.set("Distance %.2f m · Confidence %s · Profile %d samples" % (distance.get("distance_m", 0), distance.get("confidence", 0), len((profile or {}).get("profile", [])))); self.ping_label.configure(text="%.2f m\nconfidence %s\nfull profile: %d samples" % (distance.get("distance_m", 0), distance.get("confidence", 0), len((profile or {}).get("profile", []))))
        item = self.rovl_preview.take()
        if item is not None:
            self.latest_rovl = item; pos = item.get("position") or {}; self.rovl_stats.set("COM %s · Lock %s\nSlant %.1f m · Bearing %s° · Elevation %s°\nFrame: %s" % (item.get("port") or "--", "YES" if pos.get("lock") else "NO", pos.get("slant_range_m") or 0, pos.get("bearing_deg") if pos.get("bearing_deg") is not None else "--", pos.get("elevation_deg") if pos.get("elevation_deg") is not None else "--", pos.get("coordinate_frame") or "RELATIVE")); self._draw_rovl_map(pos)
        self.after(20, self._poll_previews)
    def _draw_rovl_map(self, pos):
        image = Image.new("RGB", (520, 280), "#041019"); draw = ImageDraw.Draw(image); cx, cy = 260, 145
        draw.text((12, 10), "TOP-DOWN · %s" % ("NORTH/EAST" if pos.get("north_m") is not None else "RELATIVE FRAME"), fill="#c8d9e5")
        for radius in (45, 90, 135): draw.ellipse((cx-radius, cy-radius, cx+radius, cy+radius), outline="#295066")
        draw.line((cx-145, cy, cx+145, cy), fill="#294d60"); draw.line((cx, cy-145, cx, cy+145), fill="#294d60"); draw.ellipse((cx-7, cy-7, cx+7, cy+7), fill="#ff5f60"); draw.text((cx-28, cy+12), "Topside", fill="white")
        if pos.get("lock"):
            east = float(pos.get("east_m") if pos.get("east_m") is not None else pos.get("relative_y_m") or 0); north = float(pos.get("north_m") if pos.get("north_m") is not None else pos.get("relative_x_m") or 0); scale = max(20.0, abs(east), abs(north)); x, y = cx + east / scale * 120, cy - north / scale * 120; draw.line((cx, cy, x, y), fill="#2e9dd9"); draw.polygon(((x, y-8), (x-7, y+7), (x+7, y+7)), fill="#35aef3"); draw.text((x+10, y-12), "ROV %.1fm" % float(pos.get("slant_range_m") or 0), fill="#35aef3")
        self.photos["rovl"] = ImageTk.PhotoImage(image); self.rovl_label.configure(image=self.photos["rovl"], text="")
    def _diagnostics(self):
        now = time.monotonic_ns(); snap = self.metrics.snapshot(); elapsed = max(1e-6, (now - self._previous_ns) / 1e9)
        for name in ("camera_frames_decoded", "ping1d_samples_received", "rovl_samples_received"): snap[name + "_per_s"] = (snap.get(name, 0) - self._previous.get(name, 0)) / elapsed
        try:
            import psutil; proc = psutil.Process(os.getpid()); cpu, rss = proc.cpu_percent(None), proc.memory_info().rss / 1048576.0
        except Exception: cpu, rss = None, None
        self.performance.set("CPU %s | RAM %s MB | GUI %.1f ms | Camera %.1f fps / drops %d | Ping1D %.1f Hz | ROVL %.1f Hz" % ("--" if cpu is None else "%.0f%%" % cpu, "--" if rss is None else "%.0f" % rss, max(0, (time.monotonic_ns() - int(snap.get("gui_heartbeat", now))) / 1e6), snap.get("camera_frames_decoded_per_s", 0), snap.get("camera_preview_dropped", 0), snap.get("ping1d_samples_received_per_s", 0), snap.get("rovl_samples_received_per_s", 0))); self._previous, self._previous_ns = snap, now
        if self.session is not None: self.session.write_diagnostics(snap)
        self.after(1000, self._diagnostics)
    def close(self):
        self.stop_session(); self.disconnect_all()
        for worker in (self.camera_worker, self.ping_worker, self.rovl_worker, self.blueos_worker):
            if worker is not None: worker.join(timeout=5)
        self.destroy()
