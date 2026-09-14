"""Offline Surveyor 240-16 viewer for SonarView .svlog recordings.

The SonarView files are Ping Protocol streams.  This viewer reads the
well-defined end_ping_info (3010) and atof_point_data (3012) messages and
renders the detected sonar returns without touching the vehicle or rewriting
the recordings. Surveyor channel IQ packets (3009) are beamformed lazily for
the optional fan image; only the selected ping is loaded.

Run from the repository root:
    python -m bluerov_recorder.log_viewer
    python -m bluerov_recorder.log_viewer --summary
"""

from __future__ import annotations

import argparse
import json
import math
import mmap
import os
import struct
import threading
import tkinter as tk
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Dict, Iterable, List, Optional, Tuple, Union

from PIL import Image, ImageDraw, ImageTk

try:
    from .processing import (
        CHANNEL_DATA_HEADER as COMMON_CHANNEL_DATA_HEADER,
        beamform_surveyor_channels,
        parse_channel_data_header as common_parse_channel_data_header,
        validate_channel_payloads,
    )
except ImportError:
    from processing import (
        CHANNEL_DATA_HEADER as COMMON_CHANNEL_DATA_HEADER,
        beamform_surveyor_channels,
        parse_channel_data_header as common_parse_channel_data_header,
        validate_channel_payloads,
    )


PACKET_HEADER = struct.Struct("<BBHHBB")
END_PING = struct.Struct("<IfffffIfffffffiHHHBBIQ")
ATOF_HEADER = struct.Struct("<IQffIIfIHH")
# Cerulean/SonarView ChPairGoertzelData (message 3009).  The first 20 bytes
# are protocol fields; signal_cs_ch1/ch2 are Float32 IQ pairs immediately
# afterwards.  The official parser reads 4 * results_per_channel floats.
# Kept as a public compatibility alias for existing offline tools/tests.
CHANNEL_DATA_HEADER = COMMON_CHANNEL_DATA_HEADER

MSG_JSON = 10
MSG_ATTITUDE = 504
MSG_RAW_PROFILE = 3009
MSG_END_PING = 3010
MSG_ATOF = 3012

DEFAULT_ROOT = Path(__file__).resolve().parents[2] / "records" / "sonar"
LOCAL_TZ = datetime.now().astimezone().tzinfo


@dataclass
class PingRecord:
    """Compact representation of one imaging-sonar ping."""

    number: int
    timestamp_ms: int = 0
    range_start_m: float = 0.0
    range_end_m: float = 10.0
    points: List[Tuple[float, float]] = field(default_factory=list)
    sos_mps: float = 1500.0
    ping_hz: float = 0.0
    raw_profile_packets: int = 0
    raw_packet_offsets: List[Tuple[int, int]] = field(default_factory=list)
    bins: int = 0
    channel_data_valid: bool = False
    channel_data_note: str = ""

    @property
    def timestamp(self) -> str:
        if not self.timestamp_ms:
            return "--"
        dt = datetime.fromtimestamp(self.timestamp_ms / 1000.0, timezone.utc)
        return dt.astimezone(LOCAL_TZ).strftime("%d/%m/%Y %H:%M:%S")

    @property
    def range_max(self) -> float:
        if self.points:
            return max(p[1] for p in self.points)
        return self.range_end_m

    @property
    def has_raw_intensity(self) -> bool:
        """Compatibility name: true when validated 3009 channel IQ exists."""
        return self.channel_data_valid

    @property
    def has_channel_data(self) -> bool:
        return self.channel_data_valid


@dataclass
class SonarLog:
    path: Path
    pings: List[PingRecord]
    packet_counts: Dict[int, int]
    metadata: Dict[str, object]
    error: Optional[str] = None

    @property
    def duration_s(self) -> float:
        times = [p.timestamp_ms for p in self.pings if p.timestamp_ms]
        return max(0.0, (max(times) - min(times)) / 1000.0) if len(times) >= 2 else 0.0

    @property
    def total_points(self) -> int:
        return sum(len(p.points) for p in self.pings)

    @property
    def first_time(self) -> str:
        return self.pings[0].timestamp if self.pings else "--"

    @property
    def last_time(self) -> str:
        return self.pings[-1].timestamp if self.pings else "--"


def iter_packets(data: mmap.mmap) -> Iterable[Tuple[int, int, int]]:
    """Yield (message_id, payload_offset, payload_length) from a Ping stream."""
    pos = 0
    size = len(data)
    while pos + 10 <= size:
        start = data.find(b"BR", pos)
        if start < 0 or start + 10 > size:
            return
        try:
            start_1, start_2, payload_len, message_id, _src, _dst = PACKET_HEADER.unpack_from(data, start)
        except struct.error:
            return
        if start_1 != 66 or start_2 != 82:
            pos = start + 2
            continue
        end = start + PACKET_HEADER.size + payload_len + 2
        if end > size:
            return
        yield message_id, start + PACKET_HEADER.size, payload_len
        pos = end


def parse_atof(payload: bytes, record: PingRecord) -> None:
    """Append angle/TOF detections from a Surveyor 240 ATOF message.

    The protocol stores the angle in radians.  Keeping radians internally
    avoids a hidden unit conversion in the requested y/z projection; the GUI
    converts to degrees only for labels.
    """
    if len(payload) < ATOF_HEADER.size:
        return
    try:
        _pwr, utc_ms, _listen, sos, ping_no, ping_hz, _pulse, _flags, n_points, _reserved = ATOF_HEADER.unpack_from(payload)
    except struct.error:
        return
    record.number = int(ping_no)
    record.sos_mps = float(sos or 1500.0)
    record.ping_hz = float(ping_hz or 0.0)
    if utc_ms:
        record.timestamp_ms = int(utc_ms)
    available = (len(payload) - ATOF_HEADER.size) // 16
    n_points = min(int(n_points), available)
    for index in range(n_points):
        offset = ATOF_HEADER.size + index * 16
        angle, tof, _reserved_a, _reserved_b = struct.unpack_from("<ffII", payload, offset)
        if not (-math.pi / 2.0 <= angle <= math.pi / 2.0) or not (0.0 <= tof <= 10.0):
            continue
        range_m = max(0.0, float(tof) * record.sos_mps / 2.0)
        record.points.append((float(angle), range_m))


def parse_channel_data_header(payload: bytes) -> Optional[Dict[str, Union[int, float]]]:
    """Read the confirmed SonarView 3009 ChPairGoertzelData header."""
    return common_parse_channel_data_header(payload)


def validate_channel_data(log: SonarLog, record: PingRecord) -> None:
    """Validate a complete 16-channel Surveyor ping without decoding samples."""
    record.channel_data_valid = False
    record.channel_data_note = ""
    if not record.raw_packet_offsets:
        record.channel_data_note = "no packet 3009"
        return
    headers = []
    try:
        with log.path.open("rb") as stream, mmap.mmap(stream.fileno(), 0, access=mmap.ACCESS_READ) as data:
            for payload_offset, payload_len in record.raw_packet_offsets:
                payload = data[payload_offset : payload_offset + payload_len]
                header = parse_channel_data_header(payload)
                if header is None:
                    record.channel_data_note = "invalid 3009 header"
                    return
                results = int(header["results_per_channel"])
                expected = CHANNEL_DATA_HEADER.size + 4 * results * 4
                if results <= 0 or payload_len < expected:
                    record.channel_data_note = f"short 3009 sample area ({payload_len} < {expected} bytes)"
                    return
                if not (0 <= int(header["ch1"]) < 16 and 0 <= int(header["ch2"]) < 16 and int(header["ch1"]) != int(header["ch2"])):
                    record.channel_data_note = "invalid channel indices"
                    return
                headers.append(header)
    except (OSError, ValueError):
        record.channel_data_note = "could not read packet 3009"
        return

    ping_numbers = {int(item["ping_number"]) for item in headers}
    channels = {int(item["ch1"]) for item in headers} | {int(item["ch2"]) for item in headers}
    pairs = {(int(item["ch1"]), int(item["ch2"])) for item in headers}
    results = {int(item["results_per_channel"]) for item in headers}
    if ping_numbers != {record.number}:
        record.channel_data_note = f"ping mismatch in 3009 ({sorted(ping_numbers)})"
    elif len(headers) != 8 or channels != set(range(16)) or len(pairs) != 8:
        record.channel_data_note = f"incomplete channel set ({len(headers)} packets, {len(channels)} channels)"
    elif len(results) != 1:
        record.channel_data_note = "inconsistent results_per_channel"
    else:
        record.channel_data_valid = True
        record.bins = next(iter(results))
        trailing = sum(max(0, length - CHANNEL_DATA_HEADER.size - 4 * record.bins * 4) for _, length in record.raw_packet_offsets)
        record.channel_data_note = f"16 channels · {record.bins} range steps · Float32 IQ · {trailing} trailing bytes ignored by SonarView schema"


def scan_svlog(path: Path) -> SonarLog:
    """Index a .svlog without retaining its raw profile payloads."""
    records: Dict[int, PingRecord] = {}
    counts: Dict[int, int] = {}
    metadata: Dict[str, object] = {}
    current_profile_packets: Dict[int, int] = {}
    try:
        with path.open("rb") as stream, mmap.mmap(stream.fileno(), 0, access=mmap.ACCESS_READ) as data:
            for message_id, payload_offset, payload_len in iter_packets(data):
                counts[message_id] = counts.get(message_id, 0) + 1
                payload = data[payload_offset : payload_offset + payload_len]
                if message_id == MSG_JSON:
                    try:
                        decoded = json.loads(bytes(payload).decode("utf-8", errors="replace"))
                        if isinstance(decoded, dict):
                            metadata.update(decoded)
                    except (UnicodeDecodeError, json.JSONDecodeError):
                        pass
                elif message_id == MSG_END_PING and payload_len == END_PING.size:
                    values = END_PING.unpack(payload)
                    ping_no = int(values[6])
                    record = records.setdefault(ping_no, PingRecord(number=ping_no))
                    record.range_start_m = float(values[1])
                    record.range_end_m = max(float(values[2]), record.range_start_m + 0.1)
                    # END_PING_INFO: index 16 is n_range_steps.  Index 17 is
                    # samples_per_range_bin and is not the image row count.
                    record.bins = int(values[16])
                    record.ping_hz = float(values[13] or 0.0)
                    record.timestamp_ms = int(values[-1] / 1_000_000) if values[-1] > 10_000_000_000_000 else int(values[-1])
                elif message_id == MSG_ATOF:
                    if payload_len < ATOF_HEADER.size:
                        continue
                    # ATOF layout: pwr_up_msec (4), utc_msec (8),
                    # listening_sec (4), sos_mps (4), then ping_number.
                    ping_no = int(struct.unpack_from("<I", payload, 20)[0])
                    record = records.setdefault(ping_no, PingRecord(number=ping_no))
                    parse_atof(bytes(payload), record)
                elif message_id == MSG_RAW_PROFILE:
                    # Raw profile packets precede the corresponding end_ping_info.
                    # The packet's first word is the ping number.
                    if payload_len >= 4:
                        ping_no = int(struct.unpack_from("<I", payload, 0)[0])
                        record = records.setdefault(ping_no, PingRecord(number=ping_no))
                        record.raw_packet_offsets.append((payload_offset, payload_len))
                        current_profile_packets[ping_no] = current_profile_packets.get(ping_no, 0) + 1
    except (OSError, ValueError) as exc:
        return SonarLog(path, [], counts, metadata, str(exc))

    for ping_no, count in current_profile_packets.items():
        records.setdefault(ping_no, PingRecord(number=ping_no)).raw_profile_packets = count
    pings = sorted(records.values(), key=lambda item: (item.timestamp_ms or 2**63, item.number))
    # Some logs have ATOF packets but no end_ping_info.  Keep them useful.
    result = SonarLog(path, pings, counts, metadata)
    for index, record in enumerate(pings):
        if not record.timestamp_ms:
            record.timestamp_ms = (pings[index - 1].timestamp_ms + 1 if index else 0)
        if record.range_end_m <= record.range_start_m:
            record.range_end_m = max(record.range_start_m + 0.1, record.range_max, 10.0)
        validate_channel_data(result, record)
    return result


def discover_logs(root: Path) -> List[Path]:
    if not root.exists():
        return []
    return sorted(root.rglob("*.svlog"), key=lambda p: p.stat().st_mtime, reverse=True)


def format_bytes(value: int) -> str:
    size = float(value)
    for unit in ("B", "KB", "MB", "GB"):
        if size < 1024 or unit == "GB":
            return f"{size:.1f} {unit}" if unit != "B" else f"{int(size)} B"
        size /= 1024.0
    return "--"


def format_duration(seconds: float) -> str:
    total = max(0, int(seconds))
    return f"{total // 60:02d}:{total % 60:02d}"


def heat_color(value: float, alpha: float = 1.0) -> Tuple[int, int, int]:
    """Blue -> cyan -> yellow -> white sonar palette."""
    x = max(0.0, min(1.0, value))
    stops = ((0.0, (3, 10, 35)), (0.25, (10, 57, 130)), (0.5, (0, 180, 210)), (0.75, (255, 190, 50)), (1.0, (255, 250, 220)))
    for (lo, c0), (hi, c1) in zip(stops, stops[1:]):
        if x <= hi:
            t = (x - lo) / (hi - lo)
            rgb = tuple(int(c0[i] + t * (c1[i] - c0[i])) for i in range(3))
            return tuple(int(c * alpha) for c in rgb)
    return stops[-1][1]


def cross_track_depth(record: PingRecord) -> List[Tuple[float, float, float]]:
    """Return (cross_track_y, depth_z, distance) in metres.

    Surveyor ATOF points contain angle in radians converted to degrees by the
    log parser and a time-of-flight value converted to range with the standard
    sonar equation: distance = 0.5 * sos * tof.  z is negative in the forward
    direction, matching the coordinate convention requested by the viewer.
    """
    result = []
    for angle_rad, distance in record.points:
        y = distance * math.sin(angle_rad)
        z = -distance * math.cos(angle_rad)
        result.append((y, z, distance))
    return result


def load_raw_intensity(
    log: SonarLog,
    record: PingRecord,
    threshold_percent: float = 0.0,
) -> Optional[Tuple[List[List[float]], float, float]]:
    """Beamform one validated ping using the shared Surveyor processor.

    ``threshold_percent=0`` preserves the full acoustic background.  The
    optional threshold is only a display filter and never changes the log.
    """
    if not record.channel_data_valid or not record.raw_packet_offsets:
        return None
    try:
        with log.path.open("rb") as stream, mmap.mmap(stream.fileno(), 0, access=mmap.ACCESS_READ) as data:
            payloads = [bytes(data[offset : offset + length]) for offset, length in record.raw_packet_offsets]
        valid, _note, channel_signals, bins = validate_channel_payloads(payloads, record.number)
        if not valid or bins != int(record.bins or bins):
            return None
        matrix = beamform_surveyor_channels(
            channel_signals,
            record.range_start_m,
            record.range_end_m,
            record.sos_mps,
            threshold_percent=threshold_percent,
        )
        return matrix, float(record.range_start_m), float(record.range_end_m)
    except (OSError, ValueError, struct.error):
        return None


class SonarLogViewer(tk.Tk):
    """Dark, responsive desktop viewer for indexed SonarView sessions."""

    BG = "#07111f"
    PANEL = "#0d1c2d"
    PANEL_2 = "#12263b"
    TEXT = "#e6f0f7"
    MUTED = "#93a9ba"
    ACCENT = "#43d5dc"
    ORANGE = "#ffbf57"

    def __init__(self, paths: List[Path]):
        super().__init__()
        self.title("BlueROV2 · SonarView log explorer")
        self.geometry("1420x900")
        self.minsize(1050, 680)
        self.configure(bg=self.BG)
        self.logs: List[SonarLog] = []
        self.selected_log: Optional[SonarLog] = None
        self.selected_ping = 0
        self.image_refs: List[ImageTk.PhotoImage] = []
        self.playing = False
        self.view_mode = tk.StringVar(value="ANGLE / DISTANCE")
        self._intensity_cache_key = None
        self._intensity_cache = None
        self._build_style()
        self._build_ui()
        self._load_paths(paths)

    def _build_style(self) -> None:
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure("TFrame", background=self.BG)
        style.configure("Panel.TFrame", background=self.PANEL)
        style.configure("TLabel", background=self.BG, foreground=self.TEXT, font=("Segoe UI", 10))
        style.configure("Title.TLabel", background=self.BG, foreground=self.TEXT, font=("Segoe UI", 21, "bold"))
        style.configure("Subtitle.TLabel", background=self.BG, foreground=self.MUTED, font=("Segoe UI", 10))
        style.configure("Panel.TLabel", background=self.PANEL, foreground=self.TEXT, font=("Segoe UI", 10))
        style.configure("PanelTitle.TLabel", background=self.PANEL, foreground=self.TEXT, font=("Segoe UI", 12, "bold"))
        style.configure("Accent.TButton", background=self.ACCENT, foreground="#001116", font=("Segoe UI", 10, "bold"), padding=(10, 6))
        style.map("Accent.TButton", background=[("active", "#7cf3ed")])
        style.configure("TButton", padding=(8, 5))
        style.configure("TScale", background=self.BG, troughcolor=self.PANEL_2)

    def _build_ui(self) -> None:
        header = ttk.Frame(self)
        header.pack(fill="x", padx=22, pady=(18, 10))
        ttk.Label(header, text="SONAR LOG EXPLORER", style="Title.TLabel").pack(anchor="w")
        ttk.Label(header, text="Esplora le registrazioni SonarView senza modificare i file originali", style="Subtitle.TLabel").pack(anchor="w", pady=(2, 0))

        toolbar = ttk.Frame(self)
        toolbar.pack(fill="x", padx=22, pady=(0, 12))
        ttk.Button(toolbar, text="Apri cartella", command=self._choose_folder).pack(side="left")
        ttk.Button(toolbar, text="Apri file .svlog", command=self._choose_file).pack(side="left", padx=(6, 0))
        ttk.Label(toolbar, text="Vista:", style="Subtitle.TLabel").pack(side="left", padx=(22, 6))
        self.view_selector = ttk.Combobox(
            toolbar,
            textvariable=self.view_mode,
            state="readonly",
            width=29,
            values=(
                "ANGLE / DISTANCE",
                "POLAR FAN",
                "CROSS-TRACK / DEPTH",
                "SURVEYOR FAN IMAGE",
            ),
        )
        self.view_selector.pack(side="left")
        self.view_selector.bind("<<ComboboxSelected>>", lambda _event: self._render())
        self.status = ttk.Label(toolbar, text="Indicizzazione…", style="Subtitle.TLabel")
        self.status.pack(side="right")

        body = ttk.Frame(self)
        body.pack(fill="both", expand=True, padx=22, pady=(0, 18))
        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)

        left = ttk.Frame(body, style="Panel.TFrame", padding=14)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 14))
        left.configure(width=330)
        left.grid_propagate(False)
        ttk.Label(left, text="USCITE REGISTRATE", style="PanelTitle.TLabel").pack(anchor="w")
        self.log_hint = ttk.Label(left, text="", style="Panel.TLabel", wraplength=290)
        self.log_hint.pack(anchor="w", pady=(4, 10))
        list_frame = ttk.Frame(left, style="Panel.TFrame")
        list_frame.pack(fill="both", expand=True)
        self.log_list = tk.Listbox(list_frame, bg="#0a1727", fg=self.TEXT, selectbackground="#1d5965", selectforeground="#ffffff", relief="flat", highlightthickness=0, activestyle="none", font=("Consolas", 9), width=36)
        scroll = ttk.Scrollbar(list_frame, orient="vertical", command=self.log_list.yview)
        self.log_list.configure(yscrollcommand=scroll.set)
        self.log_list.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")
        self.log_list.bind("<<ListboxSelect>>", self._on_log_select)

        right = ttk.Frame(body)
        right.grid(row=0, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)
        right.rowconfigure(2, weight=1)
        right.rowconfigure(3, weight=0)

        self.summary = ttk.Label(right, text="Seleziona una registrazione", style="Subtitle.TLabel")
        self.summary.grid(row=0, column=0, sticky="w", pady=(0, 8))
        self.cards = ttk.Frame(right)
        self.cards.grid(row=1, column=0, sticky="ew", pady=(0, 10))
        for i in range(5):
            self.cards.columnconfigure(i, weight=1)
        self.card_labels = []
        for title in ("PING", "ORARIO", "RANGE", "ECHI", "FREQUENZA", "CHANNEL DATA"):
            box = ttk.Frame(self.cards, style="Panel.TFrame", padding=(12, 8))
            box.grid(row=0, column=len(self.card_labels), sticky="ew", padx=(0 if not self.card_labels else 6, 0))
            ttk.Label(box, text=title, style="Panel.TLabel").pack(anchor="w")
            value = ttk.Label(box, text="--", style="PanelTitle.TLabel")
            value.pack(anchor="w", pady=(3, 0))
            self.card_labels.append(value)

        plots = ttk.Frame(right, style="Panel.TFrame", padding=10)
        plots.grid(row=2, column=0, sticky="nsew")
        plots.columnconfigure(0, weight=1)
        plots.rowconfigure(0, weight=1)
        plots.rowconfigure(1, weight=0)
        self.main_canvas = tk.Canvas(plots, bg="#030a14", highlightthickness=0)
        self.main_canvas.grid(row=0, column=0, sticky="nsew")
        self.main_canvas.bind("<Configure>", lambda _event: self._render())
        self.overview_canvas = tk.Canvas(plots, height=170, bg="#030a14", highlightthickness=0)
        self.overview_canvas.grid(row=1, column=0, sticky="ew", pady=(10, 0))
        self.overview_canvas.bind("<Button-1>", self._overview_click)

        controls = ttk.Frame(right)
        controls.grid(row=3, column=0, sticky="ew", pady=(10, 0))
        controls.columnconfigure(1, weight=1)
        self.play_button = ttk.Button(controls, text="▶ Riproduci", command=self._toggle_play)
        self.play_button.grid(row=0, column=0, padx=(0, 8))
        self.ping_scale = ttk.Scale(controls, from_=0, to=1, orient="horizontal", command=self._scale_changed)
        self.ping_scale.grid(row=0, column=1, sticky="ew")
        self.ping_label = ttk.Label(controls, text="0 / 0", style="Subtitle.TLabel")
        self.ping_label.grid(row=0, column=2, padx=(10, 0))

    def _load_paths(self, paths: List[Path]) -> None:
        def worker() -> None:
            result = []
            for path in paths:
                self.after(0, lambda p=path: self.status.configure(text=f"Indicizzo {p.name}…"))
                result.append(scan_svlog(path))
            self.after(0, lambda: self._finish_load(result))
        threading.Thread(target=worker, daemon=True).start()

    def _finish_load(self, logs: List[SonarLog]) -> None:
        self.logs = [log for log in logs if log.pings and not log.error]
        self.log_list.delete(0, "end")
        for log in self.logs:
            available = sum(1 for ping in log.pings if ping.has_channel_data)
            channel_text = "CHANNEL DATA: AVAILABLE" if available else "CHANNEL DATA: not available"
            text = f"{log.path.name}\n  {format_bytes(log.path.stat().st_size)} · {format_duration(log.duration_s)} · {len(log.pings):,} ping\n  {channel_text} ({available:,} ping)"
            self.log_list.insert("end", text)
        self.log_hint.configure(text=f"{len(self.logs)} registrazioni indicizzate\nSeleziona un file per vedere la scansione.")
        self.status.configure(text=f"Pronto · {sum(len(log.pings) for log in self.logs):,} ping indicizzati")
        if self.logs:
            self.log_list.selection_set(0)
            self._select_log(0)

    def _choose_folder(self) -> None:
        selected = filedialog.askdirectory(initialdir=str(DEFAULT_ROOT))
        if selected:
            self._load_paths(discover_logs(Path(selected)))

    def _choose_file(self) -> None:
        selected = filedialog.askopenfilename(initialdir=str(DEFAULT_ROOT), filetypes=[("SonarView log", "*.svlog"), ("Tutti i file", "*.*")])
        if selected:
            self._load_paths([Path(selected)])

    def _on_log_select(self, _event=None) -> None:
        selection = self.log_list.curselection()
        if selection:
            self._select_log(selection[0])

    def _select_log(self, index: int) -> None:
        if not self.logs:
            return
        self.selected_log = self.logs[index]
        self.selected_ping = 0
        self._intensity_cache_key = None
        self._intensity_cache = None
        maximum = max(0, len(self.selected_log.pings) - 1)
        self.ping_scale.configure(to=maximum)
        self.ping_scale.set(0)
        self._render()

    def _scale_changed(self, value: str) -> None:
        if self.selected_log:
            self.selected_ping = max(0, min(len(self.selected_log.pings) - 1, int(float(value))))
            self._render()

    def _toggle_play(self) -> None:
        self.playing = not self.playing
        self.play_button.configure(text="❚❚ Pausa" if self.playing else "▶ Riproduci")
        if self.playing:
            self._play_step()

    def _play_step(self) -> None:
        if not self.playing or not self.selected_log:
            return
        self.selected_ping = (self.selected_ping + 1) % len(self.selected_log.pings)
        self.ping_scale.set(self.selected_ping)
        self._render()
        self.after(45, self._play_step)

    def _overview_click(self, event) -> None:
        if not self.selected_log or not self.selected_log.pings:
            return
        width = max(1, self.overview_canvas.winfo_width())
        idx = int(max(0, min(len(self.selected_log.pings) - 1, event.x / width * len(self.selected_log.pings))))
        self.selected_ping = idx
        self.ping_scale.set(idx)
        self._render()

    def _render(self) -> None:
        if not self.selected_log or not self.selected_log.pings:
            return
        record = self.selected_log.pings[self.selected_ping]
        self.ping_label.configure(text=f"{self.selected_ping + 1:,} / {len(self.selected_log.pings):,}")
        self.summary.configure(text=f"{self.selected_log.path.name}   ·   {format_bytes(self.selected_log.path.stat().st_size)}   ·   {format_duration(self.selected_log.duration_s)}")
        values = [
            f"{record.number:,}", record.timestamp,
            f"{record.range_start_m:.1f}–{record.range_end_m:.1f} m",
            f"{len(record.points)} punti", f"{record.ping_hz / 1000:.0f} kHz" if record.ping_hz else "--",
            "AVAILABLE" if record.has_channel_data else "not available",
        ]
        for label, value in zip(self.card_labels, values):
            label.configure(text=value)
        if self.view_mode.get() == "POLAR FAN":
            self._draw_polar_fan(record)
        elif self.view_mode.get() == "CROSS-TRACK / DEPTH":
            self._draw_cross_track_depth(record)
        elif self.view_mode.get() == "SURVEYOR FAN IMAGE":
            self._draw_intensity_fan(record)
        else:
            self._draw_angle_distance(record)
        self._draw_overview()

    def _draw_angle_distance(self, record: PingRecord) -> None:
        canvas = self.main_canvas
        width = max(500, canvas.winfo_width())
        height = max(300, canvas.winfo_height())
        image = Image.new("RGB", (width, height), "#030a14")
        draw = ImageDraw.Draw(image)
        left, top, right, bottom = 68, 24, width - 24, height - 42
        max_range = max(1.0, record.range_end_m, record.range_max)
        angle_min, angle_max = -40.0, 40.0
        # Soft sonar glow around each detected echo.
        for angle_rad, distance in record.points:
            angle = math.degrees(angle_rad)
            x = left + (angle - angle_min) / (angle_max - angle_min) * (right - left)
            y = top + (distance / max_range) * (bottom - top)
            strength = max(0.2, 1.0 - distance / max_range)
            for radius in range(12, 1, -2):
                col = heat_color(strength * (1.0 - radius / 15.0), 0.12 + (12 - radius) / 35.0)
                draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill=col)
            draw.ellipse((x - 3, y - 3, x + 3, y + 3), fill=heat_color(strength))
        # Grid and labels.
        for angle in range(-40, 41, 10):
            x = left + (angle - angle_min) / (angle_max - angle_min) * (right - left)
            draw.line((x, top, x, bottom), fill="#17354b", width=1)
            draw.text((x - 12, bottom + 10), f"{angle}°", fill="#8aa5b8")
        step = 1.0 if max_range <= 12 else 2.0
        distance = 0.0
        while distance <= max_range + 0.01:
            y = top + (distance / max_range) * (bottom - top)
            draw.line((left, y, right, y), fill="#17354b", width=1)
            draw.text((8, y - 7), f"{distance:.0f} m", fill="#8aa5b8")
            distance += step
        draw.text((left, 5), "ECHI RILEVATI · ANGOLO / DISTANZA", fill="#c8e9f1")
        draw.text((right - 140, 5), f"{len(record.points)} detections", fill="#ffcf72")
        # Draw the selected ping's echo profile as a ground line.
        if record.points:
            ordered = sorted(record.points)
            line = []
            for angle_rad, distance in ordered:
                angle = math.degrees(angle_rad)
                x = left + (angle - angle_min) / (angle_max - angle_min) * (right - left)
                y = top + (distance / max_range) * (bottom - top)
                line.append((x, y))
            if len(line) > 1:
                draw.line(line, fill="#ffd36d", width=1)
        self._show_image(canvas, image)

    @staticmethod
    def _fan_xy(cx: float, cy: float, radius: float, angle_rad: float, distance: float, max_range: float) -> Tuple[float, float]:
        scale = max(0.0, min(1.0, distance / max(1e-6, max_range)))
        return cx + radius * scale * math.sin(angle_rad), cy - radius * scale * math.cos(angle_rad)

    def _draw_fan_grid(self, draw: ImageDraw.ImageDraw, width: int, height: int, max_range: float, fill_sector: bool = True) -> Tuple[float, float, float]:
        cx, cy = width / 2.0, height - 36.0
        radius = min(width * 0.46, max(80.0, height - 68.0))
        sector = [
            (cx, cy),
            *[
                self._fan_xy(cx, cy, radius, math.radians(angle), max_range, max_range)
                for angle in range(-40, 41, 2)
            ],
            (cx, cy),
        ]
        draw.polygon(sector, fill="#06192a" if fill_sector else None, outline="#2b536c")
        for distance in (max_range * 0.25, max_range * 0.5, max_range * 0.75, max_range):
            arc = [
                self._fan_xy(cx, cy, radius, math.radians(angle), distance, max_range)
                for angle in range(-40, 41, 2)
            ]
            draw.line(arc, fill="#23485f", width=1)
            if distance < max_range:
                x, y = self._fan_xy(cx, cy, radius, math.radians(38), distance, max_range)
                draw.text((x + 4, y - 8), f"{distance:.1f} m", fill="#8aa5b8")
        for angle in (-40, -20, 0, 20, 40):
            x, y = self._fan_xy(cx, cy, radius, math.radians(angle), max_range, max_range)
            draw.line((cx, cy, x, y), fill="#2b536c", width=1)
            draw.text((x - 14, y - 18), f"{angle}°", fill="#8aa5b8")
        return cx, cy, radius

    def _draw_polar_fan(self, record: PingRecord) -> None:
        """Render detections in the familiar forward-looking sonar fan."""
        canvas = self.main_canvas
        width = max(500, canvas.winfo_width())
        height = max(300, canvas.winfo_height())
        image = Image.new("RGB", (width, height), "#030a14")
        draw = ImageDraw.Draw(image)
        max_range = max(1.0, record.range_end_m, record.range_max)
        cx, cy, radius = self._draw_fan_grid(draw, width, height, max_range)
        for angle_rad, distance in record.points:
            x, y = self._fan_xy(cx, cy, radius, angle_rad, distance, max_range)
            strength = max(0.25, 1.0 - distance / max_range)
            for glow in range(12, 1, -2):
                color = heat_color(strength * (1.0 - glow / 15.0), 0.15 + (12 - glow) / 34.0)
                draw.ellipse((x - glow, y - glow, x + glow, y + glow), fill=color)
            draw.ellipse((x - 3, y - 3, x + 3, y + 3), fill=heat_color(strength))
        draw.text((18, 12), "POLAR FAN · RITORNI DEL PING SELEZIONATO", fill="#c8e9f1")
        draw.text((width - 180, 12), f"{len(record.points)} detections", fill="#ffcf72")
        self._show_image(canvas, image)

    def _draw_cross_track_depth(self, record: PingRecord) -> None:
        """Render y/z metric coordinates and a probable far-field envelope."""
        canvas = self.main_canvas
        width = max(500, canvas.winfo_width())
        height = max(300, canvas.winfo_height())
        image = Image.new("RGB", (width, height), "#030a14")
        draw = ImageDraw.Draw(image)
        points = cross_track_depth(record)
        max_range = max(1.0, record.range_end_m, record.range_max)
        left, top, right, bottom = 74, 28, width - 26, height - 52

        def project(y: float, z: float) -> Tuple[float, float]:
            px = left + (y + max_range) / (2.0 * max_range) * (right - left)
            pz = top + (-z) / max_range * (bottom - top)
            return px, pz

        for y in range(-int(max_range), int(max_range) + 1):
            px, _ = project(float(y), 0.0)
            draw.line((px, top, px, bottom), fill="#17354b", width=1)
            if y % 2 == 0:
                draw.text((px - 12, bottom + 10), f"{y} m", fill="#8aa5b8")
        step = 1.0 if max_range <= 12 else 2.0
        depth = 0.0
        while depth <= max_range + 0.01:
            _, py = project(0.0, -depth)
            draw.line((left, py, right, py), fill="#17354b", width=1)
            draw.text((8, py - 7), f"{-depth:.0f} m", fill="#8aa5b8")
            depth += step

        for y, z, distance in points:
            x, py = project(y, z)
            strength = max(0.2, 1.0 - distance / max_range)
            for glow in range(10, 1, -2):
                color = heat_color(strength * (1.0 - glow / 13.0), 0.14 + (10 - glow) / 30.0)
                draw.ellipse((x - glow, py - glow, x + glow, py + glow), fill=color)
            draw.ellipse((x - 3, py - 3, x + 3, py + 3), fill=heat_color(strength))

        # The deepest return in each cross-track bin is a useful visual proxy
        # for the probable bottom, but is not a bathymetric reconstruction.
        envelope = []
        bins = 80
        for index in range(bins):
            lo = -max_range + (2 * max_range) * index / bins
            hi = -max_range + (2 * max_range) * (index + 1) / bins
            candidates = [point for point in points if lo <= point[0] < hi]
            if candidates:
                y, z, _distance = min(candidates, key=lambda point: point[1])
                envelope.append(project(y, z))
        if len(envelope) > 1:
            draw.line(envelope, fill="#ffcf72", width=2)
        draw.text((left, 7), "CROSS-TRACK / DEPTH · COORDINATE METRICHE", fill="#c8e9f1")
        draw.text((right - 305, 7), "linea gialla = fondale probabile", fill="#ffcf72")
        draw.text((left + (right - left) / 2 - 55, height - 25), "y cross-track [m]", fill="#8aa5b8")
        draw.text((8, top - 18), "z [m]", fill="#8aa5b8")
        self._show_image(canvas, image)

    def _draw_intensity_fan(self, record: PingRecord) -> None:
        """Render the selected 16-channel ping as a beamformed fan image."""
        canvas = self.main_canvas
        width = max(500, canvas.winfo_width())
        height = max(300, canvas.winfo_height())
        image = Image.new("RGB", (width, height), "#030a14")
        draw = ImageDraw.Draw(image)
        if not record.has_raw_intensity:
            draw.text((40, height // 2 - 12), "CHANNEL DATA non disponibile o non validato per questo ping.", fill="#ffcf72")
            draw.text((40, height // 2 + 14), "Usa POLAR FAN per visualizzare le detection ATOF.", fill="#a8c2d0")
            self._show_image(canvas, image)
            return
        key = (str(self.selected_log.path), record.number)
        if self._intensity_cache_key != key:
            self._intensity_cache = load_raw_intensity(self.selected_log, record)
            self._intensity_cache_key = key
        decoded = self._intensity_cache
        if not decoded:
            draw.text((40, height // 2 - 12), "CHANNEL DATA presente ma il ping non è decodificabile.", fill="#ffcf72")
            self._show_image(canvas, image)
            return
        matrix, range_start, range_end = decoded
        values = sorted(value for row in matrix for value in row if math.isfinite(value) and value > 0.0)
        if not values:
            draw.text((40, height // 2), "Nessuna potenza valida nel ping selezionato.", fill="#ffcf72")
            self._show_image(canvas, image)
            return
        # Relative robust normalization keeps the fan legible without
        # pretending that the log contains an absolute calibrated dB image.
        db_values = [10.0 * math.log10(value) for value in values]
        lo = db_values[int(len(db_values) * 0.05)]
        hi = db_values[int(len(db_values) * 0.995)]
        if hi <= lo:
            hi = lo + 1.0
        cx, cy = width / 2.0, height - 36.0
        radius = min(width * 0.46, max(80.0, height - 68.0))
        beam_count = len(matrix)
        range_count = max((len(row) for row in matrix), default=0)
        for beam_index, row in enumerate(matrix):
            a0 = math.radians(-40.0 + 80.0 * beam_index / max(1, beam_count))
            a1 = math.radians(-40.0 + 80.0 * (beam_index + 1) / max(1, beam_count))
            for range_index, value in enumerate(row):
                r0 = range_start + (range_end - range_start) * range_index / max(1, range_count)
                r1 = range_start + (range_end - range_start) * (range_index + 1) / max(1, range_count)
                db = 10.0 * math.log10(value) if value > 0.0 and math.isfinite(value) else lo
                strength = max(0.0, min(1.0, (db - lo) / (hi - lo)))
                p0 = self._fan_xy(cx, cy, radius, a0, r0, max(range_end, 1.0))
                p1 = self._fan_xy(cx, cy, radius, a1, r0, max(range_end, 1.0))
                p2 = self._fan_xy(cx, cy, radius, a1, r1, max(range_end, 1.0))
                p3 = self._fan_xy(cx, cy, radius, a0, r1, max(range_end, 1.0))
                draw.polygon((p0, p1, p2, p3), fill=heat_color(strength))
        # Optional ATOF detections are overlaid in white/yellow on the
        # beamformed image; they come from a different protocol message.
        for angle_rad, distance in record.points:
            if -math.radians(40) <= angle_rad <= math.radians(40):
                x, y = self._fan_xy(cx, cy, radius, angle_rad, distance, max(range_end, 1.0))
                draw.ellipse((x - 3, y - 3, x + 3, y + 3), outline="#ffffff", width=2)
        self._draw_fan_grid(draw, width, height, max(range_end, 1.0), fill_sector=False)
        draw.text((18, 12), "SURVEYOR FAN IMAGE · BEAMFORMED CHANNEL IQ", fill="#c8e9f1")
        draw.text((width - 340, 12), f"{len(record.raw_packet_offsets)} packet · {beam_count} beam × {range_count} range", fill="#ffcf72")
        self._show_image(canvas, image)

    def _draw_overview(self) -> None:
        canvas = self.overview_canvas
        width = max(500, canvas.winfo_width())
        height = max(120, canvas.winfo_height())
        image = Image.new("RGB", (width, height), "#030a14")
        draw = ImageDraw.Draw(image)
        if not self.selected_log:
            self._show_image(canvas, image)
            return
        pings = self.selected_log.pings
        max_range = max(1.0, max((p.range_end_m for p in pings), default=10.0), max((p.range_max for p in pings), default=10.0))
        left, top, right, bottom = 45, 22, width - 12, height - 28
        draw.text((8, 4), "PANORAMICA TEMPORALE · ogni colonna = un ping · colore = angolo", fill="#c8e9f1")
        for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
            y = top + fraction * (bottom - top)
            draw.line((left, y, right, y), fill="#17354b", width=1)
            draw.text((8, y - 6), f"{max_range * fraction:.0f} m", fill="#8aa5b8")
        for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
            x = left + fraction * (right - left)
            draw.line((x, top, x, bottom), fill="#102b40", width=1)
        draw.text((left, bottom + 8), "inizio", fill="#8aa5b8")
        draw.text((right - 32, bottom + 8), "fine", fill="#8aa5b8")
        for index, record in enumerate(pings):
            x = left + (index / max(1, len(pings) - 1)) * (right - left)
            for angle_rad, distance in record.points:
                y = top + (distance / max_range) * (bottom - top)
                angle_strength = (math.degrees(angle_rad) + 40.0) / 80.0
                draw.ellipse((x - 1, y - 1, x + 1, y + 1), fill=heat_color(0.25 + 0.75 * angle_strength))
        draw.rectangle((left, top, right, bottom), outline="#315167")
        selected_x = left + (self.selected_ping / max(1, len(pings) - 1)) * (right - left)
        draw.line((selected_x, top, selected_x, bottom), fill="#ffffff", width=2)
        selected = self.selected_log.pings[self.selected_ping]
        selected_y = top + (selected.range_max / max_range) * (bottom - top)
        draw.line((left, selected_y, right, selected_y), fill="#ffcf72", width=1)
        draw.text((right - 130, selected_y - 14), "range ping selezionato", fill="#ffcf72")
        draw.text((right - 190, 4), "blu = -40° · giallo = +40°", fill="#ffcf72")
        self._show_image(canvas, image)

    def _show_image(self, canvas: tk.Canvas, image: Image.Image) -> None:
        photo = ImageTk.PhotoImage(image)
        self.image_refs.append(photo)
        self.image_refs = self.image_refs[-4:]
        canvas.delete("all")
        canvas.create_image(0, 0, image=photo, anchor="nw")


def print_summary(paths: List[Path]) -> None:
    for path in paths:
        log = scan_svlog(path)
        if log.error:
            print(f"{path.name}: ERROR {log.error}")
            continue
        channel_pings = sum(1 for ping in log.pings if ping.has_channel_data)
        channel_status = "CHANNEL DATA: AVAILABLE" if channel_pings else "CHANNEL DATA: not available"
        print(f"{path.name}: {len(log.pings)} ping, {log.total_points} punti, {format_duration(log.duration_s)}, {log.first_time} -> {log.last_time}, {channel_status} ({channel_pings})")


def main() -> int:
    parser = argparse.ArgumentParser(description="Visualizzatore offline dei log SonarView")
    parser.add_argument("--folder", type=Path, default=None, help="cartella da indicizzare")
    parser.add_argument("--file", type=Path, action="append", default=[], help="file .svlog da aprire; ripetibile")
    parser.add_argument("--summary", action="store_true", help="stampa un riepilogo e non apre la GUI")
    args = parser.parse_args()
    if args.file:
        paths = [p for p in args.file if p.exists()]
    else:
        root = args.folder or DEFAULT_ROOT
        paths = discover_logs(root)
    if not paths:
        print("Nessun file .svlog trovato. Usa --folder o --file.")
        return 1
    if args.summary:
        print_summary(paths)
        return 0
    app = SonarLogViewer(paths)
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
