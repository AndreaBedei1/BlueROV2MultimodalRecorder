# Data format

_Field-level description of files written by `SessionRecorder`._

---

## 📋 Session directory

Each press of **START SESSION** creates a unique directory:

```text
records/real_sessions/<session_id>/
```

`session_id` is the local timestamp followed by a short UUID suffix. The recorder opens the JSONL/CSV files immediately; `camera_rgb.mkv` and `surveyor_raw.svlog` are created when their corresponding stream paths are active. A skipped Surveyor intentionally does not create a fake raw `.svlog` file.

## 🧾 `session.json`

`session.json` is rewritten when camera metadata changes and on close. The fields written by the current code are:

| Field | Meaning |
| --- | --- |
| `session_id` | Unique session identifier |
| `session_start_utc_ns` | Host UTC start time in nanoseconds |
| `session_start_monotonic_ns` | Host monotonic start time in nanoseconds |
| `session_start_utc` | ISO-8601 rendering of the UTC start time |
| `session_end_utc_ns` | Host UTC close time, present after close |
| `session_end_monotonic_ns` | Host monotonic close time, present after close |
| `closed` | `true` after a clean recorder close |
| `camera` | Camera source, backend, timing, dimensions, and remux metadata |
| `surveyor` | Surveyor host/port, mode, replay source, and TX state |
| `ping1d` | Ping1D host and port |

The `camera` object includes `port`, `source`, `backend`, `recording_mode`, `input_status`, `remux_status`, `codec`, `width`, `height`, `nominal_fps`, `video_pts`, and `time_base` with `num`/`den`. `recording_reason` may be present when the fallback path explains a failure.

The `surveyor.mode` value is `live`, `replay`, or `skipped`; `surveyor.tx` is initialized as `LOCKED`.

## 📝 `events.jsonl`

One JSON object is written for each GUI/worker event that reaches the session:

```json
{
  "timestamp": "2026-09-14T12:34:56.789+00:00",
  "session_id": "20260914_123456_ab12cd34",
  "session_start_utc_ns": 0,
  "session_start_monotonic_ns": 0,
  "kind": "camera_connected",
  "data": {}
}
```

The exact `data` value depends on `kind`; it may be a worker status object, a message, or `null`. `surveyor_ping` and `ping_sample` events are logged with `data: null` because their full records are written to their dedicated JSONL files.

## 🎥 `camera_rgb.mkv`

When PyAV can open the input and create the Matroska output, encoded camera packets are remuxed without intentional re-encoding. The source stream template, codec, dimensions, nominal rate, and media time base are reflected in `session.json` and `camera_timestamps.csv`.

If PyAV remux fails and OpenCV is available, the fallback writes decoded frames using an OpenCV video writer. In that mode the file is a convenience recording, not a claim that the original encoded packet stream was preserved.

## ⏱️ `camera_timestamps.csv`

The header is written by the recorder as:

```text
session_id,session_start_utc_ns,session_start_monotonic_ns,frame_index,packet_index,pts,dts,time_base_num,time_base_den,pts_seconds,dts_seconds,host_monotonic_ns,host_utc_ns,session_time_s,key_frame,packet_size
```

There is one row for each decoded camera frame delivered to the session. `frame_index` and `packet_index` are zero-based worker counters when available. `pts`, `dts`, `time_base_num`, `time_base_den`, `pts_seconds`, and `dts_seconds` remain empty when the input backend does not provide them. `session_time_s` is computed from host monotonic time:

```text
(host_monotonic_ns - session_start_monotonic_ns) / 1e9
```

`key_frame` and `packet_size` are populated when the backend exposes them.

## 📡 `surveyor_raw.svlog`

This binary file contains a Ping Protocol packet stream. At session creation the recorder writes one recorder metadata packet with message id `10`, followed by the raw packet bytes received from the live Surveyor or replayed from the source file. The received packet bytes are copied without replacing their payload with JSON or a point-only representation.

The application parser recognizes these message identifiers:

| Message | Meaning used by the recorder |
| ---: | --- |
| `10` | Session metadata packet written by this recorder |
| `504` | Optional Surveyor attitude/up-vector data |
| `3009` | Channel-pair IQ data used for validation and beamforming |
| `3010` | End-ping metadata, range, ping number, and timing fields |
| `3012` | ATOF angle/time-of-flight detections |

Unknown packets remain in the binary stream and are not assigned an invented schema.

## 📊 `surveyor_pings.jsonl`

Each line is the processed record assembled at message `3010`. The current record fields are:

| Field | Meaning |
| --- | --- |
| `ping_number` | Surveyor ping number |
| `host_monotonic_ns` | Host monotonic ingest timestamp |
| `host_utc_ns` | Host UTC ingest timestamp |
| `device_timestamp_ns` | Normalized device timestamp when available |
| `range_start_m` / `range_end_m` | Ping range in metres |
| `sos_mps` | Speed of sound used by the Surveyor |
| `ping_rate_hz` | Reported ping rate |
| `points` | ATOF detection objects |
| `detection_count` | Number of decoded ATOF points |
| `channel_data_status` | `AVAILABLE`, `NOT AVAILABLE`, or `INVALID` |
| `bins` | Validated channel-data range-step count, when available |
| `channel_data_note` | Validation/decoder note, when available |
| `attitude` | Decoded message-`504` fields, or `null` |

Each ATOF point currently contains `angle_rad`, `tof_s`, `distance_m`, `reserved_a`, and `reserved_b`. Attitude currently contains the decoded up-vector fields and device timing fields exposed by the decoder.

The full IQ channel arrays and beamformed matrix are intentionally omitted from this JSONL convenience record by `serializable_surveyor_record()`. The source channel packets remain in `surveyor_raw.svlog`, and the GUI can beamform them in memory.

## 📏 `ping1d.jsonl`

Each line contains the two records emitted by the Ping1D worker plus session identity:

```json
{
  "distance": {
    "timestamp": "...",
    "host_monotonic_ns": 0,
    "host_utc_ns": 0,
    "distance_mm": 0,
    "distance_m": 0.0,
    "confidence": 0
  },
  "profile": {
    "timestamp": "...",
    "host_monotonic_ns": 0,
    "host_utc_ns": 0,
    "distance_mm": 0,
    "distance_m": 0.0,
    "confidence": 0,
    "scan_start_mm": 0,
    "scan_length_mm": 0,
    "gain": 0,
    "profile": [],
    "display_row": []
  },
  "session_id": "...",
  "session_start_utc_ns": 0,
  "session_start_monotonic_ns": 0
}
```

`profile` is `null` when only a distance response is available. The normalized `profile` list preserves the complete response-strength vector returned by the official Ping1D profile message; `display_row` is a GUI normalization and is not a second sensor measurement.

## 🔗 Timestamp rules

Host monotonic time is the matching clock. Host UTC time provides external correlation. Device timestamps, camera PTS/DTS, and media time bases are retained as separate fields and are not silently converted into one another.

