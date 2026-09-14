# Architecture

_Runtime architecture for live visualization, synchronized recording, and offline replay._

---

## 📋 System boundary

The application is a Windows desktop process with one worker per external stream and a shared event queue. The GUI consumes events for display; the session recorder consumes the same logical samples for durable output. The Surveyor processing module contains protocol parsing and fan beamforming but no device-control code.

```mermaid
flowchart TB
    accTitle: Recorder Runtime Data Flow
    accDescr: Three real sensor workers feed a shared event queue, while the GUI visualizes events and the session recorder writes raw and processed files using host timestamps.

    surveyor_worker[Surveyor worker]
    ping1d_worker[Ping1D worker]
    camera_worker[Camera worker]
    blueos_worker[BlueOS status worker]
    event_queue[(Event queue)]
    gui[Live visualization]
    recorder[Session recorder]
    raw_files[(Raw packet files)]
    processed_files[(Processed records)]

    surveyor_worker --> event_queue
    ping1d_worker --> event_queue
    camera_worker --> event_queue
    blueos_worker --> event_queue
    event_queue --> gui
    event_queue --> recorder
    surveyor_worker --> raw_files
    camera_worker --> raw_files
    recorder --> processed_files

    classDef input fill:#dbeafe,stroke:#2563eb,stroke-width:2px,color:#1e3a5f
    classDef process fill:#ede9fe,stroke:#7c3aed,stroke-width:2px,color:#3b0764
    classDef data fill:#dcfce7,stroke:#16a34a,stroke-width:2px,color:#14532d

    class surveyor_worker,ping1d_worker,camera_worker,blueos_worker input
    class event_queue,gui,recorder process
    class raw_files,processed_files data
```

## 👁️ Live visualization

The GUI shows the current camera frame, Surveyor fan image/ATOF information, Ping1D distance/profile, connection badges, and Surveyor TX state. Live visualization is a transient view: it may drop frames or close when the process exits, and it is not the authoritative dataset.

## 💾 Session recording

`START SESSION` creates a unique directory and opens the JSONL, CSV, and raw packet writers. The session can be started before or after `Connect all`; the camera callback is attached and detached under the worker/session lifecycle rules. `STOP SESSION` flushes and closes the files and adds session-end timestamps to `session.json`.

The camera worker uses a single ingest path. With PyAV, encoded packets are offered to the session remuxer and decoded frames are sent to the GUI. With the fallback path, decoded frames are written through OpenCV when available.

## 🧱 Raw and processed data

| Layer | Examples | Purpose |
| --- | --- | --- |
| Raw | `surveyor_raw.svlog`, encoded camera packets in `camera_rgb.mkv` | Preserve source bytes and native video timing where available |
| Processed | `surveyor_pings.jsonl`, `ping1d.jsonl`, `events.jsonl`, `camera_timestamps.csv` | Make decoded values, event history, and synchronization metadata easy to consume |
| Session metadata | `session.json` | Describe configuration, modes, camera backend, and lifecycle timestamps |

The application does not replace the raw Surveyor packets with a simplified point list. It writes the convenience records in parallel.

## ⏱️ Synchronization

Each stream sample receives a host `monotonic_ns` timestamp and a host `utc_ns` timestamp at ingest. Monotonic time is used for nearest-sample matching because it is not affected by wall-clock adjustments. UTC is retained for human correlation and cross-machine bookkeeping.

Camera `pts`, `dts`, and time-base fields are kept separately from the host clocks. A container PTS is a media-clock value; it is not silently treated as UTC. `match_surveyor_ping()` finds the nearest camera frame and Ping1D sample by host monotonic time and reports signed deltas in milliseconds.

## 🔄 Replay boundary

`--replay-surveyor FILE.svlog` creates a passive replay worker. It reads Ping Protocol frames from the file, rebuilds ATOF/channel/attitude/end-ping records, and emits the same logical Surveyor events used by the live worker. It does not instantiate `Surveyor240`, open the hardware TCP endpoint, or send a ping-enable command.

## 🧩 Extension point

Vehicle telemetry is deliberately not faked. A future read-only worker can emit `vehicle_telemetry.jsonl` events with a stable session identity and host timestamps. Existing files and readers should remain valid when that optional stream is added.

