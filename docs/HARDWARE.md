# Hardware and network setup

_Hardware boundary for the BlueROV2 Multimodal Recorder._

---

## 📋 System summary

The recorder is designed around a real BlueROV2 platform with three sensing inputs: a Cerulean Surveyor 240-16, a Blue Robotics Ping1D, and an RGB camera stream. The addresses below are the configuration used by this project, not universal defaults for every vehicle.

## 🚢 BlueROV2

BlueROV2 is the underwater ROV that carries the sensing payload. The platform combines an onboard computer running BlueOS with a flight controller/autopilot, vehicle sensors, thrusters, and a tethered Ethernet network. BlueOS provides vehicle software and services, while the autopilot/ArduSub layer handles vehicle stabilization and translates approved control inputs into thruster outputs.[^1]

The platform can expose vehicle state through MAVLink-compatible telemetry. This recorder currently treats BlueOS as network context and does not implement a vehicle-control path or a verified telemetry logger. MAVLink capture is reserved for the future pose stream described in [`RECONSTRUCTION_3D.md`](RECONSTRUCTION_3D.md).

The recorder may observe the following platform elements without controlling them:

- **RGB camera:** H.264/RTP video delivered over the vehicle network
- **Autopilot/flight controller:** vehicle state and control subsystem
- **Thrusters:** the propulsion and maneuvering actuators managed by the vehicle stack
- **BlueOS:** onboard software and service layer
- **Ethernet network:** the transport used by the computer, camera, sonars, and BlueOS services

The exact BlueROV2 revision, payload mounting, camera model, and vehicle telemetry availability are configuration-specific and are not inferred by this repository.

## 📡 Cerulean Surveyor 240-16

The Surveyor 240-16 is the multibeam echosounder used by this project. Cerulean describes it as a 240 kHz multibeam echo sounder, and the Python interface exposes the `Surveyor240` device class.[^2][^3]

The application handles these Surveyor concepts:

| Concept | Meaning in this project |
| --- | --- |
| Raw channel data | Original message-`3009` channel-pair packets containing the IQ sample area used for the reconstructed fan |
| ATOF detections | Sparse angle/time-of-flight points from message `3012` |
| Fan image | Relative angle/range intensity visualization beamformed from a validated 16-channel set |
| Range | Start and end range carried by the Surveyor ping metadata |
| Angle | ATOF angle in radians; the GUI displays the Surveyor sector of approximately `-40°` to `+40°` |
| Ping | One numbered Surveyor acquisition assembled from related protocol messages |
| Attitude | Optional message-`504` up-vector information when present |

Both the raw packet stream and the processed records are retained. Raw packets preserve future decoding options and provenance; JSONL records make common inspection and synchronization tasks convenient without pretending to replace the source bytes.

Project configuration:

```text
TCP 192.168.2.86:62312
```

The default recorder state is `DRY / TX LOCKED`. Normal startup does not call the Surveyor ping-parameter command with `ping_enable=True`. The only application path that can request acoustic start is guarded by `--wet-authorized`; replay never creates a Surveyor device.

## 📏 Blue Robotics Ping1D

Ping1D is the single-beam sonar/rangefinder stream used alongside the Surveyor. The Ping Protocol defines a distance measurement with confidence and a `profile` message containing response-strength samples over a scan interval.[^4]

The recorder stores:

- `distance_mm` and `distance_m`
- `confidence`
- `scan_start_mm` and `scan_length_mm` when a profile is available
- `gain`
- the complete normalized `profile` sample list
- a display-only `display_row`

Conceptually, Ping1D provides a one-dimensional range profile along one beam. The Surveyor instead provides multiple receive channels and angle/time-of-flight detections, with optional channel IQ that the application can beamform into an angle/range fan.

Project configuration through BlueOS PingProxy:

```text
UDP 192.168.2.2:9090
```

## 🎥 RGB camera

The RGB camera is the BlueROV2 video stream used for live context and synchronized recording. The configured receive ports are:

| Use | Value |
| --- | --- |
| Default UDP port | `5600` |
| Alternative UDP port | `5602` |
| Optional source | Local SDP path or SDP-capable URL |

When PyAV can open the source, one ingest loop demuxes encoded packets for MKV remux and decodes frames for the GUI. OpenCV is used only as the fallback path. The camera module does not change BlueOS stream configuration.

## 🔌 ROV Locator Mk III

The optional Cerulean ROV Locator Mk III topside transceiver normally appears on Windows as a USB COM port. Cerulean specifies `115200` baud, eight data bits, no parity, and one stop bit for receiver/transceiver USB communication.[^5] Its output uses ASCII packets compatible with NMEA-0183 sentences, including `$USRTH`, `$USINF`, `$USERR`, `$USDEB`, and forwarded GNSS sentences.[^6]

The application can select a COM port manually or use **Auto**. Auto mode briefly listens to enumerated ports and accepts a device only after observing a Cerulean `US` message. It does not infer or hard-code vendor/product IDs. Probing and acquisition are strictly read-only: the code sends no configuration, calibration, reset, baud-rate, acoustic, or passthrough command.

ROVL absence is non-fatal. **Connect all** continues to start the other inputs even when no compatible COM stream is found. Disconnect closes the serial handle.

## 🔐 Safety boundary

The recorder is an acquisition tool, not a vehicle controller. It does not arm, move, steer, or configure the ROV. Keep the Surveyor in the default locked mode unless an appropriately approved in-water procedure explicitly authorizes transmission.

## 🔗 References

[^1]: Blue Robotics. “BlueROV2 Software, Network, and Joystick Setup Instructions.” https://bluerobotics.com/learn/bluerov2-software-setup/
[^2]: Cerulean Sonar. “Surveyor 240-16 MBES.” https://ceruleansonar.com/product/surveyor-240-16/
[^3]: Cerulean Sonar. “Getting Started With Ping-Python.” https://docs.ceruleansonar.com/c/surveyor-240-16/getting-started-with-ping-python
[^4]: Blue Robotics. “Ping1D messages.” https://docs.bluerobotics.com/ping-protocol/pingmessage-ping1d/
[^5]: Cerulean Sonar. (2025). “Serial Parameters.” https://docs.ceruleansonar.com/c/rov-locator/communicating-with-the-rovl/serial-parameters
[^6]: Cerulean Sonar. (2025). “Packet Format.” https://docs.ceruleansonar.com/c/rov-locator/communicating-with-the-rovl/packet-format
