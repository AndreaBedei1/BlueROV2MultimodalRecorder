# Future 3D reconstruction

_Design boundary for a future global seabed reconstruction pipeline._

---

## 🎯 Objective

The long-term objective is to operate the Surveyor while the BlueROV2 moves over or beside the seabed and fuse successive sonar measurements into a global point cloud or another 3D representation of the surveyed area.

## 📐 Local sonar geometry

An ATOF detection contains an angle and a time of flight. Using the speed of sound recorded for that ping, the application can derive a range and a local sensor-frame point. The channel-data fan is also local to the Surveyor aperture and its current range/angle display.

This is not yet a global map. Successive pings are acquired at different vehicle poses, so their local coordinates cannot be fused correctly without a time-aligned transform from the sensor frame to a common vehicle/world frame.

## 🧭 Pose required for fusion

A future pose stream should record, when available:

- roll, pitch, and yaw
- depth
- local `x`, `y`, and `z` position
- velocity
- source and quality indicators
- host monotonic and UTC timestamps
- device or telemetry timestamps when provided

The pose must also be related to the Surveyor mounting transform and any time offset between the vehicle telemetry and sonar acquisition.

## 🔒 No fabricated global reconstruction

The current repository does not emit a global point cloud from guessed vehicle motion. Attitude alone is insufficient for horizontal translation; depth alone is insufficient for full pose. Without a verified position/velocity source, a global reconstruction would conflate sensor geometry with an unsupported trajectory estimate.

## 🛰️ Planned telemetry extension

The intended additive output is:

```text
vehicle_telemetry.jsonl
```

The file should be optional and referenced from `session.json` only when an implementation has been verified on the deployed BlueROV2. A read-only MAVLink/BlueOS integration may be evaluated later, but it must not arm, actuate, change parameters, or interfere with the vehicle. The current codebase leaves this as a TODO rather than inventing telemetry values.

## 🗺️ Suggested future processing stages

1. Validate and time-stamp vehicle telemetry without changing vehicle state.
2. Measure the Surveyor-to-vehicle rigid transform.
3. Quantify sensor and telemetry latency.
4. Convert each ping into local metric geometry with provenance.
5. Transform samples into a common frame only when pose quality passes explicit checks.
6. Export a point cloud with uncertainty and source timestamps.

The existing raw packet and processed JSONL files are retained so this extension can be implemented without discarding previously recorded sessions.

