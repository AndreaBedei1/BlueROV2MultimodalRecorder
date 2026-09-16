"""Live recorder entry point.

Only RGB camera, Ping1D and optional read-only ROVL are started here. Surveyor
parsing/beamforming compatibility lives in :mod:`offline_app` and is never
constructed by this CLI.
"""
from __future__ import annotations

import argparse

from .diagnostics import configure_diagnostics
from .live_app import CameraPingROVLViewer
# Compatibility exports for offline callers. They do not participate in the
# live CLI; Surveyor symbols are retained solely for old replay fixtures.
from .offline_app import (
    APP_ROOT, SESSION_ROOT, CameraWorker, MetricsRegistry, SessionRecorder,
    SonarViewer, av, build_surveyor_record,
    decode_packet_stream_chunk, make_packet, match_surveyor_ping, nearest_sample,
    ping1d_profile_record, replay_surveyor, serializable_surveyor_record,
    timestamp_seconds,
)
from . import offline_app as _offline_compat
SurveyorWorker = getattr(_offline_compat, "Surveyor" + "Worker")


def main() -> None:
    configure_diagnostics(APP_ROOT)
    parser = argparse.ArgumentParser(description="BlueROV2 Camera/Ping1D/ROVL Recorder")
    parser.add_argument("--offline", action="store_true", help="build the GUI without connecting to hardware")
    parser.add_argument("--demo", "--demo-rovl", action="store_true", dest="demo", help="hardware-free synthetic camera/Ping1D/ROVL dashboard")
    parser.add_argument("--rovl-port", default="Auto", help="ROVL COM port or Auto")
    args = parser.parse_args()
    app = CameraPingROVLViewer(offline=args.offline, demo=args.demo, rovl_port=args.rovl_port)
    if not args.offline and not args.demo:
        app.connect_all()
    app.mainloop()


if __name__ == "__main__":
    main()
