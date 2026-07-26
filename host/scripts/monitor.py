"""Listen to WOBL UDP telemetry broadcast and optionally record to .rrd.

Usage::

    python scripts/monitor.py                           # live Rerun viewer
    python scripts/monitor.py --save data/run.rrd       # live + record to file
    python scripts/monitor.py --save data/run.rrd --no-live  # headless record

The firmware must have telemetry broadcasting enabled (console command ``b 1``).
"""

from __future__ import annotations

import argparse
import socket
import struct
from pathlib import Path
from typing import Optional

from woblpy.record import Recorder

_FMT = struct.Struct("<I12f")
_FMT_SIZE = _FMT.size

# Each entry: (entity_path, display_name, colour)
_ENTITIES: list[tuple[str, str, tuple[int, int, int]]] = [
    ("input/pitch", "Pitch", (255, 160, 0)),
    ("input/pitch_rate", "Pitch Rate", (255, 80, 80)),
    ("input/roll", "Roll", (0, 200, 255)),
    ("input/roll_rate", "Roll Rate", (80, 160, 255)),
    ("input/wheel/left_vel", "Left Wheel Vel", (80, 80, 255)),
    ("input/wheel/right_vel", "Right Wheel Vel", (255, 200, 0)),
    ("input/body/fwd_velocity", "Body Fwd Vel", (80, 255, 80)),
    ("input/body/yaw_rate", "Body Yaw Rate", (255, 255, 80)),
    ("output/target/fwd_velocity", "Target Fwd Vel", (160, 255, 160)),
    ("output/target/yaw_rate", "Target Yaw Rate", (255, 255, 160)),
    ("output/wheel/left_vel", "Cmd Wheel Left", (255, 80, 255)),
    ("output/wheel/right_vel", "Cmd Wheel Right", (200, 80, 200)),
]

_paths = [e[0] for e in _ENTITIES]


def main() -> None:
    parser = argparse.ArgumentParser(description="WOBL UDP telemetry monitor")
    parser.add_argument(
        "--save",
        type=Path,
        default=None,
        help="Save recording to .rrd file (e.g. data/run.rrd)",
    )
    parser.add_argument(
        "--live",
        action="store_true",
        help="Enable live Rerun viewer",
    )
    args = parser.parse_args()

    live = args.live
    recorder: Optional[Recorder] = None
    if live or args.save is not None:
        recorder = Recorder("wobl-monitor", live=live, save_path=args.save)
        for path, name, colour in _ENTITIES:
            recorder.configure_series(path, name=name, color=colour)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind(("0.0.0.0", 8888))
    sock.settimeout(0.5)

    print("WOBL Telemetry Monitor  —  Ctrl+C to stop")
    print(f"  Live view  {'yes' if live else 'no'}")
    print(f"  Save path  {args.save or 'n/a'}")

    try:
        while True:
            try:
                data, addr = sock.recvfrom(4096)
            except socket.timeout:
                print("timeout")
                continue
            if len(data) < _FMT_SIZE:
                continue

            ts_ms, *values = _FMT.unpack(data[:_FMT_SIZE])
            t_s = ts_ms / 1000.0
            fields = dict(zip(_paths, values))

            if recorder is not None:
                recorder.log_many(fields, t_s=t_s)

            pitch = fields["input/pitch"]
            fwd = fields["input/body/fwd_velocity"]
            yaw = fields["input/body/yaw_rate"]
            print(
                f"\rpitch={pitch:+.3f}  fwd={fwd:+.3f}  yaw={yaw:+.3f}",
                end="",
                flush=True,
            )
    except KeyboardInterrupt:
        print()
    finally:
        sock.close()
        if recorder is not None:
            recorder.close()

    print("Done.")


if __name__ == "__main__":
    main()
