"""Listen to WOBL UDP telemetry broadcast and optionally record to .rrd.

Usage::

    python scripts/monitor.py                              # receive only
    python scripts/monitor.py --live                       # live Rerun viewer
    python scripts/monitor.py --save data/run.rrd          # record to file
    python scripts/monitor.py --live --save data/run.rrd   # both

While recording, press Enter to drop a checkpoint into the .rrd.

The firmware must have telemetry broadcasting enabled (console command ``b 1``).
"""

from __future__ import annotations

import argparse
import socket
import struct
from pathlib import Path

import pynput.keyboard

from woblpy.record import Recorder

_FMT = struct.Struct("<I?16f")
_FMT_SIZE = _FMT.size

# Each entry: (entity_path, display_name, colour).  Order must match the
# field order of BroadcastTelemetry in firmware/src/comms/broadcaster.cpp.
_ENTITIES: list[tuple[str, str, tuple[int, int, int]]] = [
    # Input Commands
    ("command/enable", "Cmd Enable", (200, 200, 200)),
    ("command/roll", "Cmd Roll", (255, 140, 200)),
    ("command/height", "Cmd Height", (255, 200, 140)),
    ("command/forward_velocity", "Cmd Fwd Vel", (160, 255, 160)),
    ("command/turn_velocity", "Cmd Turn Vel", (255, 255, 160)),
    # Observed State
    ("observer/pitch", "Pitch", (255, 160, 0)),
    ("observer/pitch_rate", "Pitch Rate", (255, 80, 80)),
    ("observer/roll", "Roll", (0, 200, 255)),
    ("observer/roll_rate", "Roll Rate", (80, 160, 255)),
    ("observer/forward_velocity", "Fwd Vel", (80, 255, 80)),
    ("observer/turn_velocity", "Turn Vel", (255, 255, 80)),
    ("observer/left_leg_height", "Left Leg Height", (180, 120, 255)),
    ("observer/right_leg_height", "Right Leg Height", (120, 180, 255)),
    # Control Output
    ("output/wheel/left", "Cmd Wheel Left", (255, 80, 255)),
    ("output/wheel/right", "Cmd Wheel Right", (200, 80, 200)),
    ("output/servo/left", "Cmd Servo Left", (80, 255, 255)),
    ("output/servo/right", "Cmd Servo Right", (80, 200, 200)),
]

_paths = [e[0] for e in _ENTITIES]


class EnterKeyTrigger:
    """Non-blocking "Enter was pressed" trigger."""

    def __init__(self) -> None:
        self._pressed = False
        self._listener: pynput.keyboard.Listener | None = None

    @property
    def active(self) -> bool:
        return self._listener is not None

    def start(self) -> None:
        self._listener = pynput.keyboard.Listener(on_press=self._on_press)
        self._listener.start()

    def stop(self) -> None:
        if self._listener is not None:
            self._listener.stop()
            self._listener = None

    def wasPressed(self) -> bool:
        pressed, self._pressed = self._pressed, False
        return pressed

    def _on_press(self, key: object) -> None:
        if key == pynput.keyboard.Key.enter:
            self._pressed = True


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
    enterTrigger = EnterKeyTrigger()

    recorder: Recorder | None = None
    if live or args.save is not None:
        recorder = Recorder("wobl-monitor", live=live, save_path=args.save)
        for path, name, colour in _ENTITIES:
            recorder.configure_series(path, name=name, color=colour)
        enterTrigger.start()

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
                data, _ = sock.recvfrom(4096)
            except TimeoutError:
                print("\rWaiting for telemetry broadcast...", end="", flush=True)
                continue
            if len(data) < _FMT_SIZE:
                continue

            ts_ms, *values = _FMT.unpack(data[:_FMT_SIZE])
            # cmd_enable arrives as a bool -> 0.0 / 1.0 for plotting
            values[0] = float(values[0])
            t_s = ts_ms / 1000.0
            fields = dict(zip(_paths, values))

            if recorder is not None:
                recorder.log_many(fields, t_s=t_s)

                if enterTrigger.wasPressed():
                    recorder.log_checkpoint(t_s)
                    print(f"\n  checkpoint @ t={t_s:.2f}s")

            pitch = fields["observer/pitch"]
            fwd = fields["observer/forward_velocity"]
            yaw = fields["observer/turn_velocity"]
            print(
                f"\rpitch={pitch:+.3f}  fwd={fwd:+.3f}  yaw={yaw:+.3f}",
                end="",
                flush=True,
            )
    except KeyboardInterrupt:
        print()
    finally:
        sock.close()
        enterTrigger.stop()
        if recorder is not None:
            recorder.close()

    print("Done.")


if __name__ == "__main__":
    main()
