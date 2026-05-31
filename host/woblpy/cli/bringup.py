import argparse
import signal
import sys

from woblpy.control.controller_loop import ControllerLoop
from woblpy.hardware.wobl_serial import WoblSerial
from woblpy.hardware.wobl_sim import WoblSim
from woblpy.record import Recorder


def main() -> None:
    parser = argparse.ArgumentParser(description="Bringup script for WOBL robot")
    parser.add_argument("mode", choices=["real", "sim"])
    parser.add_argument(
        "--port",
        default=None,
        help="Serial port for real mode (auto-detected if omitted)",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run sim without the MuJoCo viewer",
    )
    parser.add_argument(
        "--record",
        action="store_true",
        help="Record telemetry to data/bringup.rrd"
    )
    args = parser.parse_args()

    hardware: WoblSerial | WoblSim
    if args.mode == "real":
        hardware = WoblSerial.open(args.port)
    else:
        hardware = WoblSim(with_viewer=not args.headless)

    recorder = Recorder("record", save_path="data/bringup.rrd", live=False) if args.record else None
    loop = ControllerLoop(hardware, recorder=recorder)

    def _shutdown(sig: int, frame: object) -> None:
        print(f"\nReceived signal {sig}, shutting down…")
        loop.stop()
        hardware.close()
        if recorder is not None:
            recorder.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        if isinstance(hardware, WoblSim) and not args.headless:
            loop.start()
            hardware.app.launch()
            loop.stop()
        else:
            loop.run()
    finally:
        hardware.close()
        if recorder is not None:
            recorder.close()


if __name__ == "__main__":
    main()
