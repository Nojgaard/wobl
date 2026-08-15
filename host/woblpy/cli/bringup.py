"""Bringup script for the WOBL robot simulation."""

import argparse
import signal

from woblpy.control.control_policy import ControlPolicy
from woblpy.control.keyboard_controller import KeyboardController
from woblpy.record import Recorder
from woblpy.sim.application import Application
from woblpy.sim.robot import Robot, RobotWorld

_DEFAULT_RECORD_PATH = "data/bringup.rrd"


def main() -> None:
    parser = argparse.ArgumentParser(description="Bringup script for WOBL robot")
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run sim without the MuJoCo viewer",
    )
    parser.add_argument(
        "--record",
        action="store_true",
        help="Record telemetry to a .rrd file",
    )
    parser.add_argument(
        "--save",
        default=_DEFAULT_RECORD_PATH,
        help=f"Output path for --record (default: {_DEFAULT_RECORD_PATH})",
    )
    args = parser.parse_args()

    recorder = (
        Recorder("bringup", save_path=args.save, live=False) if args.record else None
    )

    robot = Robot()
    world = RobotWorld(robot)

    policy = ControlPolicy(robot, recorder)

    # 100 Hz control rate matches the Controller's assumed dt; 200 Hz physics.
    world.set_timesteps(control_timestep=0.010, physics_timestep=0.001)
    app = Application(world, policy)

    kbd = KeyboardController(policy, max_fwd=0.3, max_yaw=0.6 / 11.9)
    kbd.start()

    def _shutdown(sig: int, frame: object) -> None:
        print(f"\nReceived signal {sig}, shutting down…")
        app.running = False

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        if args.headless:
            app.launch_headless()
        else:
            app.launch()
    finally:
        kbd.stop()
        if recorder is not None:
            recorder.close()


if __name__ == "__main__":
    main()
