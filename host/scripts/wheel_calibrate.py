#!/usr/bin/env python3
"""Wheel FOC calibration utility.

This script triggers a calibration-only initFOC pass on one or both wheels,
reads back discovered zero electric angle + sensor direction, and persists
results to NVS.

Usage:
    uv run scripts/wheel_calibrate.py [--port COM3] [--wheel left|right|both]
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from woblpy.hardware.wobl_serial import WoblSerial

WHEEL_LEFT = 1
WHEEL_RIGHT = 2


RESET = "\033[0m"
BOLD = "\033[1m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
RED = "\033[31m"
CYAN = "\033[36m"


def _header(text: str) -> None:
    print(f"\n{BOLD}{CYAN}{'-' * 60}{RESET}")
    print(f"{BOLD}{CYAN}  {text}{RESET}")
    print(f"{BOLD}{CYAN}{'-' * 60}{RESET}")


def _ok(text: str) -> None:
    print(f"{GREEN}OK{RESET} {text}")


def _warn(text: str) -> None:
    print(f"{YELLOW}WARN{RESET} {text}")


def _err(text: str) -> None:
    print(f"{RED}ERR{RESET} {text}")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=None, help="Serial port (auto-detect if omitted).")
    parser.add_argument(
        "--wheel",
        choices=("left", "right", "both"),
        default="both",
        help="Which wheel(s) to calibrate.",
    )
    parser.add_argument(
        "--wait-s",
        type=float,
        default=5.0,
        help="Seconds to wait before readback.",
    )
    return parser.parse_args()


def _targets(wheel: str) -> list[tuple[int, str]]:
    if wheel == "left":
        return [(WHEEL_LEFT, "left")]
    if wheel == "right":
        return [(WHEEL_RIGHT, "right")]
    return [(WHEEL_LEFT, "left"), (WHEEL_RIGHT, "right")]


def _calibrate_one(hw: WoblSerial, target: int, name: str, wait_s: float) -> bool:
    _header(f"Calibrating {name} wheel")
    print("Place robot so wheels can spin freely, then press Enter.")
    input()

    accepted = hw.write_wheel_calibration(target=target)
    if not accepted:
        _err("Firmware rejected calibration command")
        return False

    time.sleep(wait_s)
    result = hw.read_wheel_calibration(target=target)

    print(f"zero_electric_angle:{result.zero_electric_angle:+.6f} rad")
    print(f"sensor_direction:   {result.sensor_direction}")

    _ok("Calibration pass completed")
    _ok("Calibration persisted to NVS")
    return True


def main() -> None:
    args = _parse_args()
    _header("WOBL Wheel Calibration")
    print("Connecting to ESP32...")
    hw = WoblSerial.open(port=args.port)

    status = hw.request_status()
    left_status = status.left_wheel_status
    right_status = status.right_wheel_status

    print(f"Left wheel status:  {left_status}")
    print(f"Right wheel status: {right_status}")

    if left_status != 1 or right_status != 1:
        _warn("One or both wheels are not OK. Calibration may fail or be inaccurate.")
        print("Make sure the robot is on a stable surface and both wheels can spin freely.")

    all_ok = True
    try:
        for target, name in _targets(args.wheel):
            ok = _calibrate_one(hw, target, name, wait_s=args.wait_s)
            all_ok = all_ok and ok
    finally:
        hw.close()

    _header("Done")
    if all_ok:
        _ok("Calibration command flow completed")
        print("Reboot ESP32 before enabling drive so startup initFOC uses saved values.")
    else:
        _warn("One or more wheels failed calibration")
        sys.exit(1)


if __name__ == "__main__":
    main()
