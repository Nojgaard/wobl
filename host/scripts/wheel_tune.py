#!/usr/bin/env python3
"""Interactive wheel velocity tuning utility.

Firmware stores tuning per wheel, while this script defaults to applying one
set of values to both wheels (global-by-default UX).

Usage:
    uv run scripts/wheel_tune.py [--port COM3]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from woblpy.hardware.protocol import WheelTuningPayload
from woblpy.hardware.wobl_serial import WoblSerial

WHEEL_LEFT = 1
WHEEL_RIGHT = 2


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=None, help="Serial port (auto-detect if omitted).")
    parser.add_argument(
        "--wheel",
        choices=("left", "right", "both"),
        default="both",
        help="Target wheel(s).",
    )
    parser.add_argument("--persist", action="store_true", help="Persist values to NVS.")
    parser.add_argument("--p", type=float, default=None)
    parser.add_argument("--i", type=float, default=None)
    parser.add_argument("--d", type=float, default=None)
    parser.add_argument("--tf", type=float, default=None, help="LPF velocity Tf")
    parser.add_argument("--velocity-limit", type=float, default=None)
    parser.add_argument("--voltage-limit", type=float, default=None)
    return parser.parse_args()


def _targets(wheel: str) -> list[int]:
    if wheel == "left":
        return [WHEEL_LEFT]
    if wheel == "right":
        return [WHEEL_RIGHT]
    return [WHEEL_LEFT, WHEEL_RIGHT]


def _prompt_float(label: str, default: float) -> float:
    text = input(f"{label} [{default}]: ").strip()
    if not text:
        return default
    return float(text)


def _collect_values(args: argparse.Namespace, base: WheelTuningPayload) -> dict[str, float]:
    vals: dict[str, float] = {}
    vals["p"] = args.p if args.p is not None else _prompt_float("P", base.p)
    vals["i"] = args.i if args.i is not None else _prompt_float("I", base.i)
    vals["d"] = args.d if args.d is not None else _prompt_float("D", base.d)
    vals["tf"] = args.tf if args.tf is not None else _prompt_float("LPF Tf", base.lpf_velocity_tf)
    vals["velocity_limit"] = (
        args.velocity_limit
        if args.velocity_limit is not None
        else _prompt_float("Velocity limit", base.velocity_limit)
    )
    vals["voltage_limit"] = (
        args.voltage_limit
        if args.voltage_limit is not None
        else _prompt_float("Voltage limit", base.voltage_limit)
    )
    return vals


def _print_tuning(prefix: str, t: WheelTuningPayload) -> None:
    print(
        f"{prefix} target={t.target} p={t.p:.4f} i={t.i:.4f} d={t.d:.4f} "
        f"tf={t.lpf_velocity_tf:.4f} vel_lim={t.velocity_limit:.2f} "
        f"volt_lim={t.voltage_limit:.2f}"
    )


def main() -> None:
    args = _parse_args()
    hw = WoblSerial.open(port=args.port)

    try:
        base = hw.read_wheel_tuning(WHEEL_LEFT)
        print("Current tuning (left wheel baseline):")
        _print_tuning("-", base)

        vals = _collect_values(args, base)

        for target in _targets(args.wheel):
            ok = hw.write_wheel_tuning(
                target=target,
                p=vals["p"],
                i=vals["i"],
                d=vals["d"],
                lpf_velocity_tf=vals["tf"],
                velocity_limit=vals["velocity_limit"],
                voltage_limit=vals["voltage_limit"],
                persist=args.persist,
            )
            if not ok:
                raise RuntimeError(f"write_wheel_tuning failed for target {target}")

        print("\nReadback:")
        for target in _targets(args.wheel):
            t = hw.read_wheel_tuning(target)
            _print_tuning("-", t)

        if args.persist:
            print("\nValues were persisted to NVS.")
    finally:
        hw.close()


if __name__ == "__main__":
    main()
