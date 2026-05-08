#!/usr/bin/env python3
"""Send MSG_CMD_DRIVE commands to the ESP32 and measure round-trip latency.

The script sends a small, safe wheel velocity (default 0.0 rad/s, disabled)
so it is safe to run without the robot being in a stable position.
Pass --velocity to set a non-zero wheel velocity (use with caution).

Usage:
    python scripts/esp32_echo.py
    python scripts/esp32_echo.py --port COM3 --count 100
    python scripts/esp32_echo.py --velocity 0.5   # 0.5 rad/s test
"""

import argparse
import statistics
import sys
import time

sys.path.insert(0, ".")

from woblpy.hardware.protocol import DriveCommand
from woblpy.hardware.wobl_serial import WoblSerial


def main() -> None:
    parser = argparse.ArgumentParser(description="Drive-telem round-trip latency test")
    parser.add_argument("--port", default=None)
    parser.add_argument("--count", type=int, default=50, help="Number of round-trips")
    parser.add_argument(
        "--velocity",
        type=float,
        default=0.0,
        help="Wheel velocity in rad/s sent to both wheels (default 0.0)",
    )
    parser.add_argument(
        "--enabled",
        action="store_true",
        default=False,
        help="Enable the wheel outputs (disabled by default for safety)",
    )
    args = parser.parse_args()

    cmd = DriveCommand(
        left_enabled=args.enabled,
        left_velocity=args.velocity,
        right_enabled=args.enabled,
        right_velocity=args.velocity,
    )

    print(f"Opening serial port {'(auto-detect)' if args.port is None else args.port} …")
    latencies: list[float] = []

    with WoblSerial.open(args.port) as hw:
        print(
            f"Connected.  Sending {args.count} drive commands "
            f"(enabled={args.enabled}, velocity={args.velocity:.3f} rad/s)…\n"
        )

        for i in range(args.count):
            t0 = time.monotonic()
            telem = hw.step_drive(cmd)
            rtt_ms = (time.monotonic() - t0) * 1000
            latencies.append(rtt_ms)

            if i % 10 == 0 or i == args.count - 1:
                w, x, y, z = telem.quat_wxyz
                print(
                    f"  [{i+1:4d}] RTT={rtt_ms:5.1f} ms  "
                    f"quat=({w:.3f}, {x:.3f}, {y:.3f}, {z:.3f})  "
                    f"lv={telem.left_vel:+.3f}  rv={telem.right_vel:+.3f}  "
                    f"t={telem.timestamp_ms} ms"
                )

    print(f"\nResults over {len(latencies)} samples:")
    print(f"  min  = {min(latencies):.2f} ms")
    print(f"  mean = {statistics.mean(latencies):.2f} ms")
    print(f"  p95  = {sorted(latencies)[int(0.95 * len(latencies))]:.2f} ms")
    print(f"  max  = {max(latencies):.2f} ms")


if __name__ == "__main__":
    main()
