#!/usr/bin/env python3
"""Send MSG_STATUS_REQ to the ESP32 and pretty-print the response.

Usage:
    python scripts/esp32_ping.py
    python scripts/esp32_ping.py --port /dev/ttyUSB0
    python scripts/esp32_ping.py --port COM3
"""

import argparse
import sys
import time

sys.path.insert(0, ".")

from woblpy.hardware.wobl_serial import WoblSerial


def main() -> None:
    parser = argparse.ArgumentParser(description="Ping the WOBL ESP32 over serial")
    parser.add_argument("--port", default=None, help="Serial port (auto-detected if omitted)")
    parser.add_argument("--count", type=int, default=5, help="Number of pings (default: 5)")
    parser.add_argument("--interval", type=float, default=0.5, help="Seconds between pings")
    args = parser.parse_args()

    print(f"Opening serial port {'(auto-detect)' if args.port is None else args.port} …")
    with WoblSerial.open(args.port) as hw:
        print("Connected.  Sending status requests…\n")
        for i in range(args.count):
            t0 = time.monotonic()
            status = hw.request_status()
            rtt_ms = (time.monotonic() - t0) * 1000

            print(f"Ping {i + 1}/{args.count}  RTT={rtt_ms:.1f} ms")
            print(f"  IMU:    status={status.imu_status}  rate={status.imu_rate:.1f} Hz")
            print(f"  Wheels: foc={status.foc_rate:.1f} Hz  update={status.wheel_rate:.1f} Hz")
            print(f"          left_status={status.left_wheel_status}  right_status={status.right_wheel_status}")
            print(f"  Servos: left_ok={status.left_servo_ok}  right_ok={status.right_servo_ok}")
            print()

            if i < args.count - 1:
                time.sleep(args.interval)

    print("Done.")


if __name__ == "__main__":
    main()
