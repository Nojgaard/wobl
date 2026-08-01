"""Compute LQR gains and upload them to the firmware via the serial console.

Usage::

    python scripts/upload_lqr.py                   # auto-detect port
    python scripts/upload_lqr.py --port COM5
    python scripts/upload_lqr.py --offset 0.05
    python scripts/upload_lqr.py --dry-run         # preview only
"""

import argparse
import time

import serial
import serial.tools.list_ports

from woblpy.control.lqr import compute_lqr_gains

_BAUD = 115200
_ESP32_VIDS = {0x303A, 0x10C4, 0x1A86}

# Console key → index into K returned by compute_lqr_gains()
# K = [pitchKp, pitchRateKp, velocityKp, positionKp]
_GAINS = {"p": 0, "r": 1, "v": 2, "x": 3}


def _detect_port() -> str:
    ports = [
        p.device for p in serial.tools.list_ports.comports() if p.vid in _ESP32_VIDS
    ]
    if not ports:
        raise RuntimeError("No ESP32 found. Pass --port or connect the device.")
    if len(ports) > 1:
        print(f"Multiple ports: {ports}; using {ports[0]}")
    return ports[0]


def main() -> None:
    parser = argparse.ArgumentParser(description="Upload LQR gains to firmware")
    parser.add_argument("--port", default=None)
    parser.add_argument(
        "--offset", type=float, default=0.07, help="Pitch offset (rad, default 0.07)"
    )
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    K = compute_lqr_gains()
    print(
        f"Computed:  pitchKp={K[0]:.4f}  pitchRateKp={K[1]:.4f}  "
        f"velocityKp={K[2]:.4f}  positionKp={K[3]:.4f}  "
        f"offset={args.offset:.4f}"
    )

    scale = 1.0

    if args.dry_run:
        return

    with serial.Serial(args.port or _detect_port(), _BAUD, timeout=0.5) as ser:
        time.sleep(0.2)
        ser.reset_input_buffer()

        for key, idx in _GAINS.items():
            ser.write(f"g {key}={K[idx]:.6f}\n".encode())
            print(f"  {ser.readline().decode().strip()}")

        ser.write(f"g o={args.offset:.6f}\n".encode())
        print(f"  {ser.readline().decode().strip()}")

        ser.write(f"g c={scale:.6f}\n".encode())
        print(f"  {ser.readline().decode().strip()}")

        ser.write(b"g\n")
        resp = ser.readline().decode().strip()
        print(f"\n  Current: {resp}")


if __name__ == "__main__":
    main()
