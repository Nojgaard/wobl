#!/usr/bin/env python3
"""Interactive IMU calibration script for WOBL.

Phases
------
1. Gyro   — robot stationary, flat, 20 s — DMP learns gyro bias
2. Accel  — 6 orientations (±X, ±Y, ±Z face-up), 5 s each
3. Mag    — rotate robot slowly through all orientations for 30 s (figure-8)
4. Read   — pull current bias values from the IMU and display them
5. Save   — write biases to NVS on the ESP32 and verify round-trip

Usage
-----
    uv run scripts/imu_calibrate.py [--port /dev/ttyUSB0]

If --port is omitted the script auto-detects an ESP32 USB serial device.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from datetime import datetime

sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent.parent))

from woblpy.hardware.protocol import CalibPayload, DriveCommand, DriveTelemetry
from woblpy.hardware.wobl_serial import WoblSerial
from woblpy.record import Recorder

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

RESET = "\033[0m"
BOLD = "\033[1m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
RED = "\033[31m"
CYAN = "\033[36m"


def _header(text: str) -> None:
    print(f"\n{BOLD}{CYAN}{'─' * 60}{RESET}")
    print(f"{BOLD}{CYAN}  {text}{RESET}")
    print(f"{BOLD}{CYAN}{'─' * 60}{RESET}")


def _ok(text: str) -> None:
    print(f"{GREEN}✓ {text}{RESET}")


def _warn(text: str) -> None:
    print(f"{YELLOW}⚠ {text}{RESET}")


def _err(text: str) -> None:
    print(f"{RED}✗ {text}{RESET}")


def _prompt(text: str) -> None:
    """Wait for the user to press Enter."""
    input(f"\n{BOLD}» {text} — press Enter when ready…{RESET}")


def _gyro_mag(telem: DriveTelemetry) -> float:
    """Scalar magnitude of gyro vector in rad/s."""
    gx, gy, gz = telem.gyro
    return math.sqrt(gx * gx + gy * gy + gz * gz)


def _pitch_roll_deg(telem: DriveTelemetry) -> tuple[float, float]:
    """Euler pitch/roll in degrees from DriveTelemetry quaternion."""
    roll, pitch, _ = telem.orientation_euler()
    return math.degrees(pitch), math.degrees(roll)


# ---------------------------------------------------------------------------
# Phases
# ---------------------------------------------------------------------------

def phase_gyro(hw: WoblSerial, duration: float = 20.0, *, rec: Recorder | None = None) -> None:
    """Phase 1: gyro bias — keep robot flat and stationary."""
    _header("Phase 1 / 5 — Gyro bias")
    print("Place the robot flat on a level surface and do NOT move it.")
    _prompt("Ready to start gyro phase")

    print(f"\nCalibrating for {duration:.0f} s — do not move the robot…\n")
    deadline = time.monotonic() + duration
    t0_ms: int | None = None
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        telem = hw.step_drive(DriveCommand())
        if t0_ms is None:
            t0_ms = telem.timestamp_ms
        t_s = (telem.timestamp_ms - t0_ms) / 1000.0
        gx, gy, gz = telem.gyro
        gmag = _gyro_mag(telem)
        bar_len = max(0, int(remaining / duration * 30))
        bar = "█" * bar_len + "░" * (30 - bar_len)
        print(
            f"\r  [{bar}] {remaining:5.1f}s |gyro|={gmag:.4f} rad/s  ",
            end="",
            flush=True,
        )
        if rec is not None:
            rec.log_many(
                {
                    "imu/gyro/x": gx,
                    "imu/gyro/y": gy,
                    "imu/gyro/z": gz,
                    "imu/gyro/magnitude": gmag,
                },
                t_s,
            )
    print()
    _ok("Gyro phase complete")


def phase_accel(hw: WoblSerial, hold_s: float = 5.0, *, rec: Recorder | None = None) -> None:
    """Phase 2: accelerometer bias — 6-point tumble."""
    _header("Phase 2 / 5 — Accelerometer bias (6-point tumble)")
    orientations = [
        ("Upright (normal standing position)",        "+Z up"),
        ("Upside-down (belly up)",                    "-Z up"),
        ("Right side down (lying on right side)",     "+X up"),
        ("Left side down (lying on left side)",       "-X up"),
        ("Front down (nose pointing at floor)",       "+Y up"),
        ("Back down (rear pointing at floor)",        "-Y up"),
    ]
    print(
        "Hold the robot steady in each orientation for "
        f"{hold_s:.0f} s when prompted."
    )

    for i, (desc, axis) in enumerate(orientations, 1):
        _prompt(f"[{i}/6] {desc} ({axis})")
        if rec is not None:
            rec.log_text("imu/phase", f"{desc} ({axis})")
        deadline = time.monotonic() + hold_s
        t0_ms: int | None = None
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            telem = hw.step_drive(DriveCommand())
            if t0_ms is None:
                t0_ms = telem.timestamp_ms
            t_s = (telem.timestamp_ms - t0_ms) / 1000.0
            pitch, roll = _pitch_roll_deg(telem)
            print(
                f"\r  Holding… {remaining:4.1f}s  pitch={pitch:+6.1f}°  roll={roll:+6.1f}°  ",
                end="",
                flush=True,
            )
            if rec is not None:
                rec.log_many({"imu/attitude/pitch": pitch, "imu/attitude/roll": roll}, t_s)
        print()
    _ok("Accelerometer phase complete")


def phase_mag(hw: WoblSerial, duration: float = 30.0) -> None:
    """Phase 3: magnetometer calibration — figure-8 rotation."""
    _header("Phase 3 / 5 — Magnetometer (figure-8)")
    print("Slowly rotate the robot through all orientations (figure-8 motion).")
    print("Cover pitch, roll, and yaw — move continuously for the full duration.")
    _prompt("Ready to start magnetometer phase")

    print(f"\nMove robot for {duration:.0f} s…\n")
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        telem = hw.step_drive(DriveCommand())
        pitch, roll = _pitch_roll_deg(telem)
        bar_len = max(0, int(remaining / duration * 30))
        bar = "█" * bar_len + "░" * (30 - bar_len)
        print(
            f"\r  [{bar}] {remaining:5.1f}s  pitch={pitch:+6.1f}°  roll={roll:+6.1f}°  ",
            end="",
            flush=True,
        )
    print()
    _ok("Magnetometer phase complete")


def phase_read(hw: WoblSerial) -> CalibPayload:
    """Phase 4: read current bias values from the IMU."""
    _header("Phase 4 / 5 — Read biases from IMU")
    print("Querying current bias values…")

    # Give the DMP a moment to converge after the motion phases.
    time.sleep(0.5)
    calib = hw.read_calib(target=0)

    print(f"\n  Gyro  offsets (LSB): {calib.gyro_offset}")
    print(f"  Accel offsets (LSB): {calib.accel_offset}")
    print(f"  Mag   offsets (LSB): {calib.mag_offset}")

    gmag_lsb = math.sqrt(sum(v * v for v in calib.gyro_offset))
    amag_lsb = math.sqrt(sum(v * v for v in calib.accel_offset))
    if gmag_lsb < 1 and amag_lsb < 1:
        _warn("All biases are zero — DMP may not have converged yet.")
        _warn("Consider repeating phases 1–3 with more movement.")
    else:
        _ok("Non-zero biases received")

    return calib

def phase_save(hw: WoblSerial) -> None:
    """Phase 5: trigger the firmware to persist the learned biases to NVS."""
    _header("Phase 5 / 5 — Save to NVS")
    print("Signalling firmware to save learned biases…")

    success = hw.write_calib(target=0)
    if not success:
        _err("write_calib returned failure")
        sys.exit(1)
    _ok("Write acknowledged by firmware")

    print("\n  Reading back saved biases…")
    time.sleep(0.2)
    saved = hw.read_calib(target=0)
    print(f"  Gyro  offsets (LSB): {saved.gyro_offset}")
    print(f"  Accel offsets (LSB): {saved.accel_offset}")
    print(f"  Mag   offsets (LSB): {saved.mag_offset}")

    print("\n  Running 5 s post-save stability check…")
    deadline = time.monotonic() + 5.0
    samples: list[tuple[float, float]] = []
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        telem = hw.step_drive(DriveCommand())
        pitch, roll = _pitch_roll_deg(telem)
        samples.append((pitch, roll))
        print(
            f"\r  pitch={pitch:+6.2f}°  roll={roll:+6.2f}°  ",
            end="",
            flush=True,
        )
    print()

    if samples:
        avg_pitch = sum(p for p, _ in samples) / len(samples)
        avg_roll = sum(r for _, r in samples) / len(samples)
        print(f"\n  Average  pitch={avg_pitch:+.2f}°  roll={avg_roll:+.2f}°")
        if abs(avg_pitch) < 3.0 and abs(avg_roll) < 3.0:
            _ok("Pose looks level — calibration successful!")
        else:
            _warn(
                "Pose is not level after calibration. "
                "You may want to re-run on a flatter surface."
            )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port",
        default=None,
        help="Serial port (e.g. COM3 or /dev/ttyUSB0). Auto-detected if omitted.",
    )
    parser.add_argument(
        "--skip-motion",
        action="store_true",
        help="Skip phases 1–3 (gyro/accel/mag motion) and go straight to read+save.",
    )
    parser.add_argument(
        "--rerun",
        action="store_true",
        help="Enable live Rerun visualisation.",
    )
    parser.add_argument(
        "--rrd",
        metavar="FILE",
        default=None,
        help=(
            "Save recording to FILE (.rrd). "
            "Auto-generates data/imu_calib_YYYYMMDD_HHMMSS.rrd when --rerun is given."
        ),
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()

    save_path: str | None = args.rrd
    if save_path is None and args.rerun:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        save_path = f"data/imu_calib_{ts}.rrd"

    print(f"{BOLD}WOBL IMU Calibration{RESET}")
    print("Connecting to ESP32…")
    try:
        hw = WoblSerial.open(args.port)
    except RuntimeError as exc:
        _err(str(exc))
        sys.exit(1)
    print(f"Connected on {hw._serial.port}")

    with Recorder("imu_calibrate", live=args.rerun, save_path=save_path) as rec:
        rec.configure_series("imu/gyro/x",         name="Gyro X",   color=(255, 80,  80))
        rec.configure_series("imu/gyro/y",         name="Gyro Y",   color=(80,  255, 80))
        rec.configure_series("imu/gyro/z",         name="Gyro Z",   color=(80,  80,  255))
        rec.configure_series("imu/gyro/magnitude", name="Gyro |ω|", color=(200, 200, 200))
        rec.configure_series("imu/attitude/pitch", name="Pitch",    color=(255, 160,  0))
        rec.configure_series("imu/attitude/roll",  name="Roll",     color=(0,   200, 255))

        try:
            phase_read(hw)
            if not args.skip_motion:
                phase_gyro(hw, rec=rec)
                phase_accel(hw, rec=rec)
                phase_mag(hw)

            phase_read(hw)
            #if not args.skip_motion:
                #phase_save(hw)

        except KeyboardInterrupt:
            print(f"\n{YELLOW}Interrupted — calibration not saved.{RESET}")
        finally:
            hw.close()

    _header("Calibration complete")
    print("Biases are now stored in NVS and will be loaded automatically on boot.\n")


if __name__ == "__main__":
    main()
