#!/usr/bin/env python3
"""Synthetic end-to-end test for Recorder — no hardware required.

Simulates the data produced during IMU calibration:
  Phase 1 — gyro noise with decaying amplitude (mimics DMP convergence)
  Phase 2 — 6 orientations × hold period (mimics 6-point tumble)

Usage
-----
    uv run scripts/test_recorder.py            # live Rerun viewer + save .rrd
    uv run scripts/test_recorder.py --no-rerun  # save .rrd only, no viewer
"""

from __future__ import annotations

import argparse
import math
import random
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from woblpy.record import Recorder

RESET = "\033[0m"
BOLD  = "\033[1m"
GREEN = "\033[32m"
CYAN  = "\033[36m"

# Simulated loop rate matching the real IMU telemetry rate (~100 Hz)
_DT_MS = 10          # ms per sample
_DT_S  = _DT_MS / 1000.0


# ---------------------------------------------------------------------------
# Phase simulations
# ---------------------------------------------------------------------------

def simulate_gyro_phase(rec: Recorder, duration: float = 5.0) -> None:
    """Gaussian noise with amplitude decaying over ``duration`` s.

    Starts at std≈0.05 rad/s and converges toward std≈0.005 rad/s, mimicking
    the DMP bias-learning process visible in phase 1 of imu_calibrate.py.
    """
    print(f"\n{BOLD}Simulating Phase 1 — Gyro bias ({duration:.0f} s)…{RESET}")
    total_ms = int(duration * 1000)
    t_ms = 0
    while t_ms <= total_ms:
        progress = t_ms / total_ms
        std = 0.05 * (1.0 - 0.9 * progress)   # decays 0.05 → 0.005 rad/s
        gx = random.gauss(0.0, std)
        gy = random.gauss(0.0, std)
        gz = random.gauss(0.0, std)
        gmag = math.sqrt(gx * gx + gy * gy + gz * gz)
        t_s = t_ms / 1000.0
        rec.log_many(
            {
                "imu/gyro/x":         gx,
                "imu/gyro/y":         gy,
                "imu/gyro/z":         gz,
                "imu/gyro/magnitude": gmag,
            },
            t_s,
        )
        print(
            f"\r  t={t_s:5.2f}s  |ω|={gmag:.4f} rad/s  std≈{std:.4f}",
            end="",
            flush=True,
        )
        t_ms += _DT_MS
        time.sleep(_DT_S)
    print()
    print(f"{GREEN}✓ Gyro phase done{RESET}")


# (label, target_pitch_deg, target_roll_deg)
_ORIENTATIONS: list[tuple[str, float, float]] = [
    ("Upright (+Z up)",          0.0,    0.0),
    ("Upside-down (-Z up)",      0.0,  180.0),
    ("Right side down (+X up)",  0.0,   90.0),
    ("Left side down (-X up)",   0.0,  -90.0),
    ("Front down (+Y up)",      90.0,    0.0),
    ("Back down (-Y up)",      -90.0,    0.0),
]


def simulate_accel_phase(rec: Recorder, hold_s: float = 2.0) -> None:
    """Step through 6 orientations, holding each for ``hold_s`` s.

    The global timestamp advances continuously across all orientations so the
    Rerun viewer shows the full sequence on a single timeline.
    """
    print(
        f"\n{BOLD}Simulating Phase 2 — Accel 6-point "
        f"({hold_s:.0f} s / orientation)…{RESET}"
    )
    # Phase 2 timeline starts right after the gyro phase (t_s offset is 0 here
    # because each phase uses its own t0; the viewer treats them independently).
    t_ms = 0
    for desc, target_pitch, target_roll in _ORIENTATIONS:
        rec.log_text("imu/phase", desc)
        t_hold_ms = 0
        while t_hold_ms < int(hold_s * 1000):
            pitch = target_pitch + random.gauss(0.0, 0.3)
            roll  = target_roll  + random.gauss(0.0, 0.3)
            t_s = t_ms / 1000.0
            rec.log_many({"imu/attitude/pitch": pitch, "imu/attitude/roll": roll}, t_s)
            print(
                f"\r  [{desc[:26]:<26}]  pitch={pitch:+6.1f}°  roll={roll:+6.1f}°  ",
                end="",
                flush=True,
            )
            t_ms      += _DT_MS
            t_hold_ms += _DT_MS
            time.sleep(_DT_S)
        print()
    print(f"{GREEN}✓ Accel phase done{RESET}")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--no-rerun",
        action="store_true",
        help="Skip live Rerun viewer (save .rrd only).",
    )
    p.add_argument(
        "--rrd",
        metavar="FILE",
        default="data/test_recorder.rrd",
        help="Output .rrd path (default: data/test_recorder.rrd).",
    )
    return p.parse_args()


def main() -> None:
    args = _parse_args()
    rrd_path = Path(args.rrd)

    print(f"{BOLD}Recorder synthetic test{RESET}")
    print(f"  live={not args.no_rerun}  save → {rrd_path}")

    with Recorder("imu_calibrate", live=not args.no_rerun, save_path=rrd_path) as rec:
        rec.configure_series("imu/gyro/x",         name="Gyro X",   color=(255, 80,  80))
        rec.configure_series("imu/gyro/y",         name="Gyro Y",   color=(80,  255, 80))
        rec.configure_series("imu/gyro/z",         name="Gyro Z",   color=(80,  80,  255))
        rec.configure_series("imu/gyro/magnitude", name="Gyro |ω|", color=(200, 200, 200))
        rec.configure_series("imu/attitude/pitch", name="Pitch",    color=(255, 160,  0))
        rec.configure_series("imu/attitude/roll",  name="Roll",     color=(0,   200, 255))

        simulate_gyro_phase(rec)
        simulate_accel_phase(rec)

    print(f"\n{CYAN}To inspect the recording:{RESET}")
    print(
        f'  python -c "import rerun.recording as rrd; '
        f"print(rrd.load_recording(r'{rrd_path}').schema())\""
    )


if __name__ == "__main__":
    main()
