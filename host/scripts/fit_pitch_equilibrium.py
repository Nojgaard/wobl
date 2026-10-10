"""Fit the pitch-equilibrium polynomial for the WOBL balancing controller.

Two sources of measured (height, pitch) data are supported:

* default          — sweep the sim, settling the closed-loop leg linkage at each
                     height and measuring the body-frame center-of-mass offset
                     relative to the wheel axle.
* ``--recording``  — average the last ``--window`` seconds before each
                     checkpoint of a ``monitor.py`` recording.

Either way it fits

    theta_eq(h) = p0*h^3 + p1*h^2 + p2*h + p3

and prints the coefficients for the firmware controller.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from dm_control import mjcf

from woblpy.control.leg_kinematics import LegKinematics
from woblpy.control.pitch_equilibrium import HEIGHT_TO_PITCH_COEFFS
from woblpy.record import load_recording
from woblpy.sim.robot import Robot

_SETTLE_STEPS = 1500
_HEIGHT_RANGE = (0.06, 0.16)  # MotionController._MIN/_MAX_HEIGHT


def measure_equilibrium(heights: np.ndarray) -> np.ndarray:
    """Return an Nx2 array of (height, theta_eq) rows for the given heights."""
    robot = Robot()
    leg_ik = LegKinematics(robot.leg_keypoints, robot.servo_limits())
    physics = mjcf.Physics.from_mjcf_model(robot.mjcf_model)

    hip_act_l = robot.mjcf_model.find("actuator", "L_hip")
    hip_act_r = robot.mjcf_model.find("actuator", "R_hip")
    hip_l = robot.mjcf_model.find("joint", "L_hip")

    rows = []
    for height in heights:
        physics.reset()
        physics.bind(hip_act_l).ctrl = leg_ik.to_angle(height)
        physics.bind(hip_act_r).ctrl = leg_ik.to_angle(height)
        for _ in range(_SETTLE_STEPS):  # let the <connect> loop close
            physics.step()

        com = np.array(robot.com(physics))
        com[2] = com[2]
        print(com)
        actual = leg_ik.to_height(np.asarray(physics.bind(hip_l).qpos).item())
        rows.append((actual, -np.arctan2(com[0], com[2])))

    return np.array(rows)


def measure_from_recording(path: Path, window: float, min_samples: int) -> np.ndarray:
    """Return an Nx2 array of (height, pitch) rows averaged before each checkpoint."""
    df, checkpoints = load_recording(path)
    height = (df["observer/left_leg_height"] + df["observer/right_leg_height"]) / 2
    pitch = df["observer/pitch"]
    wheel = (df["output/wheel/left"] + df["output/wheel/right"]) / 2

    rows = []
    for t in checkpoints:
        recent = (df.index > t - window) & (df.index <= t)
        n = int(recent.sum())
        h, p = height[recent].mean(), pitch[recent].mean()
        print(
            f"  t={t:7.2f}s  h={h:+.4f}  pitch={p:+.4f}  "
            f"wheel={wheel[recent].mean():+.4f}  n={n:5d}"
        )
        if n >= min_samples:  # sparse windows are packet loss, not a settled robot
            rows.append((h, p))

    data = np.array(rows).reshape(-1, 2)
    return data[~np.isnan(data).any(axis=1)]


def main() -> None:
    parser = argparse.ArgumentParser(description="Fit the pitch-equilibrium polynomial")
    parser.add_argument(
        "--recording",
        type=Path,
        default=None,
        help="Fit from a monitor.py .rrd instead of the sim sweep",
    )
    parser.add_argument(
        "--window",
        type=float,
        default=2.0,
        help="Seconds averaged before each checkpoint (default: 2.0)",
    )
    parser.add_argument(
        "--min-samples",
        type=int,
        default=20,
        help="Skip checkpoints with fewer samples in the window (default: 20)",
    )
    args = parser.parse_args()

    if args.recording is not None:
        print(f"checkpoints in {args.recording.name}:")
        data = measure_from_recording(args.recording, args.window, args.min_samples)
        source = args.recording.name
    else:
        data = measure_equilibrium(np.linspace(*_HEIGHT_RANGE, 25))
        source = "sim"

    if len(data) < 4:
        raise SystemExit(f"only {len(data)} usable points")

    coeffs = np.polyfit(data[:, 0], data[:, 1], 3)
    residual = np.polyval(coeffs, data[:, 0]) - data[:, 1]

    print(f"\n{len(data)} points")
    print("coefficients (h^3, h^2, h, 1):")
    print(coeffs)
    print(f"max residual: {np.max(np.abs(residual)) * 1000:.3f} mrad")

    heights = data[:, 0]
    grid = np.linspace(heights.min(), heights.max(), 200)
    plt.plot(heights, data[:, 1], "o", label=f"measured ({source})")
    plt.plot(grid, np.polyval(coeffs, grid), "-", label="fit")
    plt.plot(grid, np.polyval(HEIGHT_TO_PITCH_COEFFS, grid), "--", label="flashed")
    plt.xlabel("height (m)")
    plt.ylabel("theta_eq (rad)")
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
