"""Fit the pitch-equilibrium polynomial for the WOBL balancing controller.

Sweeps the robot height through its operating range, settles the closed-loop
leg linkage at each height, and measures the body-frame center-of-mass offset
relative to the wheel axle. Fits

    theta_eq(h) = p0*h^3 + p1*h^2 + p2*h + p3

and prints the coefficients for the firmware controller.
"""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np
from dm_control import mjcf

from woblpy.control.leg_kinematics import LegKinematics
from woblpy.sim.robot import Robot

_SETTLE_STEPS = 1500
_HEIGHT_RANGE = (0.06, 0.16)  # MotionController._MIN/_MAX_HEIGHT


def measure_equilibrium(heights: np.ndarray) -> np.ndarray:
    """Return an Nx2 array of (height, theta_eq) rows for the given heights."""
    robot = Robot()
    leg_ik = LegKinematics(robot.leg_keypoints)
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


def main() -> None:
    data = measure_equilibrium(np.linspace(*_HEIGHT_RANGE, 25))
    coeffs = np.polyfit(data[:, 0], data[:, 1], 3)
    residual = np.polyval(coeffs, data[:, 0]) - data[:, 1]

    print("\ncoefficients (h^3, h^2, h, 1):")
    print(coeffs)
    print(f"max residual: {np.max(np.abs(residual)) * 1000:.3f} mrad")

    heights = data[:, 0]
    grid = np.linspace(heights.min(), heights.max(), 200)
    plt.plot(heights, data[:, 1], "o", label="measured")
    plt.plot(grid, np.polyval(coeffs, grid), "-", label="fit")
    plt.xlabel("height (m)")
    plt.ylabel("theta_eq (rad)")
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
