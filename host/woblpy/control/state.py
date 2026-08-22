"""State observer, reading the robot state straight from the observation dict."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any, Self

import numpy as np
from scipy.spatial.transform import Rotation

from woblpy.control.leg_kinematics import LegKinematics


@dataclass
class State:
    """Robot state derived from a policy observation."""

    _leg_ik: LegKinematics
    roll: float = 0.0
    pitch: float = 0.0
    roll_rate: float = 0.0
    pitch_rate: float = 0.0
    forward_velocity: float = 0.0
    turn_velocity: float = 0.0
    height: float = 0.0

    _WHEEL_BASE = 0.3  # meters
    _WHEEL_RADIUS = 0.04  # meters

    def update(self, obs: Mapping[str, Any], dt: float) -> Self:
        orientation = np.asarray(obs["robot/orientation"], dtype=float)
        if not orientation.any():  # zero quaternion on the first frame
            return self

        self.roll, self.pitch, _ = Rotation.from_quat(
            orientation, scalar_first=True
        ).as_euler("xyz")

        gyro = obs["robot/angular_velocity"]
        joint_pos = obs["robot/joint_positions"]
        joint_vel = obs["robot/joint_velocities"]
        left_velocity = joint_vel[2]
        right_velocity = joint_vel[3]

        leg_heights = [self._leg_ik.to_height(p) for p in joint_pos[0:2]]

        self.roll_rate = gyro[0]
        self.pitch_rate = gyro[1]
        self.forward_velocity = (
            (left_velocity + right_velocity) / 2.0 * self._WHEEL_RADIUS
        )
        self.turn_velocity = (
            (right_velocity - left_velocity) / self._WHEEL_BASE * self._WHEEL_RADIUS
        )
        self.height = (leg_heights[0] + leg_heights[1]) / 2.0
        return self
