"""State observer, reading the robot state straight from the observation dict."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

from woblpy.control.low_pass_filter import LowPassFilter


class Observer:
    """Turns a policy observation dict into the state used by the controller."""

    _WHEEL_BASE = 0.3  # meters
    _WHEEL_RADIUS = 0.04  # meters

    @dataclass
    class State:
        """Filtered robot state."""

        roll: float = 0.0
        pitch: float = 0.0
        roll_rate: float = 0.0
        pitch_rate: float = 0.0
        forward_velocity: float = 0.0
        turn_velocity: float = 0.0

    def __init__(self) -> None:
        self._roll_rate = LowPassFilter(0.0)
        self._pitch_rate = LowPassFilter(0.0)
        self._left_wheel_velocity = LowPassFilter(0.0)
        self._right_wheel_velocity = LowPassFilter(0.0)
        self._state = self.State()

    def update(self, obs: Mapping[str, Any], dt: float) -> State:
        orientation = np.asarray(obs["robot/orientation"], dtype=float)
        if not orientation.any():  # zero quaternion on the first frame
            return self._state

        roll, pitch, _ = Rotation.from_quat(orientation, scalar_first=True).as_euler(
            "xyz"
        )

        gyro = obs["robot/angular_velocity"]
        wheel_vel = obs["robot/joint_velocities"]
        lv = self._left_wheel_velocity(wheel_vel[2], dt)
        rv = self._right_wheel_velocity(wheel_vel[3], dt)

        self._state = self.State(
            roll=roll,
            pitch=pitch,
            roll_rate=self._roll_rate(gyro[0], dt),
            pitch_rate=self._pitch_rate(gyro[1], dt),
            forward_velocity=(lv + rv) / 2.0 * self._WHEEL_RADIUS,
            turn_velocity=(rv - lv) / self._WHEEL_BASE * self._WHEEL_RADIUS,
        )
        return self._state
