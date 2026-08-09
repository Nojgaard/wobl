"""State observer, reading the robot state straight from the observation dict.

The sim policy hands the controller the raw dm_control observation dict
(framequat, gyro, joint velocities).  The observer turns that directly into the
filtered state the balance controller operates on: roll, pitch, filtered body
rates, and forward/turn velocity from the wheel speeds.

Wheel velocities are quantized to the real DDSM encoder resolution before
filtering; the filters themselves default to passthrough (Tf = 0) like the
firmware observer.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

from woblpy.control.low_pass_filter import LowPassFilter

_VELOCITY_RESOLUTION = 0.105  # rad/s — real DDSM encoder quantization


def _quantize(v: float) -> float:
    return round(v / _VELOCITY_RESOLUTION) * _VELOCITY_RESOLUTION


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
        # Tf = 0 → passthrough, matching the firmware observer defaults.
        self._roll_rate = LowPassFilter(0.0)
        self._pitch_rate = LowPassFilter(0.0)
        self._left_wheel_velocity = LowPassFilter(0.0)
        self._right_wheel_velocity = LowPassFilter(0.0)
        self._state = self.State()

    def update(self, obs: Mapping[str, Any], dt: float) -> State:
        """Update the state estimate from a policy observation dict."""
        orientation = np.asarray(obs["robot/orientation"], dtype=float)
        if not orientation.any():  # zero quaternion on the first frame
            return self._state

        roll, pitch, _ = Rotation.from_quat(orientation, scalar_first=True).as_euler(
            "xyz"
        )

        gyro = obs["robot/angular_velocity"]
        wheel_vel = obs["robot/joint_velocities"]
        lv = self._left_wheel_velocity(_quantize(wheel_vel[2]), dt)
        rv = self._right_wheel_velocity(_quantize(wheel_vel[3]), dt)

        self._state = self.State(
            roll=roll,
            pitch=pitch,
            roll_rate=self._roll_rate(gyro[0], dt),
            pitch_rate=self._pitch_rate(gyro[1], dt),
            forward_velocity=(lv + rv) / 2.0 * self._WHEEL_RADIUS,
            turn_velocity=(rv - lv) / self._WHEEL_BASE * self._WHEEL_RADIUS,
        )
        return self._state
