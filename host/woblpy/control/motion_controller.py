from __future__ import annotations

from collections.abc import Mapping
from typing import Any

import numpy as np

from woblpy.control.lqr import compute_lqr_gains
from woblpy.control.observer import Observer


class MotionController:
    def __init__(self, pitch_offset: float = 0.07) -> None:
        gains = compute_lqr_gains()
        self._pitch_kp = float(gains[0])
        self._pitch_rate_kp = float(gains[1])
        self._velocity_kp = float(gains[2])
        self._position_kp = float(gains[3])

        self._pitch_offset = pitch_offset

        self.observer = Observer()
        self._position_error = 0.0

        self.forward_velocity = 0.0  # target forward velocity (m/s)
        self.turn_velocity = 0.0  # target turn velocity (rad/s)
        self.state = Observer.State()  # latest observer state

    def update(self, obs: Mapping[str, Any], dt: float) -> tuple[float, float]:
        self.state = self.observer.update(obs, dt)
        return self._balance(self.state, dt)

    def _balance(self, state: Observer.State, dt: float) -> tuple[float, float]:
        pitch_error = state.pitch - self._pitch_offset
        velocity_error = state.forward_velocity - self.forward_velocity

        self._position_error += velocity_error * dt
        self._position_error = float(np.clip(self._position_error, -0.1, 0.1))

        ctrl_fwd_vel = (
            -self._pitch_kp * pitch_error
            - self._pitch_rate_kp * state.pitch_rate
            - self._position_kp * self._position_error
            - self._velocity_kp * velocity_error
        )

        ctrl_left = ctrl_fwd_vel + self.turn_velocity
        ctrl_right = ctrl_fwd_vel - self.turn_velocity
        return ctrl_left, ctrl_right
