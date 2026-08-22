from __future__ import annotations

from collections.abc import Mapping
from typing import Any

import numpy as np
from simple_pid import PID

from woblpy.control import pitch_equilibrium
from woblpy.control.leg_kinematics import LegKinematics
from woblpy.control.lqr import compute_lqr_gains
from woblpy.control.state import State
from woblpy.control.target_state import TargetState
from woblpy.sim.robot import Robot


class MotionController:
    def __init__(self, robot: Robot) -> None:
        gains = compute_lqr_gains()
        self._pitch_kp = float(gains[0])
        self._pitch_rate_kp = float(gains[1])
        self._velocity_kp = float(gains[2])
        self._position_kp = float(gains[3])

        self._leg_ik = LegKinematics(robot.leg_keypoints)
        self.target = TargetState(self._leg_ik)

        self._pid = PID(Kp=1.0, Ki=0.1, Kd=0.05, setpoint=0.0)

        self._position_error = 0.0
        self.state = State(self._leg_ik)

    def update(
        self, obs: Mapping[str, Any], dt: float
    ) -> tuple[float, float, float, float]:
        self.state.update(obs, dt)
        ctrl_left, ctrl_right = self._balance(self.state, dt)
        hip_angle = float(self._leg_ik.to_angle(self.target.height))
        return hip_angle, hip_angle, ctrl_left, ctrl_right

    def _balance(self, state: State, dt: float) -> tuple[float, float]:
        self._pitch_offset = pitch_equilibrium.from_height(state.height)
        pitch_error = state.pitch - self._pitch_offset
        velocity_error = state.forward_velocity - self.target.forward_velocity

        self._position_error += velocity_error * dt
        self._position_error = float(np.clip(self._position_error, -0.1, 0.1))

        ctrl_fwd_vel = (
            -self._pitch_kp * pitch_error
            - self._pitch_rate_kp * state.pitch_rate
            - self._position_kp * self._position_error
            - self._velocity_kp * velocity_error
        )

        ctrl_left = ctrl_fwd_vel + self.target.turn_velocity
        ctrl_right = ctrl_fwd_vel - self.target.turn_velocity
        return ctrl_left, ctrl_right
