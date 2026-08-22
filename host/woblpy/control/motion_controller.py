from __future__ import annotations

from collections.abc import Mapping
from typing import Any

import numpy as np

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

        self._leg_ik = LegKinematics(robot.leg_keypoints, robot.servo_limits())
        self.target = TargetState(self._leg_ik)

        self._roll_kp = 0.15
        self._roll_kd = 0.01

        self._yaw_rate_kp = 0.5
        self._yaw_rate_ki = 0.5
        self._yaw_rate_integral = 0.0

        self._position_error = 0.0
        self.state = State(self._leg_ik)

        self._wheel_seperation = robot.wheel_seperation()

    def update(
        self, obs: Mapping[str, Any], dt: float
    ) -> tuple[float, float, float, float]:
        self.state.update(obs, dt)
        self.target.update(dt)

        balance_left, balance_right = self._balance(self.state, dt)
        turn_left, turn_right = self._turn(self.state, dt)
        left_wheel = balance_left + turn_left
        right_wheel = balance_right + turn_right

        left_hip, right_hip = self._pose(dt)
        return left_hip, right_hip, left_wheel, right_wheel

    def _pose(self, dt: float) -> tuple[float, float]:
        state = self.state

        hff = self._wheel_seperation * np.sin(self.target.roll_command)

        roll_error = self.target.roll_command - state.roll
        dh = hff + self._roll_kp * roll_error - self._roll_kd * state.roll_rate
        left_leg_height = self.target.height + 0.5 * dh
        right_leg_height = self.target.height - 0.5 * dh

        left_angle = self._leg_ik.to_angle(left_leg_height)
        right_angle = self._leg_ik.to_angle(right_leg_height)
        return (left_angle, right_angle)

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

        return ctrl_fwd_vel, ctrl_fwd_vel

    def _turn(self, state: State, dt: float) -> tuple[float, float]:
        yaw_rate_error = self.target.turn_velocity - state.turn_velocity
        self._yaw_rate_integral += yaw_rate_error * dt
        self._yaw_rate_integral = float(
            np.clip(
                self._yaw_rate_integral,
                -0.05,
                0.05,
            )
        )
        ctrl_turn = (
            self._yaw_rate_kp * yaw_rate_error
            + self._yaw_rate_ki * self._yaw_rate_integral
        )

        return -ctrl_turn, ctrl_turn
