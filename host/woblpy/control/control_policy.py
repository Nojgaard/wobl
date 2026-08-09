"""Policy glue between the MuJoCo sim and the Controller.

ControlPolicy is the dm_control "policy" callable: each time the Application
steps the physics it queries the observation, feeds it into the Controller,
and returns the wheel action vector.  The control loop runs entirely on the
main thread (no daemon control thread).

The keyboard listener (see woblpy.control.keyboard_controller) writes plain
velocity targets via set_velocity_target()/reset_velocity_target(); they are
applied to the Controller on the next main-thread tick, keeping the Controller
single-threaded.

Wheel velocity quantization (0.105 rad/s resolution) mirrors the real encoder
resolution (see the original sim_node.py noise model).
"""

from __future__ import annotations

import time

import numpy as np
from dm_env import TimeStep

from woblpy.control.controller import Controller
from woblpy.control.datatypes import DriveTelemetry
from woblpy.record import Recorder

_VELOCITY_RESOLUTION = 0.105  # rad/s — matches real DDSM encoder quantization


class ControlPolicy:
    """dm_control policy: observation -> Controller -> wheel action."""

    def __init__(
        self,
        controller: Controller | None = None,
        recorder: Recorder | None = None,
        hip_pos: float = 0.1,
    ) -> None:
        self._controller = controller if controller is not None else Controller()
        self._recorder = recorder
        self._hip_pos = hip_pos

        self._target_fwd = 0.0
        self._target_yaw = 0.0
        self._reset_requested = False

    def set_velocity_target(self, fwd_velocity: float, yaw_rate: float) -> None:
        """Update the target forward velocity and yaw rate (rad/s)."""
        self._target_fwd = fwd_velocity
        self._target_yaw = yaw_rate

    def reset_velocity_target(self) -> None:
        """Zero the targets; the Controller state resets on the next tick."""
        self._target_fwd = 0.0
        self._target_yaw = 0.0
        self._reset_requested = True

    def __call__(self, timestep: TimeStep) -> np.ndarray:
        obs = timestep.observation

        self._apply_target()
        telem = self._telem_from_obs(obs)
        self._controller.update_drive_telem(telem)
        cmd = self._controller.update()
        cmd.left_enabled = True
        cmd.right_enabled = True

        if self._recorder is not None:
            self._recorder.log_controller(telem, cmd, self._controller)

        return np.array(
            [
                self._hip_pos,
                self._hip_pos,
                _quantize(cmd.left_velocity),
                _quantize(cmd.right_velocity),
            ]
        )

    def _apply_target(self) -> None:
        if self._reset_requested:
            self._controller.cmd_fwd_velocity.value = 0.0
            self._controller.cmd_yaw_rate.value = 0.0
            self._controller.integral_error = 0.0
            self._reset_requested = False
        self._controller.cmd_fwd_velocity.update(self._target_fwd)
        self._controller.cmd_yaw_rate.update(self._target_yaw)

    def _telem_from_obs(self, obs) -> DriveTelemetry:
        orientation = obs["robot/orientation"]  # framequat [x, y, z, w]
        gyr = obs["robot/angular_velocity"]
        joint_pos = obs["robot/joint_positions"]
        joint_vel = obs["robot/joint_velocities"]

        return DriveTelemetry(
            quat_xyzw=(
                float(orientation[3]),
                float(orientation[0]),
                float(orientation[1]),
                float(orientation[2]),
            ),
            gyro=(float(gyr[0]), float(gyr[1]), float(gyr[2])),
            left_angle=float(joint_pos[2]),
            left_vel=_quantize(float(joint_vel[2])),
            right_angle=float(joint_pos[3]),
            right_vel=_quantize(float(joint_vel[3])),
            timestamp_ms=int(time.time() * 1000.0),
        )


def _quantize(v: float) -> float:
    return round(v / _VELOCITY_RESOLUTION) * _VELOCITY_RESOLUTION
