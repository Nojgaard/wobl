from __future__ import annotations

import time

import numpy as np
from dm_env import TimeStep

from woblpy.control.motion_controller import MotionController
from woblpy.record import Recorder


class ControlPolicy:
    def __init__(
        self,
        recorder: Recorder | None = None,
        hip_pos: float = 0.1,
        dt: float = 0.01,
    ) -> None:
        self._controller = MotionController()
        self._recorder = recorder
        self._hip_pos = hip_pos
        self._dt = dt

        self.target_fwd = 0.0
        self.target_yaw = 0.0

    def __call__(self, timestep: TimeStep) -> np.ndarray:
        obs = timestep.observation
        self._controller.forward_velocity = self.target_fwd
        self._controller.turn_velocity = self.target_yaw
        left, right = self._controller.update(obs, self._dt)

        if self._recorder is not None:
            self._recorder.log_controller(
                obs, self._controller, left, right, t_s=time.time()
            )

        return np.array([self._hip_pos, self._hip_pos, left, right])
