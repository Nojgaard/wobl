from __future__ import annotations

import time

import numpy as np
from dm_env import TimeStep

from woblpy.control import pitch_equilibrium
from woblpy.control.motion_controller import MotionController
from woblpy.record import Recorder
from woblpy.sim.robot import Robot


class ControlPolicy:
    def __init__(
        self,
        robot: Robot,
        recorder: Recorder | None = None,
        dt: float = 0.01,
    ) -> None:
        self.controller = MotionController(robot)
        self._recorder = recorder
        self._dt = dt
        self._last_print_time = time.time()
        self._robot = robot

    def __call__(self, timestep: TimeStep) -> np.ndarray:
        obs = timestep.observation
        hip_left, hip_right, left, right = self.controller.update(obs, self._dt)

        if self._recorder is not None:
            self._recorder.log_controller(
                obs, self.controller, left, right, t_s=time.time()
            )

        now = time.time()
        if now - self._last_print_time > 0.2:
            print(
                f"Pitch: {self.controller.state.pitch:.3f}, "
                f"Height: {self.controller.state.height:.3f}, "
                f"Pitch Offset: {pitch_equilibrium.from_height(self.controller.state.height):.3f}, "
            )
            self._last_print_time = now

        return np.array([hip_left, hip_right, left, right])
