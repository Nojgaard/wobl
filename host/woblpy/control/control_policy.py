from __future__ import annotations

import time

import numpy as np
from dm_env import TimeStep

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
            state = self.controller.state
            mean_height = (state.left_height + state.right_height) / 2.0
            print(
                f"Pitch: {state.pitch:.3f}, "
                f"Roll: {state.roll:.3f}, "
                f"Height: {mean_height:.3f}, "
                f"Vel:  {state.forward_velocity:.3f}, "
                f"Turn: {state.turn_velocity:.3f}, "
            )
            self._last_print_time = now

        return np.array([hip_left, hip_right, left, right])
