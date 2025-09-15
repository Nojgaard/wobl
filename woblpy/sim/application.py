import time
from enum import Enum
from typing import Callable, cast

import dm_env
import mujoco.viewer
import numpy.typing as npt
from dm_control.composer import Environment, Task
from dm_control.mujoco import Physics
from dm_env import TimeStep


class State(Enum):
    RUNNING = 0
    RESET = 1
    STOPPED = 2


class Application:
    def __init__(self, task: Task, policy: Callable[[TimeStep], npt.ArrayLike]):
        self._task = task
        self._env = Environment(task, strip_singleton_obs_buffer_dim=True)
        self._step_data = self._env.reset()
        self._timestep = task.control_timestep
        self._state = State.RUNNING
        self._policy = policy
        self._physics: Physics = cast(Physics, self._env.physics)
        self.running = True

    def _step(self):
        match self._state:
            case State.RUNNING:
                action = self._policy(self._step_data)
                self._step_data = self._env.step(action)
            case State.RESET:
                self._state = State.RUNNING
                self._step_data = self._restart()
            case State.STOPPED:
                self._physics.forward()

    def _restart(self):
        self._physics.reset()
        self._env._hooks.initialize_episode(
            self._env._physics_proxy, self._env._random_state
        )
        self._env._observation_updater.reset(
            self._env._physics_proxy, self._env._random_state
        )
        self._physics.forward()

        return dm_env.TimeStep(
            step_type=dm_env.StepType.FIRST,
            reward=None,
            discount=None,
            observation=self._env._observation_updater.get_observation(),
        )

    def _key_callback(self, key: int):
        # print(key, chr(key))
        if ord("R") == key:
            self._state = State.RESET
        elif ord(" ") == key:
            self._state = (
                State.RUNNING if self._state == State.STOPPED else State.STOPPED
            )

    def launch_headless(self):
        """Launch the application in headless mode."""
        while True:
            step_start = time.time()
            self._step()

            time_until_next_step = self._timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    def launch(self):
        physics = self._physics
        with mujoco.viewer.launch_passive(
            physics.model.ptr, physics.data.ptr, key_callback=self._key_callback
        ) as viewer:
            sync_interval = 1.0 / 30  # 30 times per second
            last_sync_time = time.monotonic()
            while self.running and viewer.is_running():
                step_start = time.monotonic()
                self._step()

                if step_start - last_sync_time >= sync_interval:
                    viewer.sync()
                    last_sync_time = step_start

                time_until_next_step = self._timestep - (time.monotonic() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
        self._env.close()


if __name__ == "__main__":
    import numpy as np

    from woblpy.sim.robot import Robot, RobotWorld

    # Frequency tracking variables
    call_times = 0
    last_print_time = time.time()
    print_interval = 1.0  # Print frequency every 1 second

    def dummy_policy(timestep: TimeStep) -> npt.ArrayLike:
        global call_times, last_print_time, print_interval

        current_time = time.time()
        call_times += 1

        # Print frequency every print_interval seconds
        if current_time - last_print_time >= print_interval:
            elapsed = current_time - last_print_time
            frequency = call_times / elapsed if elapsed > 0 else 0.0
            print(f"dummy_policy callback frequency: {frequency:.1f} Hz")
            last_print_time = current_time
            call_times = 0

        return np.zeros(4)

    robot = Robot()
    world = RobotWorld(robot)
    app = Application(world, dummy_policy)
    app.launch()
