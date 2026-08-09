from __future__ import annotations

import pynput

from woblpy.control.control_policy import ControlPolicy


class KeyboardController:
    def __init__(
        self,
        policy: ControlPolicy,
        max_fwd: float = 1.0,
        max_yaw: float = 1.0,
    ) -> None:
        self._policy = policy
        self._max_fwd = max_fwd
        self._max_yaw = max_yaw
        self._listener: pynput.keyboard.Listener | None = None

    def start(self) -> None:
        import pynput.keyboard

        self._listener = pynput.keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release,
        )
        self._listener.start()

    def stop(self) -> None:
        if self._listener is not None:
            self._listener.stop()

    def _on_press(self, key: pynput.keyboard.Key | pynput.keyboard.KeyCode | None):
        if key is None:
            return

        if isinstance(key, pynput.keyboard.KeyCode):
            return  # ignore non-special keys

        if key.name == "up":  # UP
            self._policy.set_velocity_target(self._max_fwd, 0.0)
        elif key.name == "down":  # DOWN
            self._policy.set_velocity_target(-self._max_fwd, 0.0)
        elif key.name == "left":  # LEFT
            self._policy.set_velocity_target(0.0, self._max_yaw)
        elif key.name == "right":  # RIGHT
            self._policy.set_velocity_target(0.0, -self._max_yaw)

    def _on_release(self, key: pynput.keyboard.Key | pynput.keyboard.KeyCode | None):
        self._policy.reset_velocity_target()
