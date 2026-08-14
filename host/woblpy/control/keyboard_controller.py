from __future__ import annotations

import pynput

from woblpy.control.control_policy import ControlPolicy


class KeyboardController:
    def __init__(
        self,
        policy: ControlPolicy,
        max_fwd: float = 0.3,
        max_yaw: float = 1.0,
    ) -> None:
        self._policy = policy
        self._max_fwd = max_fwd
        self._max_yaw = max_yaw
        self._cur_fwd = 0
        self._cur_yaw = 0
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

        if key.name == "up":
            self._policy.target_fwd = self._max_fwd
        elif key.name == "down":
            self._policy.target_fwd = -self._max_fwd
        elif key.name == "left":
            self._policy.target_yaw = -self._max_yaw
        elif key.name == "right":
            self._policy.target_yaw = self._max_yaw

    def _on_release(self, key: pynput.keyboard.Key | pynput.keyboard.KeyCode | None):
        if key is None or isinstance(key, pynput.keyboard.KeyCode):
            return

        if key.name in ["up", "down"]:
            self._policy.target_fwd = 0
        elif key.name in ["left", "right"]:
            self._policy.target_yaw = 0
