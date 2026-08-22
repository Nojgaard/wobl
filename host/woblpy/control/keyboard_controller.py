from __future__ import annotations

import pynput

from woblpy.control.control_policy import ControlPolicy


class KeyboardController:
    def __init__(
        self,
        policy: ControlPolicy,
        max_fwd: float = 0.3,
        max_yaw: float = 1.0,
        height_step: float = 0.005,
    ) -> None:
        self._policy = policy
        self._max_fwd = max_fwd
        self._max_yaw = max_yaw
        self._height_step = height_step
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

        target = self._policy.controller.target
        if key.name == "up":
            target.forward_velocity = self._max_fwd
        elif key.name == "down":
            target.forward_velocity = -self._max_fwd
        elif key.name == "left":
            target.turn_velocity = -self._max_yaw
        elif key.name == "right":
            target.turn_velocity = self._max_yaw
        elif key.name == "page_up":
            target.height += self._height_step
        elif key.name == "page_down":
            target.height -= self._height_step

    def _on_release(self, key: pynput.keyboard.Key | pynput.keyboard.KeyCode | None):
        if key is None or isinstance(key, pynput.keyboard.KeyCode):
            return

        target = self._policy.controller.target
        if key.name in ["up", "down"]:
            target.forward_velocity = 0
        elif key.name in ["left", "right"]:
            target.turn_velocity = 0
