"""Control loop — replaces the old ControllerNode.

ControllerLoop owns a Hardware backend and a Controller, drives the balancing
loop at *drive_hz* (default 100 Hz), and sends servo pose updates at
*pose_hz* (default 15 Hz).

Usage — real hardware (blocking):
    loop = ControllerLoop(hardware)
    loop.run()          # blocks; Ctrl-C or loop.stop() to exit

Usage — sim with viewer (viewer runs in main thread):
    loop = ControllerLoop(hardware)
    loop.start()        # background thread
    hardware.app.launch()
    loop.stop()

The optional *on_state* callback is called after every drive tick with the
latest telemetry and outgoing command — use it for in-process recording.
"""

from __future__ import annotations

import threading
import time
from typing import Callable, Optional

from woblpy.control.controller import Controller
from woblpy.hardware.protocol import (
    DriveCommand,
    DriveTelemetry,
    Hardware,
    PoseCommand,
    PoseTelemetry,
)


class ControllerLoop:
    def __init__(
        self,
        hardware: Hardware,
        drive_hz: float = 100.0,
        pose_hz: float = 15.0,
        on_state: Optional[
            Callable[[DriveTelemetry, DriveCommand, PoseTelemetry], None]
        ] = None,
    ) -> None:
        self._hardware = hardware
        self._drive_period = 1.0 / drive_hz
        self._pose_every_n = max(1, round(drive_hz / pose_hz))
        self._on_state = on_state

        self._controller = Controller()
        self._drive_cmd = DriveCommand()
        self._pose_cmd = PoseCommand()
        self._latest_pose_telem = PoseTelemetry()

        self._running = False
        self._thread: Optional[threading.Thread] = None

    # ------------------------------------------------------------------
    # Public command setters — thread-safe, for joystick / external input
    # ------------------------------------------------------------------

    def set_velocity_target(self, fwd_velocity: float, yaw_rate: float) -> None:
        """Update the target forward velocity and yaw rate (rad/s)."""
        self._controller.cmd_fwd_velocity.update(fwd_velocity)
        self._controller.cmd_yaw_rate.update(yaw_rate)

    def set_pose(self, left_pos_rad: float, right_pos_rad: float) -> None:
        """Update the servo position targets (rad)."""
        self._pose_cmd = PoseCommand(
            left_enabled=True,
            left_pos_rad=left_pos_rad,
            right_enabled=True,
            right_pos_rad=right_pos_rad,
        )

    # ------------------------------------------------------------------
    # Internal loop
    # ------------------------------------------------------------------

    def _loop(self) -> None:
        tick = 0
        next_time = time.monotonic()

        while self._running:
            # --- Drive tick (every iteration) ---
            drive_telem = self._hardware.step_drive(self._drive_cmd)
            self._controller.update_drive_telem(drive_telem)
            self._drive_cmd = self._controller.update()

            # --- Pose tick (every N-th iteration) ---
            if tick % self._pose_every_n == 0:
                self._latest_pose_telem = self._hardware.step_pose(self._pose_cmd)

            if self._on_state is not None:
                self._on_state(drive_telem, self._drive_cmd, self._latest_pose_telem)

            tick += 1
            next_time += self._drive_period
            sleep_time = next_time - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # Fell behind schedule — reset rather than catching up in a burst.
                next_time = time.monotonic()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def run(self) -> None:
        """Block the calling thread until stop() is called."""
        self._running = True
        self._loop()

    def start(self) -> None:
        """Run the loop in a daemon background thread."""
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Signal the loop to stop and wait for it to finish."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
