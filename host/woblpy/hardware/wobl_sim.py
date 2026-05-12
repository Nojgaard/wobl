"""MuJoCo simulation backend implementing the Hardware protocol.

Two operating modes:

  WoblSim()                 — headless, synchronous.  step_drive() advances
                              the physics directly in the calling thread.
                              Useful for headless testing and CI.

  WoblSim(with_viewer=True) — the viewer runs in the main thread via
                              app.launch().  The controller loop must run in
                              a background thread; step_drive() blocks until
                              the sim has completed one physics step after the
                              command is written.

Wheel velocity quantization (0.105 rad/s resolution) mirrors the real encoder
resolution and the noise model in the original sim_node.py.
"""

from __future__ import annotations

import threading
from typing import Optional

import numpy as np
from dm_env import TimeStep

from woblpy.hardware.protocol import (
    DriveCommand,
    DriveTelemetry,
    PoseCommand,
    PoseTelemetry,
    StatusData,
)
from woblpy.sim.application import Application
from woblpy.sim.robot import Robot, RobotWorld

_VELOCITY_RESOLUTION = 0.105  # rad/s — matches real DDSM encoder quantization


class WoblSim:
    """Simulation hardware backend implementing the Hardware protocol."""

    def __init__(self, with_viewer: bool = False) -> None:
        self._with_viewer = with_viewer
        self._drive_cmd = DriveCommand()
        self._pose_cmd = PoseCommand()
        self._latest_drive_telem = DriveTelemetry()
        self._latest_pose_telem = PoseTelemetry(left_valid=True, right_valid=True)

        if with_viewer:
            self._lock = threading.Lock()
            self._telem_ready = threading.Event()

        robot = Robot()
        world = RobotWorld(robot)
        # 100 Hz control rate matches the drive loop; 200 Hz physics for stability.
        world.set_timesteps(control_timestep=0.010, physics_timestep=0.005)
        self.app = Application(world, self._update)

    # ------------------------------------------------------------------
    # Sim policy callback — called by Application at each control step
    # ------------------------------------------------------------------

    def _update(self, timestep: TimeStep) -> np.ndarray:
        obs = timestep.observation
        orientation = obs["robot/orientation"]       # framequat [x, y, z, w]
        gyr = obs["robot/angular_velocity"]
        joint_pos = obs["robot/joint_positions"]
        joint_vel = obs["robot/joint_velocities"]
        joint_eff = obs["robot/joint_efforts"]

        # Quantize wheel velocities to match real encoder resolution
        left_vel = _quantize(float(joint_vel[2]))
        right_vel = _quantize(float(joint_vel[3]))

        drive_telem = DriveTelemetry(
            quat_xyzw=(
                float(orientation[3]),
                float(orientation[0]),
                float(orientation[1]),
                float(orientation[2]),
            ),
            gyro=(float(gyr[0]), float(gyr[1]), float(gyr[2])),
            left_angle=float(joint_pos[2]),
            left_vel=left_vel,
            right_angle=float(joint_pos[3]),
            right_vel=right_vel,
            timestamp_ms=0,
        )
        pose_telem = PoseTelemetry(
            left_valid=True,
            right_valid=True,
            left_pos=float(joint_pos[0]),
            left_vel=float(joint_vel[0]),
            left_effort=float(joint_eff[0]),
            right_pos=float(joint_pos[1]),
            right_vel=float(joint_vel[1]),
            right_effort=float(joint_eff[1]),
        )

        if self._with_viewer:
            with self._lock:
                self._latest_drive_telem = drive_telem
                self._latest_pose_telem = pose_telem
                drive_cmd = self._drive_cmd
                pose_cmd = self._pose_cmd
            self._telem_ready.set()
        else:
            self._latest_drive_telem = drive_telem
            self._latest_pose_telem = pose_telem
            drive_cmd = self._drive_cmd
            pose_cmd = self._pose_cmd

        return np.array([
            pose_cmd.left_pos_rad,
            pose_cmd.right_pos_rad,
            _quantize(drive_cmd.left_velocity),
            _quantize(drive_cmd.right_velocity),
        ])

    # ------------------------------------------------------------------
    # Hardware protocol
    # ------------------------------------------------------------------

    def step_drive(self, cmd: DriveCommand) -> DriveTelemetry:
        if self._with_viewer:
            self._telem_ready.clear()
            with self._lock:
                self._drive_cmd = cmd
            self._telem_ready.wait(timeout=1.0)
            with self._lock:
                return self._latest_drive_telem
        else:
            self._drive_cmd = cmd
            self.app._step()
            return self._latest_drive_telem

    def step_pose(self, cmd: PoseCommand) -> PoseTelemetry:
        if self._with_viewer:
            with self._lock:
                self._pose_cmd = cmd
                return self._latest_pose_telem
        else:
            self._pose_cmd = cmd
            return self._latest_pose_telem

    def request_status(self) -> StatusData:
        return StatusData(
            imu_status=0,
            imu_rate=100.0,
            foc_rate=100.0,
            wheel_rate=100.0,
            left_servo_ok=True,
            right_servo_ok=True,
        )

    def close(self) -> None:
        self.app.running = False

    def __enter__(self) -> "WoblSim":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


def _quantize(v: float) -> float:
    return round(v / _VELOCITY_RESOLUTION) * _VELOCITY_RESOLUTION
