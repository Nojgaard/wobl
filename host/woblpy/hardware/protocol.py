"""Hardware-agnostic data model.

All units follow the firmware convention: radians, rad/s, percent, milliseconds.
Quaternion order is always (w, x, y, z).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol, runtime_checkable

from scipy.spatial.transform import Rotation as R


@dataclass
class DriveCommand:
    """Command sent as MSG_CMD_DRIVE.  velocity is in rad/s (or torque units
    for current-controlled wheels — the wire field name is shared)."""

    left_enabled: bool = True
    left_velocity: float = 0.0
    right_enabled: bool = True
    right_velocity: float = 0.0


@dataclass
class DriveTelemetry:
    """Telemetry received as MSG_TELEM_DRIVE."""

    quat_xyzw: tuple[float, float, float, float] = field(
        default_factory=lambda: (0.0, 0.0, 0.0, 1.0)
    )
    gyro: tuple[float, float, float] = field(default_factory=lambda: (0.0, 0.0, 0.0))
    left_angle: float = 0.0
    left_vel: float = 0.0
    left_current: float = 0.0
    right_angle: float = 0.0
    right_vel: float = 0.0
    right_current: float = 0.0
    timestamp_ms: int = 0

    def orientation_euler(self) -> tuple[float, float, float]:
        """Return fixed-axis roll, pitch, yaw in radians from the quaternion."""
        return R.from_quat(self.quat_xyzw).as_euler("xyz")


@dataclass
class PoseCommand:
    """Command sent as MSG_CMD_POSE."""

    left_enabled: bool = True
    left_pos_rad: float = 0.0
    right_enabled: bool = True
    right_pos_rad: float = 0.0


@dataclass
class PoseTelemetry:
    """Telemetry received as MSG_TELEM_POSE."""

    left_valid: bool = False
    right_valid: bool = False
    left_pos: float = 0.0
    left_vel: float = 0.0
    left_effort: float = 0.0
    right_pos: float = 0.0
    right_vel: float = 0.0
    right_effort: float = 0.0


@dataclass
class StatusData:
    """Status received as MSG_STATUS."""

    imu_status: int = 0
    imu_rate: float = 0.0
    foc_rate: float = 0.0
    wheel_rate: float = 0.0
    left_wheel_status: int = 0
    right_wheel_status: int = 0
    left_servo_ok: bool = False
    right_servo_ok: bool = False


@runtime_checkable
class Hardware(Protocol):
    """Protocol implemented by both WoblSerial and WoblSim."""

    def step_drive(self, cmd: DriveCommand) -> DriveTelemetry:
        """Send a drive command and return the resulting telemetry."""
        ...

    def step_pose(self, cmd: PoseCommand) -> PoseTelemetry:
        """Send a pose command and return the resulting telemetry."""
        ...

    def request_status(self) -> StatusData:
        """Request and return a hardware status snapshot."""
        ...

    def close(self) -> None:
        """Release resources."""
        ...
