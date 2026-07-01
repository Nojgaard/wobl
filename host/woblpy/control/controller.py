import numpy as np

from woblpy.control.diff_drive_kinematics import DiffDriveKinematics
from woblpy.control.kalman_filter import KalmanFilter
from woblpy.control.linear_filter import LinearFilter
from woblpy.control.lqr import compute_lqr_gains
from woblpy.hardware.protocol import DriveCommand, DriveTelemetry


class Controller:
    def __init__(self):
        # self._k = np.array([-7.70647133, -0.87846039, 2.61800094, 1.41421356])
        self._k = compute_lqr_gains()
        self.integral_error = 0.0
        # self.offset_pitch = 0.0313
        self.offset_pitch = 0.07
        # self.offset_pitch = 0.04
        self._dt: float = 0.01  # seconds; set each tick from telemetry timestamps
        self._last_telem_ms: int | None = (
            None  # firmware timestamp of previous telemetry packet
        )

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.roll_rate = LinearFilter(0.8, 0.0)
        self.pitch_rate = LinearFilter(0.9, 0.0)
        # self.pitch_rate = KalmanFilter(0.1, 0.02)

        self.yaw_rate = LinearFilter(0.5, 0.0)
        # q=1.0, r=0.25 matches plot_bench.py defaults (validated on bench data)
        self.fwd_velocity = KalmanFilter(2.0, 0.25)
        self.fwd_velocity_raw = 0.0  # for logging/debugging only; not used in control
        # self.fwd_velocity = LinearFilter(0.05, 0.0)

        self.cmd_fwd_velocity = LinearFilter(0.2, 0.0)
        self.cmd_yaw_rate = LinearFilter(0.2, 0.0)

        self.ctrl_velocity = 0.0
        self.ctrl_yaw_rate = 0.0

        self.last_left_vel = 0.0
        self.last_right_vel = 0.0
        self.max_vel_rate = 40.0  # rad/s² - tune this based on testing

        self.diff_drive = DiffDriveKinematics(0.3, 0.04, 20.0)

    def update_drive_telem(self, telem: DriveTelemetry) -> None:
        if sum(telem.quat_xyzw) == 0.0:
            return

        self.rpy = telem.orientation_euler()
        self.roll, self.pitch, self.yaw = self.rpy
        self.roll_rate.update(telem.gyro[0])
        self.pitch_rate.update(telem.gyro[1])

        fwd_velocity, yaw_rate = self.diff_drive.forward_kinematics(
            telem.left_vel, telem.right_vel
        )
        prev_ms, self._last_telem_ms = self._last_telem_ms, telem.timestamp_ms
        dt_ms = (telem.timestamp_ms - prev_ms) if prev_ms is not None else 20
        self._dt = dt_ms / 1000.0 if 0 < dt_ms <= 500 else 0.01
        self.fwd_velocity.update(fwd_velocity, dt=self._dt)
        self.fwd_velocity_raw = fwd_velocity
        self.yaw_rate.update(yaw_rate)

    def update(self) -> DriveCommand:
        k_pitch = self._k[0]
        k_pitch_rate = self._k[1]
        k_velocity = self._k[2]  # K[2] = velocity gain (from lqr state: v)
        k_position = self._k[3]  # K[3] = integral gain (from lqr integral_action on v)

        cmd_fwd_velocity = self.cmd_fwd_velocity.value
        cmd_yaw_rate = self.cmd_yaw_rate.value

        pitch = self.pitch - self.offset_pitch
        pitch_rate = self.pitch_rate.value

        fwd_velocity = self.fwd_velocity.value - cmd_fwd_velocity

        self.integral_error += fwd_velocity * self._dt
        self.integral_error = np.clip(self.integral_error, -0.3, 0.3)

        ctrl_vel = (
            -k_pitch * pitch
            - k_pitch_rate * pitch_rate
            - k_velocity * fwd_velocity
            - k_position * self.integral_error
        )

        # print(f"torqe: {ctrl_vel / 0.059} A")
        # torque_a = ctrl_vel / 0.059  # Convert desired wheel velocity to torque (A) using motor constant
        # Desired velocity commands
        ctrl_yaw_rate = cmd_yaw_rate

        ctrl_left_rps, ctrl_right_rps = self.diff_drive.inverse_kinematics(
            ctrl_vel, ctrl_yaw_rate
        )
        # torque_a = np.clip(torque_a, -0.5, 0.5)  # Limit torque to ±0.5 A for safety
        return DriveCommand(
            left_enabled=True,
            left_velocity=float(ctrl_left_rps),
            right_enabled=True,
            right_velocity=float(ctrl_right_rps),
        )
