import time

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
        self.offset_pitch = 0.0313
        # self.offset_pitch = 0.04
        self.last_time = time.monotonic()
        self._last_telem_ms: int | None = None  # firmware timestamp of previous telemetry packet

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.roll_rate = LinearFilter(0.5, 0.0)
        self.pitch_rate = LinearFilter(0.5, 0.0)
        # self.pitch_rate = KalmanFilter(0.1, 0.02)

        self.yaw_rate = LinearFilter(0.5, 0.0)
        # q=1.0, r=0.25 matches plot_bench.py defaults (validated on bench data)
        self.fwd_velocity = KalmanFilter(1.0, 0.25)
        # self.fwd_velocity = LinearFilter(0.2, 0.0)

        self.cmd_fwd_velocity = LinearFilter(0.2, 0.0)
        self.cmd_yaw_rate = LinearFilter(0.2, 0.0)

        self.ctrl_velocity = 0.0
        self.ctrl_yaw_rate = 0.0

        self.last_left_torque = 0.0
        self.last_right_torque = 0.0
        self.max_torque_rate = 40.0  # Nm/s - tune this based on testing

        self.diff_drive = DiffDriveKinematics(0.3, 0.04, 10.0)

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
        telem_dt = dt_ms / 1000.0 if 0 < dt_ms <= 500 else 0.02
        # No tau/target here: in balance mode the firmware runs torque control,
        # so velocity is not closed-loop driven toward a target (unlike wheel_tune
        # where the velocity PID makes tau≈20ms valid for plot_bench.py).
        self.fwd_velocity.update(fwd_velocity, dt=telem_dt)
        self.yaw_rate.update(yaw_rate)

    def update_dt(self):
        now = time.monotonic()
        dt = now - self.last_time
        self.last_time = now
        if dt <= 0 or dt > 0.5:
            dt = 0.02
        return dt

    def update_velocity(self, v, v_target, dt, a_max, k=2.0):
        # Compute desired acceleration (spring-like toward target)
        a = k * (v_target - v)

        # Clamp acceleration to max limits
        if a > a_max:
            a = a_max
        elif a < -a_max:
            a = -a_max

        # Update velocity
        v_new = v + a * dt
        return v_new

    def update(self) -> DriveCommand:
        k_pitch = self._k[0]
        k_pitch_rate = self._k[1]
        k_position = self._k[2]
        k_velocity = self._k[3]

        cmd_fwd_velocity = self.cmd_fwd_velocity.value
        cmd_yaw_rate = self.cmd_yaw_rate.value

        pitch = self.pitch - self.offset_pitch
        pitch_rate = self.pitch_rate.value

        fwd_velocity = self.fwd_velocity.value - cmd_fwd_velocity

        dt = self.update_dt()
        self.integral_error += fwd_velocity * dt
        self.integral_error = np.clip(self.integral_error, -0.5, 0.5)

        # self.pitch_rate.update((self.pitch - last_pitch) / dt)
        # pitch_rate = self.pitch_rate.value
        # pitch_rate = 0

        ctrl_torque = -(
            k_pitch * pitch
            + k_pitch_rate * pitch_rate
            + k_velocity * fwd_velocity
            + k_position * self.integral_error
        )
        ctrl_yaw_torque = cmd_yaw_rate * 0.5

        # Add deadband near equilibrium to reduce chatter
        # if abs(pitch) < 0.02:
        #    ctrl_torque *= 0.2  # Reduce gain significantly near equilibrium

        """ctrl_fwd_velocity = self.update_velocity(
            self.last_left_torque, ctrl_torque, dt, a_max=50.0, k=2.0
        )
        self.last_left_torque = ctrl_fwd_velocity

        ctrl_left_rps, ctrl_right_rps = self.diff_drive.inverse_kinematics(
            ctrl_fwd_velocity, ctrl_yaw_torque
        )"""

        # Desired torques
        left_torque = ctrl_torque + ctrl_yaw_torque
        right_torque = ctrl_torque - ctrl_yaw_torque

        # Apply torque rate limiting
        max_delta = self.max_torque_rate * dt

        left_torque = np.clip(
            left_torque,
            self.last_left_torque - max_delta,
            self.last_left_torque + max_delta,
        )
        right_torque = np.clip(
            right_torque,
            self.last_right_torque - max_delta,
            self.last_right_torque + max_delta,
        )

        # Store for next iteration
        self.last_left_torque = left_torque
        self.last_right_torque = right_torque

        # Clamp to motor limits
        left_torque = np.clip(left_torque, -1.96, 1.96)
        right_torque = np.clip(right_torque, -1.96, 1.96)

        return DriveCommand(
            left_enabled=True,
            left_velocity=float(left_torque),
            right_enabled=True,
            right_velocity=float(right_torque),
        )
