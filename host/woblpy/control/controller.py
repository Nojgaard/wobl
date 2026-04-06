import time

import numpy as np
from scipy.spatial.transform import Rotation as R

from woblpy.control.diff_drive_kinematics import DiffDriveKinematics
from woblpy.control.kalman_filter import KalmanFilter
from woblpy.control.linear_filter import LinearFilter
from woblpy.control.lqr import compute_lqr_gains
from woblpy.messages.messages_pb2 import Imu, JointCommand, JointState


class Controller:
    def __init__(self):
        # self._k = np.array([-7.70647133, -0.87846039, 2.61800094, 1.41421356])
        self._k = compute_lqr_gains()
        self.integral_error = 0.0
        self.offset_pitch = 0.0313
        # self.offset_pitch = 0.04
        self.last_time = time.monotonic()

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.roll_rate = LinearFilter(0.5, 0.0)
        self.pitch_rate = LinearFilter(0.5, 0.0)
        # self.pitch_rate = KalmanFilter(0.1, 0.02)

        self.yaw_rate = LinearFilter(0.5, 0.0)
        self.fwd_velocity = KalmanFilter(0.001, 0.02)
        # self.fwd_velocity = LinearFilter(0.2, 0.0)

        self.cmd_fwd_velocity = LinearFilter(0.2, 0.0)
        self.cmd_yaw_rate = LinearFilter(0.2, 0.0)

        self.ctrl_velocity = 0.0
        self.ctrl_yaw_rate = 0.0

        self.diff_drive = DiffDriveKinematics(0.3, 0.075, 10.0)

    def update_imu(self, imu: Imu):
        q = imu.orientation
        if np.linalg.norm([q.x, q.y, q.z, q.w]) == 0:
            return

        self.rpy = R.from_quat([q.x, q.y, q.z, q.w]).as_euler("XYZ")
        self.roll, self.pitch, self.yaw = self.rpy
        self.roll_rate.update(imu.angular_velocity.x)
        self.pitch_rate.update(imu.angular_velocity.y)

    def update_joint_state(self, joint_state: JointState):
        self.joint_state = joint_state
        left_wheel_rps, right_wheel_rps = joint_state.velocity[2:]
        fwd_velocity, yaw_rate = self.diff_drive.forward_kinematics(
            left_wheel_rps, right_wheel_rps
        )
        self.fwd_velocity.update(fwd_velocity)
        self.yaw_rate.update(yaw_rate)

    def update_dt(self):
        now = time.monotonic()
        dt = now - self.last_time
        self.last_time = now
        if dt <= 0 or dt > 0.5:
            dt = 0.02
        return dt

    def update(self, joint_command: JointCommand):
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
        self.integral_error = np.clip(self.integral_error, -0.2, 0.2)

        # self.pitch_rate.update((self.pitch - last_pitch) / dt)
        # pitch_rate = self.pitch_rate.value
        # pitch_rate = 0

        ctrl_fwd_velocity = -(
            k_pitch * pitch
            + k_pitch_rate * pitch_rate
            + k_velocity * fwd_velocity
            + k_position * self.integral_error
        )

        # Add deadband near equilibrium to reduce chatter
        # if abs(pitch) < 0.02:
        #    ctrl_torque *= 0.2  # Reduce gain significantly near equilibrium

        ctrl_left_rps, ctrl_right_rps = self.diff_drive.inverse_kinematics(
            ctrl_fwd_velocity, cmd_yaw_rate
        )

        joint_command.velocity[2] = ctrl_left_rps
        joint_command.velocity[3] = ctrl_right_rps
