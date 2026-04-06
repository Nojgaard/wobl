import numpy as np


class DiffDriveKinematics:
    def __init__(self, wheel_base: float, wheel_radius: float, wheel_max_rps: float):
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_radius
        self.wheel_max_rps = wheel_max_rps

    def forward_kinematics(
        self, left_wheel_rps: float, right_wheel_rps: float
    ) -> np.ndarray:
        # Compute the robot's forward kinematics based on the wheel velocities
        wheel_radius, wheel_base = self.wheel_radius, self.wheel_base
        fwd_velocity = (left_wheel_rps + right_wheel_rps) / 2 * wheel_radius
        yaw_rate = (right_wheel_rps - left_wheel_rps) / wheel_base * wheel_radius
        return np.array([fwd_velocity, yaw_rate])

    def inverse_kinematics(self, fwd_velocity: float, yaw_rate: float) -> np.ndarray:
        # Compute the wheel velocities required to achieve the desired robot velocities
        yaw_rps = yaw_rate * self.wheel_base / 2
        left_wheel_rps = (fwd_velocity - yaw_rps) / self.wheel_radius
        right_wheel_rps = (fwd_velocity + yaw_rps) / self.wheel_radius
        max_rps = self.wheel_max_rps
        return np.clip([left_wheel_rps, right_wheel_rps], -max_rps, max_rps)
