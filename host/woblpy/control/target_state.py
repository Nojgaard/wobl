import numpy as np

from woblpy.control.leg_kinematics import LegKinematics


class TargetState:
    """A target state for the robot to achieve."""

    _HEIGHT_RANGE = (0.06, 0.16)
    _ROLL_RANGE = (-0.3, 0.3)
    _FORWARD_VELOCITY_RANGE = (-0.3, 0.3)
    _TURN_VELOCITY_RANGE = (-1.0, 1.0)

    _MAX_ROLL_RATE = 3.0  # rad/s

    def __init__(self, leg_ik: LegKinematics):
        self._height: float = leg_ik.to_height(0)
        self._roll: float = 0.0
        self._forward_velocity: float = 0.0
        self._turn_velocity: float = 0.0

        self.roll_command: float = 0.0

    @property
    def height(self) -> float:
        return self._height

    @height.setter
    def height(self, height: float) -> None:
        self._height = float(np.clip(height, *self._HEIGHT_RANGE))

    @property
    def roll(self) -> float:
        return self._roll

    @roll.setter
    def roll(self, roll: float) -> None:
        self._roll = float(np.clip(roll, *self._ROLL_RANGE))

    @property
    def forward_velocity(self) -> float:
        return self._forward_velocity

    @forward_velocity.setter
    def forward_velocity(self, velocity: float) -> None:
        self._forward_velocity = float(np.clip(velocity, *self._FORWARD_VELOCITY_RANGE))

    @property
    def turn_velocity(self) -> float:
        return self._turn_velocity

    @turn_velocity.setter
    def turn_velocity(self, velocity: float) -> None:
        self._turn_velocity = float(np.clip(velocity, *self._TURN_VELOCITY_RANGE))

    def update(self, dt: float) -> None:
        roll_error = self._roll - self.roll_command
        max_roll_change = self._MAX_ROLL_RATE * dt
        self.roll_command += np.clip(roll_error, -max_roll_change, max_roll_change)
