import os

import numpy as np
from dm_control import composer, mjcf
from dm_control.composer import Entity, Observables, Task
from dm_control.composer.observation import observable
from dm_control.composer.variation import distributions, noises
from dm_control.locomotion.arenas import Floor
from dm_control.mujoco import Physics


class Robot(Entity):
    def __init__(self, assets_dir: str = "mjcf"):
        self._model_path = os.path.join(assets_dir, "robot.xml")
        super().__init__()

    def _build(self):
        self._model = mjcf.from_path(self._model_path)
        self.joint_names = ["L_hip", "R_hip", "L_foot", "R_foot"]
        self.mjcf_joints = [
            self.mjcf_model.find("joint", name) for name in self.joint_names
        ]

    @property
    def mjcf_model(self):
        return self._model

    def _build_observables(self):
        return RobotObservables(self)

    @property
    def observables(self) -> "RobotObservables":
        return super().observables  # type: ignore


class RobotObservables(Observables):
    _entity: Robot

    @composer.observable
    def joint_positions(self):
        return observable.MJCFFeature("qpos", self._entity.mjcf_joints)

    @composer.observable
    def joint_velocities(self):
        return observable.MJCFFeature("qvel", self._entity.mjcf_joints)

    @composer.observable
    def joint_efforts(self):
        return observable.Generic(lambda physics: physics.data.actuator_force)

    @composer.observable
    def angular_velocity(self):
        return observable.MJCFFeature(
            "sensordata",
            self._entity.mjcf_model.sensor.gyro,  # type: ignore
        )

    @composer.observable
    def linear_acceleration(self):
        return observable.MJCFFeature(
            "sensordata",
            self._entity.mjcf_model.sensor.accelerometer,  # type: ignore
        )

    @composer.observable
    def linear_velocity(self):
        return observable.MJCFFeature(
            "sensordata",
            self._entity.mjcf_model.sensor.velocimeter,  # type: ignore
        )

    @composer.observable
    def orientation(self):
        return observable.MJCFFeature(
            "sensordata",
            self._entity.mjcf_model.sensor.framequat,  # type: ignore
        )


class RobotWorld(Task):
    def __init__(self, robot: Robot, static_entity=False):
        super().__init__()
        self.robot = robot
        self._arena = Floor(reflectance=0.0)
        if static_entity:
            self._arena.attach(self.robot)
        else:
            self._arena.add_free_entity(self.robot)
        self.set_timesteps(control_timestep=0.015, physics_timestep=0.005)

        self.robot.observables.joint_positions.enabled = True
        self.robot.observables.joint_velocities.enabled = True
        self.robot.observables.joint_velocities.corruptor = noises.Multiplicative(
            distributions.LogNormal(sigma=0.03)
        )
        self.robot.observables.joint_velocities.delay = 1
        self.robot.observables.joint_efforts.enabled = True

        self.robot.observables.orientation.enabled = True
        self.robot.observables.orientation.delay = 1

        self.robot.observables.angular_velocity.enabled = True
        self.robot.observables.linear_acceleration.enabled = True

        self.robot.observables.angular_velocity.delay = 1
        self.robot.observables.linear_acceleration.delay = 1

        self.robot.observables.linear_acceleration.enabled = True

    def initialize_episode(self, physics: Physics, random_state):
        self.robot.set_pose(physics, np.array([0, 0, 0.24]), np.array([1, 0, 0, 0]))

    @property
    def root_entity(self):
        return self._arena

    def get_reward(self, physics: Physics):
        return 0
