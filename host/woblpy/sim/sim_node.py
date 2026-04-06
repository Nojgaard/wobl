import time

import numpy as np
from dm_env import TimeStep

from woblpy.common.node import Node
from woblpy.messages.messages_pb2 import (
    Imu,
    JointCommand,
    JointState,
    Quaternion,
    Vector3,
)
from woblpy.sim.application import Application
from woblpy.sim.robot import Robot, RobotWorld


class SimNode(Node):
    def __init__(self):
        super().__init__()

        self.joint_command = JointCommand(
            position=np.zeros(4),
            velocity=np.zeros(4),
        )
        self.joint_state_pub = self.add_pub("joint_state")
        self.imu_pub = self.add_pub("imu")
        self.add_sub("joint_command", self.joint_command)

        robot = Robot()
        world = RobotWorld(robot)
        world.set_timesteps(control_timestep=0.015, physics_timestep=0.005)
        self.app = Application(world, self.update)

    def update(self, timestep: TimeStep):
        if not self._is_open:
            self.app.running = False
            return np.zeros(4)

        now = time.time()
        orientation = timestep.observation["robot/orientation"]
        angular_velocity = timestep.observation["robot/angular_velocity"]
        linear_acceleration = timestep.observation["robot/linear_acceleration"]

        imu = Imu(
            timestamp=now,
            orientation=Quaternion(
                x=orientation[1], y=orientation[2], z=orientation[3], w=orientation[0]
            ),
            angular_velocity=Vector3(
                x=angular_velocity[0], y=angular_velocity[1], z=angular_velocity[2]
            ),
            linear_acceleration=Vector3(
                x=linear_acceleration[0],
                y=linear_acceleration[1],
                z=linear_acceleration[2],
            ),
        )

        joint_state = JointState(
            timestamp=now,
            position=timestep.observation["robot/joint_positions"],
            velocity=timestep.observation["robot/joint_velocities"],
            effort=timestep.observation["robot/joint_efforts"],
        )
        resolution = 0.105
        quantized_velocity = (
            np.round(np.array(joint_state.velocity) / resolution) * resolution
        )
        joint_state.velocity[2] = quantized_velocity[2]
        joint_state.velocity[3] = quantized_velocity[3]
        self.send(self.imu_pub, imu)
        self.send(self.joint_state_pub, joint_state)

        cmd = self.joint_command
        resolution = 0.105
        quantized_velocity = np.round(np.array(cmd.velocity) / resolution) * resolution
        action = np.array(
            [
                cmd.position[0],
                cmd.position[1],
                quantized_velocity[2],
                quantized_velocity[3],
            ]
        )
        return action


def main():
    print("[SimNode] Starting simulation node...")
    with SimNode() as node:
        try:
            node.app.launch()
        except KeyboardInterrupt:
            pass
    print("[SimNode] Closing node")


if __name__ == "__main__":
    main()
