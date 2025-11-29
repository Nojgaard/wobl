import time

import zenoh

from woblpy.common.node import Node
from woblpy.control.controller import Controller
from woblpy.messages.messages_pb2 import ControllerState, Imu, JointCommand, JointState


class ControllerNode(Node):
    def __init__(self):
        super().__init__()
        
        self.controller = Controller()
        self.controller_state = ControllerState()

        self.joint_command_pub = self.add_pub("joint_command")
        self.joint_command = JointCommand(
            position=[0.0, 0.0, 0.0, 0.0],
            velocity=[1.0, 1.0, 0.0, 0.0],
        )

        self.imu = Imu()
        self.imu_sub = self.add_sub("imu", callback=self.update_imu)

        self.joint_state = JointState()
        self.joint_state_sub = self.add_sub(
            "joint_state", callback=self.update_joint_state
        )

        self.state_pub = self.add_pub("controller_state")

        self.add_timer(self.update, frequency_hz=80.0)

    def update_imu(self, sample: zenoh.Sample):
        self.imu.ParseFromString(sample.payload.to_bytes())
        self.controller.update_imu(self.imu)

    def update_joint_state(self, sample: zenoh.Sample):
        self.joint_state.ParseFromString(sample.payload.to_bytes())
        self.controller.update_joint_state(self.joint_state)

    def publish_controller_state(self):
        controller = self.controller
        state = self.controller_state
        state.timestamp = time.time()

        state.orientation.x = controller.roll
        state.orientation.y = controller.pitch

        state.angular_velocity.x = controller.roll_rate.value
        state.angular_velocity.y = controller.pitch_rate.value

        state.tar_fwd_velocity = controller.fwd_velocity.value
        state.tar_yaw_rate = controller.yaw_rate.value

        state.cmd_fwd_velocity = controller.ctrl_velocity
        state.cmd_yaw_rate = controller.ctrl_yaw_rate
        self.send(self.state_pub, state)

    def update(self):
        if self.imu.timestamp == 0 or self.joint_state.timestamp == 0:
            return

        joint_command = self.joint_command
        self.controller.update(joint_command)
        joint_command.timestamp = time.time()
        self.send(self.joint_command_pub, joint_command)

        self.publish_controller_state()


if __name__ == "__main__":
    print("[ControllerNode] Starting controller node...")
    with ControllerNode() as node:
        node.spin()
    print("[ControllerNode] Closing node")
