import time

import numpy as np
from scipy.spatial.transform import Rotation as R

from woblpy.common.node import Node
from woblpy.messages.imu_pb2 import Imu
from woblpy.messages.joint_pb2 import JointCommand, JointState


def print_orientation(imu: Imu):
    q = imu.orientation
    if np.linalg.norm([q.x, q.y, q.z, q.w]) == 0:
        return

    rpy = R.from_quat([q.x, q.y, q.z, q.w]).as_euler("XYZ")
    print(
        f"IMU Orientation - Roll: {rpy[0]:.3f}, Pitch: {rpy[1]:.3f}, Yaw: {rpy[2]:.3f}"
    )


def main():
    node = Node()

    imu = Imu()
    node.add_sub("imu", imu)

    joint_state = JointState()
    node.add_sub("joint_state", joint_state)
    pub_cmd = node.add_pub("joint_command")

    while node.is_open() and (imu.timestamp == 0 or joint_state.timestamp == 0):
        print("Waiting for hardware feedback...")
        time.sleep(1)

    print("Received initial hardware feedback.")
    print("Standing up...")
    node.send(
        pub_cmd,
        JointCommand(
            position=[0.0, 0.0, 0.0, 0.0],
            velocity=[1.0, 1.0, 0.0, 0.0],
            timestamp=time.time(),
        ),
    )

    node.add_timer(lambda: print_orientation(imu), frequency_hz=2.0)
    node.spin()

    node.close()


if __name__ == "__main__":
    main()
