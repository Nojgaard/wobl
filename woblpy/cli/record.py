import numpy as np
import rerun_sdk.rerun as rr
import zenoh
from scipy.spatial.transform import Rotation as R

from woblpy.common.node import Node
from woblpy.messages.imu_pb2 import Imu
from woblpy.messages.joint_pb2 import JointCommand, JointState

imu = Imu()
joint_state = JointState()
joint_command = JointCommand()


def on_imu(sample: zenoh.Sample):
    imu.ParseFromString(sample.payload.to_bytes())

    q = imu.orientation
    if np.linalg.norm([q.x, q.y, q.z, q.w]) == 0:
        return
    rpy = R.from_quat([q.x, q.y, q.z, q.w]).as_euler("XYZ")
    roll, pitch, yaw = rpy
    rr.log("imu/euler/roll", rr.Scalars(roll))
    rr.log("imu/euler/pitch", rr.Scalars(pitch))


def on_joint_state(sample: zenoh.Sample):
    joint_state.ParseFromString(sample.payload.to_bytes())
    rr.log("joint/state/velocity/left", rr.Scalars(joint_state.velocity[2]))
    rr.log("joint/state/velocity/right", rr.Scalars(joint_state.velocity[3]))


def on_joint_command(sample: zenoh.Sample):
    joint_command.ParseFromString(sample.payload.to_bytes())
    rr.log("joint/command/velocity/left", rr.Scalars(joint_command.velocity[2]))
    rr.log("joint/command/velocity/right", rr.Scalars(joint_command.velocity[3]))


def main():
    rr.init("woblpy_recording", spawn=False)
    print("Rerun initialized")

    rr.save("data/current_recording.rrd")

    rr.log("imu/euler/roll", rr.SeriesLines(names="Roll", colors=[255, 0, 0]))
    rr.log("imu/euler/pitch", rr.SeriesLines(names="Pitch", colors=[0, 255, 0]))

    rr.log(
        "joint/command/velocity/left",
        rr.SeriesLines(names="Left Command Velocity", colors=[0, 0, 255]),
    )
    rr.log(
        "joint/command/velocity/right",
        rr.SeriesLines(names="Right Command Velocity", colors=[0, 165, 255]),
    )
    rr.log(
        "joint/state/velocity/left",
        rr.SeriesLines(names="Left State Velocity", colors=[255, 0, 0]),
    )
    rr.log(
        "joint/state/velocity/right",
        rr.SeriesLines(names="Right State Velocity", colors=[255, 165, 0]),
    )
    with Node() as node:
        node.add_sub("imu", callback=on_imu)
        node.add_sub("joint_state", callback=on_joint_state)
        node.add_sub("joint_command", callback=on_joint_command)
        node.spin()
    print("Node shut down")


if __name__ == "__main__":
    main()
