import numpy as np
import rerun_sdk.rerun as rr
import zenoh
from scipy.spatial.transform import Rotation as R

from woblpy.common.node import Node
from woblpy.messages.messages_pb2 import ControllerState, Imu, JointCommand, JointState

imu = Imu()
joint_state = JointState()
joint_command = JointCommand()
controller_state = ControllerState()


def on_imu(sample: zenoh.Sample):
    imu.ParseFromString(sample.payload.to_bytes())

    q = imu.orientation
    if np.linalg.norm([q.x, q.y, q.z, q.w]) == 0:
        return
    rpy = R.from_quat([q.x, q.y, q.z, q.w]).as_euler("XYZ")
    roll, pitch, yaw = rpy
    rr.log("imu/euler/roll", rr.Scalars(roll))
    rr.log("imu/euler/pitch", rr.Scalars(pitch))
    rr.log("imu/euler/yaw", rr.Scalars(yaw))

    rr.log("imu/euler/roll_rate", rr.Scalars(imu.angular_velocity.x))
    rr.log("imu/euler/pitch_rate", rr.Scalars(imu.angular_velocity.y))
    rr.log("imu/euler/yaw_rate", rr.Scalars(imu.angular_velocity.z))


def on_joint_state(sample: zenoh.Sample):
    joint_state.ParseFromString(sample.payload.to_bytes())

    rr.log("joint/state/effort/left", rr.Scalars(joint_state.effort[2]))
    rr.log("joint/state/effort/right", rr.Scalars(joint_state.effort[3]))

    rr.log("joint/state/velocity/left", rr.Scalars(joint_state.velocity[2]))
    rr.log("joint/state/velocity/right", rr.Scalars(joint_state.velocity[3]))


def on_joint_command(sample: zenoh.Sample):
    joint_command.ParseFromString(sample.payload.to_bytes())
    rr.log("joint/command/velocity/left", rr.Scalars(joint_command.velocity[2]))
    rr.log("joint/command/velocity/right", rr.Scalars(joint_command.velocity[3]))


def on_controller_state(sample: zenoh.Sample):
    controller_state.ParseFromString(sample.payload.to_bytes())

    rr.log(
        "controller/state/fwd_velocity", rr.Scalars(controller_state.tar_fwd_velocity)
    )
    rr.log("controller/state/yaw_rate", rr.Scalars(controller_state.tar_yaw_rate))

    rr.log("controller/state/pitch", rr.Scalars(controller_state.orientation.y))
    rr.log(
        "controller/state/pitch_rate", rr.Scalars(controller_state.angular_velocity.y)
    )


def main():
    rr.init("woblpy_recording", spawn=False)
    server_uri = rr.serve_grpc()
    print("Rerun initialized:", server_uri)

    # rr.save("data/current_recording.rrd")

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
        node.add_sub("controller_state", callback=on_controller_state)
        node.spin()
    print("Node shut down")


if __name__ == "__main__":
    main()
