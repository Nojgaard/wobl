import numpy as np
import rerun_sdk.rerun as rr
import zenoh
from scipy.spatial.transform import Rotation as R

from woblpy.common.node import Node
from woblpy.messages.imu_pb2 import Imu


def on_imu(sample: zenoh.Sample):
    imu = Imu()
    imu.ParseFromString(sample.payload.to_bytes())

    q = imu.orientation
    if np.linalg.norm([q.x, q.y, q.z, q.w]) == 0:
        return
    rpy = R.from_quat([q.x, q.y, q.z, q.w]).as_euler("XYZ")
    roll, pitch, yaw = rpy
    rr.log("imu/euler/roll", rr.Scalars(roll))
    rr.log("imu/euler/pitch", rr.Scalars(pitch))


def main():
    rr.init("woblpy_recording", spawn=True)
    print("Rerun initialized")
    rr.log("imu/euler/roll", rr.SeriesLines(names="Roll", colors=[255, 0, 0]))
    rr.log("imu/euler/pitch", rr.SeriesLines(names="Pitch", colors=[0, 255, 0]))
    with Node() as node:
        node.add_sub("imu", callback=on_imu)
        node.spin()
    print("Node shut down")


if __name__ == "__main__":
    main()
