import time

from woblpy.common.node import Node
from woblpy.messages.messages_pb2 import JointCommand, JointState


def main():
    node = Node()
    state = JointState()
    node.add_sub("joint_state", state)
    pub = node.add_pub("joint_command")
    joint_command = JointCommand(
        position=[0.0, 0.0, 0.0, 0.0],
        velocity=[0.0, 0.0, 0.0, 0.0],
    )

    while state.timestamp == 0:
        print("Waiting for driver...")
        time.sleep(1.0)

    print("[Wheel Eval] Driver is ready, sending command....")

    wheel_velocities = [
        0.0,
        0.5,
        1.0,
        5,
        10,
        -5,
        -10,
        -0.5,
        -1.0,
        0.0,
        1.5,
        0.5,
        -0.5,
        -1.5,
        0.0,
    ]
    durations = [5, 2, 2, 2.0, 2.5, 2.0, 2, 2.0, 2.5, 2.0, 0.5, 0.5, 0.5, 0.5, 2]

    joint_command.velocity[0] = 1.0
    joint_command.velocity[1] = 1.0
    joint_command.position[0] = state.position[0]
    joint_command.position[1] = state.position[1]

    for vel, dur in zip(wheel_velocities, durations):
        joint_command.velocity[2] = vel
        joint_command.velocity[3] = vel
        joint_command.timestamp = time.time()
        node.send(pub, joint_command)
        print(f"Sent wheel velocities: {vel} rad/s for {dur} seconds")
        time.sleep(dur)

    node.close()

    print("[Wheel Eval] Closing controller node.")


if __name__ == "__main__":
    main()
