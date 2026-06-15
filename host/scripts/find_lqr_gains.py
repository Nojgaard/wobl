import control
import numpy as np
from dm_control import mjcf
from PIL import Image
from scipy.optimize import brentq

from woblpy.sim.robot import Robot


def compute_mass(model):
    total_mass = 0.0
    for i in range(model.nbody):
        body_mass = model.body_mass[i]
        body_name = model.body(i).name

        print(f"Body {i}: {body_name} (mass={body_mass:.6f} kg)")
        total_mass += body_mass
    return total_mass


def compute_total_com(model, data):
    total_mass = 0.0
    com = np.zeros(3)

    for i in range(model.nbody):
        body_mass = model.body_mass[i]
        # Get world position of this body's CoM
        body_com = data.xipos[i]
        com += body_mass * body_com
        total_mass += body_mass

    return com / total_mass if total_mass > 0 else com


def find_equilibrium(robot: Robot, physics: mjcf.Physics):
    com = compute_total_com(physics.model, physics.data)
    wheel_world = physics.bind(robot.mjcf_model.find("joint", "L_foot")).xanchor
    com_x_offset = com[0] - wheel_world[0]
    com_z_offset = com[2] - wheel_world[2]
    theta_eq = -com_x_offset / com_z_offset
    return theta_eq


def compute_com_height(robot: Robot, physics: mjcf.Physics):
    com = compute_total_com(physics.model, physics.data)[[0, 2]]
    wheel_pos = physics.bind(robot.mjcf_model.find("joint", "L_foot")).xanchor[[0, 2]]

    return np.linalg.norm(com - wheel_pos)


def settle(physics, robot, hip_angle, n_steps=300):
    physics.reset()
    physics.set_control([hip_angle, hip_angle, 0, 0])
    for _ in range(n_steps):
        physics.step()


def hip_foot_height_diff(physics, robot, hip_angle):
    settle(physics, robot, hip_angle)
    hip_z = physics.bind(robot.mjcf_model.find("joint", "L_hip")).xanchor[2]
    foot_z = physics.bind(robot.mjcf_model.find("joint", "L_foot")).xanchor[2]
    return hip_z - foot_z


def main():
    robot = Robot()
    physics = mjcf.Physics.from_mjcf_model(robot.mjcf_model)

    """target_height_m = 0.112
    h_min = hip_foot_height_diff(physics, robot, -0.2)
    h_max = hip_foot_height_diff(physics, robot, 0.65)
    print(f"Height diff range: [{h_min*1000:.1f}mm, {h_max*1000:.1f}mm] over hip [-0.2, 0.65] rad")
    hip_angle = brentq(
        lambda a: hip_foot_height_diff(physics, robot, a) - target_height_m,
        a=-0.2, b=0.65,
    )
    print(f"Hip angle for {target_height_m*1000:.0f}mm height diff: {hip_angle:.4f} rad")

    settle(physics, robot, hip_angle)
    """

    angle_offset = 0.1
    settle(physics, robot, hip_angle=0.3342 + angle_offset)
    #Image.fromarray(physics.render(height=480, width=640, camera_id=-1)).save("state.png")

    # System Parameters
    g = 9.81  # acceleration due to gravity (m/s^2)
    # sourced from https://blog.pictor.us/lqr-control-of-a-self-balancing-robot/
    # robot physical parameters
    m = compute_mass(physics.model)  # total mass (kg)
    l = compute_com_height(
        robot, physics
    )  # length to the center of mass from wheel axis (m)
    r = 0.04  # Wheel radius (m)
    theta_eq = find_equilibrium(robot, physics)

    print(f"Total mass (m): {m} kg")
    print(f"COM height (l): {l} m")
    print(f"Wheel radius (r): {r} m")
    print(f"Gravity (g): {g} m/s^2")
    print(f"Equilibrium pitch angle (theta_eq): {theta_eq} rad")

    A = np.array([[0, 1, 0], [g / l, 0, 0], [-g / m, 0, 0]])

    B = np.array([[0], [-1 / (m * (l**2))], [1 / m]])

    # Weighting matrices
    Q = np.diag([3.0, 0.2, 1.5, 1.0])  # Penalizing theta, theta_dot, x, and x_dot
    R = np.array([[1.0]])  # Penalizing control effort

    # Define which states to use for integral action
    # This matrix defines what linear combination of states to integrate
    # For velocity control, we want to integrate the velocity (4th state)
    C_integral = np.array([[0, 0, 1]])  # Integrate the velocity state

    # Calculate the LQR gain matrix K
    # This is what our Python script returns.
    K, _, _ = control.lqr(A, B, Q, R, integral_action=C_integral)

    print("A matrix:\n", A)
    print("B matrix:\n", B)
    print("Q matrix:\n", Q)
    print("R matrix:\n", R)
    print("LQR gain K:\n", K)


if __name__ == "__main__":
    main()
