import control
import numpy as np


def compute_lqr_gains():
    # sourced from https://blog.pictor.us/lqr-control-of-a-self-balancing-robot/
    m = 1.2  # mass of the robot (kg)
    L = 0.06  # distance from wheel axle to center of mass (m)
    g = 9.81  # acceleration due to gravity (m/s^2)
    r = 0.04  # radius of the wheels (m)

    # State vector: [x, x_dot, theta, theta_dot]
    A = np.array(
        [
            [0, 1, 0, 0],  #     position
            [0, 0, g, 0],  #     velocity
            [0, 0, 0, 1],  #     angle
            [0, 0, g / L, 0],  # angular velocity
        ]
    )
    B = np.array([[0], [1 / (m * r)], [0], [-1 / (m * L * r)]])

    Q = np.diag(
        [
            2.0,  # x          (prefer to stay in place)
            3.0,  # x_dot      (do not run away)
            150.0,  # theta      (most important: avoid falling)
            0.5,  # theta_dot  (was 0.5; increased to push phase lead >45 deg at ~1.6 Hz)
        ]
    )

    # R penalizes the control effort (wheel torque).
    # Larger R → less aggressive control (reduced torque usage).
    R = np.array([[1.3]])

    K, _, _ = control.lqr(A, B, Q, R)
    K = K[0]

    # [k_pitch, k_pitch_rate, k_velocity, k_position]
    return np.array([K[2], K[3], K[1], K[0]])


if __name__ == "__main__":
    K = compute_lqr_gains()
    print("Computed LQR Gains:")
    print(K)
