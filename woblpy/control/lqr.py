import control
import numpy as np


def compute_lqr_gains():
    """Compute LQR gains for a self-balancing two-wheeled robot.

    Returns:
        K: Gain vector [k_pitch, k_pitch_rate, k_position, k_velocity]
    """
    # Physical parameters
    # m_body, m_wheel = 2.3909, 0.25  # Body and wheel mass (kg)
    m_body, m_wheel = 2.0, 0.20  # Body and wheel mass (kg)
    # com_height, r = 0.09779, 0.039  # CoM height and wheel radius (m)
    # com_height, r = 0.11, 0.039  # CoM height and wheel radius (m)
    com_height, r = 0.12, 0.075  # CoM height and wheel radius (m)
    g = 9.80665  # Gravity (m/s²)

    # Moments of inertia
    I_body = m_body * com_height**2
    I_wheel = 0.5 * m_wheel * r**2
    m_total = m_body + m_wheel

    # Effective inertias
    I_eff = I_body + m_body * com_height**2  # Total body rotational inertia
    m_eff = m_total + I_wheel / r**2  # Effective mass including wheel inertia

    # Coupled dynamics denominator
    denom = I_eff * m_eff - (m_body * com_height) ** 2

    # State space: x = [θ, θ̇, x_pos, ẋ], u = τ (wheel torque)
    A = np.array(
        [
            [0, 1, 0, 0],
            [m_body * g * com_height * m_eff / denom, 0, 0, 0],
            [0, 0, 0, 1],
            [m_body**2 * g * com_height**2 / denom, 0, 0, 0],
        ]
    )

    B = np.array([[0], [-m_eff / (r * denom)], [0], [I_eff / (r * denom)]])

    # LQR cost matrices
    Q = np.diag(
        [
            3.0,  # θ - pitch angle (critical!)
            0.2,  # θ̇ - angular velocity
            1.5,  # x_pos - position (for integral action)
            1.0,  # ẋ - linear velocity
        ]
    )

    R = np.array([[0.8]])  # Torque effort (in Nm²)

    K, _, _ = control.lqr(A, B, Q, R)
    return K[0]


if __name__ == "__main__":
    K = compute_lqr_gains()
    print(f"LQR Gains: {K}")
