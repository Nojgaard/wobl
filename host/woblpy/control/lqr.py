import control
import numpy as np


def compute_lqr_gains():
    """Compute LQR gains for a self-balancing, two-wheeled biped robot.

    This function calculates the LQR controller gains for a simplified
    inverted-pendulum model of the robot, using its physical parameters.
    """

    # ---- System Dynamics Parameters ----
    # Two-mass coupled model: body (pendulum) + wheels (cart).
    # A single point mass gives a singular coupling matrix (Δ=0);
    # nonzero wheel mass is required for a well-posed system.
    #
    # Masses from robot.xml: total ~1.2 kg, wheels 0.061 kg each.

    m_b = 1.078              # Body mass above axle = total - 2×wheel (kg)
    m_w = 0.061              # Single wheel mass (kg)
    com_length = 0.06089132455005064  # Height of CoM above wheel axle (m)
    gravity = 9.80665        # Acceleration due to gravity (m/s²)
    wheel_radius = 0.04      # Radius of the wheels (m)

    # ---- Derived quantities ----
    I_b = 0.0                # Body moment of inertia about own CoM (point-mass approx)
    I_w = 0.5 * m_w * wheel_radius**2  # Wheel moment of inertia (solid disk approx)
    I_axle = I_b + m_b * com_length**2   # Body inertia about wheel axle (parallel axis)
    M_eff = m_b + 2*m_w + 2*I_w / wheel_radius**2  # Effective translational mass
    Delta = I_axle * M_eff - (m_b * com_length)**2  # Coupling determinant (>0)

    # ---- Continuous-Time State-Space Model ----
    #
    # Coupled inverted-pendulum-on-wheels model.
    # Derived from two linearized equations solved simultaneously:
    #
    #   Body rotation about axle:  I_axle·θ̈ + m_b·l·v̇ = m_b·g·l·θ
    #   Combined translation:      m_b·l·θ̈ + M_eff·v̇ = τ / r
    #
    # State vector x = [θ, θ̇, v]
    #   θ   : Body pitch angle (rad)
    #   θ̇  : Body angular velocity (rad/s)
    #   v   : Robot linear velocity (m/s)
    #
    # Input u = Wheel torque τ (Nm)
    #
    # Solving the 2×2 system (Δ = I_axle·M_eff − (m_b·l)²):
    #   θ̈ = (M_eff·m_b·g·l / Δ)·θ  −  (m_b·l / (r·Δ))·τ
    #   v̇ = (−(m_b·l)²·g / Δ)·θ  +  (I_axle / (r·Δ))·τ
    #
    a_21 = M_eff * m_b * gravity * com_length / Delta
    a_31 = -((m_b * com_length) ** 2) * gravity / Delta
    b_21 = -m_b * com_length / (wheel_radius * Delta)
    b_31 = I_axle / (wheel_radius * Delta)

    A = np.array(
        [
            [0, 1, 0],      # d(θ)/dt = θ̇
            [a_21, 0, 0],   # θ̈  (>0, gravity amplifies tilt)
            [a_31, 0, 0],   # v̇  (<0, forward tilt pushes wheels back)
        ]
    )

    B = np.array(
        [
            [0],            # τ does not directly affect θ
            [b_21],         # θ̈ contribution  (<0, torque tilts body backward)
            [b_31],         # v̇ contribution   (>0, torque accelerates robot forward)
        ]
    )

    # ---- LQR Cost Matrices ----
    #
    # Q penalizes deviation of states — it defines what we care about regulating.
    # Higher weights = stronger correction.
    #
    # We penalize:
    #   θ (body inclination): Keep the robot upright
    #   θ̇ (angular speed): Smooth motion
    #   v (forward speed): Don't accelerate too quickly
    #
    Q = np.diag(
        [
            3.0,  # θ      (most important: avoid falling)
            0.2,  #  θ̇      (reduce fast tipping)
            1.5,  #  v      (light penalty to discourage runaway speed)
            1.0,  # Integral of v (to eliminate steady-state error)
        ]
    )

    # R penalizes the control effort (wheel torque).
    # Larger R → less aggressive control (reduced torque usage).
    R = np.array([[1.0]])

    # ---- Integral Action ----
    #
    # Optional: integrate a specific state to eliminate steady-state error.
    # Here, we integrate robot velocity (v) to track a desired forward speed.
    #
    # C_integral defines which combination of states to integrate.
    # In this case:  integrate 'v'
    C_integral = np.array([[0, 0, 1.0]])  # Integrate the velocity state

    # ---- Compute the LQR Gains ----
    #
    # The controller will produce:
    #       u = -Kx
    # Where K contains the gains applied to each state.
    #
    K, _, _ = control.lqr(A, B, Q, R, integral_action=C_integral)

    return K[0]


if __name__ == "__main__":
    K = compute_lqr_gains()
    print("Computed LQR Gains:")
    print(K)
