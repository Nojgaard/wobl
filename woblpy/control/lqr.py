import control
import numpy as np


def compute_lqr_gains():
    """Compute LQR gains for a self-balancing, two-wheeled biped robot.

    This function calculates the LQR controller gains for a simplified
    inverted-pendulum model of the robot, using its physical parameters.
    """

    # ---- System Dynamics Parameters ----
    mass = 1.8909214736935442  # Mass of the robot (kg)
    com_length = 0.12300142698585038  # Height of the center of mass (m)
    gravity = 9.80665  # Acceleration due to gravity (m/s^2)
    torque_constant = 0.37  # Motor torque constant (Nm/A)

    # ---- Continuous-Time State-Space Model ----
    #
    # We approximate the robot as a single inverted pendulum on wheels.
    #
    # State vector x = [θ, θ̇, v]
    #   θ   : Body pitch angle (rad)
    #   θ̇  : Body angular velocity (rad/s)
    #   v   : Robot linear velocity (m/s)
    #
    # Input u = Wheel torque (Nm)
    #
    # The A matrix describes how the state evolves with no input:
    A = np.array(
        [
            [0, 1, 0],  # d(θ)/dt = θ̇
            [gravity / com_length, 0, 0],  # d(θ̇)/dt ∝ gravity and CoM height
            [-gravity / mass, 0, 0],  # d(v)/dt affected by body angle
        ]
    )

    # The B matrix describes how the input torque affects the system.
    #
    # It captures how wheel torque influences:
    #   - Angular acceleration of the body
    #   - Linear acceleration of the robot
    B = np.array(
        [
            [0],  # Torque does not directly influence θ
            [-1 / (mass * (com_length**2))],  # Torque creates angular acceleration
            [1 / mass],  # Torque creates linear acceleration
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
            5.0,  # θ      (most important: avoid falling)
            0.5,  #  θ̇      (reduce fast tipping)
            1.0,  #  v      (light penalty to discourage runaway speed)
            2.5,  # Integral of v (to eliminate steady-state error)
        ]
    )

    # R penalizes the control effort (wheel torque).
    # Larger R → less aggressive control (reduced torque usage).
    R = np.array([[2.0]])

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
