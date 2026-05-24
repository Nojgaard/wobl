import math


class KalmanFilter:
    def __init__(self, process_noise, measurement_noise):
        """
        Initialize Kalman Filter

        Args:
            process_noise (float): Process noise variance (q)
            measurement_noise (float): Measurement noise variance (r)
        """
        self.q = process_noise
        self.r = measurement_noise
        self.x = 0.0
        self.p = 1.0

    def update(self, measurement, dt=1.0, target=None, tau=None):
        """
        Update step of Kalman filter

        Args:
            measurement (float): Measurement value
            dt (float): Time step (default: 1.0)
            target (float | None): Known control target. When provided together
                with tau, the prediction step uses a first-order model toward
                the target instead of a random walk, reducing lag on commanded
                changes without affecting steady-state noise rejection.
            tau (float | None): Closed-loop plant time constant in seconds.

        Returns:
            float: Updated state estimate
        """
        # Predict
        if target is not None and tau is not None:
            # First-order model: x drifts toward target with time constant tau.
            # alpha = 1 - exp(-dt/tau) is the exact discrete-time step.
            alpha = 1.0 - math.exp(-dt / tau)
            self.x = self.x + alpha * (target - self.x)
            self.p = (1.0 - alpha) ** 2 * self.p + self.q * dt
        else:
            # Random-walk model (original behaviour)
            self.p = self.p + self.q * dt

        # Kalman gain
        k = self.p / (self.p + self.r)

        # State update
        self.x = self.x + k * (measurement - self.x)

        # Error covariance update
        self.p = (1.0 - k) * self.p

        return self.x

    @property
    def value(self):
        """Get current state estimate"""
        return self.x

    def covariance(self):
        """Get current error covariance"""
        return self.p

    def reset(self, initial_state=0.0, initial_covariance=1.0):
        """
        Reset filter to initial conditions

        Args:
            initial_state (float): Initial state value (default: 0.0)
            initial_covariance (float): Initial covariance value (default: 1.0)
        """
        self.x = initial_state
        self.p = initial_covariance
