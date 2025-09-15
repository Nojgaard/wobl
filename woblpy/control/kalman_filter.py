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

    def update(self, measurement, dt=1.0):
        """
        Update step of Kalman filter

        Args:
            measurement (float): Measurement value
            dt (float): Time step (default: 1.0)

        Returns:
            float: Updated state estimate
        """
        # State prediction (assuming constant velocity model)
        # x_k = x_{k-1} (no state transition for position-only model)

        # Error covariance prediction
        self.p = self.p + self.q * dt

        # Kalman gain
        k = self.p / (self.p + self.r)

        # State update
        self.x = self.x + k * (measurement - self.x)

        # Error covariance update
        self.p = (1.0 - k) * self.p

        return self.x

    def state(self):
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
