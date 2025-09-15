class LinearFilter:
    def __init__(self, alpha: float, initial_value: float):
        self.alpha = alpha
        self.value = initial_value

    def update(self, new_value: float):
        self.value = self.alpha * new_value + (1 - self.alpha) * self.value
