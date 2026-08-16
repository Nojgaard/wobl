import numpy as np

HEIGHT_TO_PITCH_COEFFS = np.array([-180.146716, 69.166527, -9.579059, 0.535259])


def from_height(height: float) -> float:
    """Return the pitch offset (rad) for the given height (m)."""
    return float(np.polyval(HEIGHT_TO_PITCH_COEFFS, height))
