import numpy as np

# Simulation fit
HEIGHT_TO_PITCH_COEFFS = np.array([-146.6411667, 55.03699507, -7.32007243, 0.37413675])

# Emperical fit
# HEIGHT_TO_PITCH_COEFFS = np.array(
#    [-193.81821387, 75.18364797, -10.34721967, 0.52621788]
# )


def from_height(height: float) -> float:
    """Return the pitch offset (rad) for the given height (m)."""
    return float(np.polyval(HEIGHT_TO_PITCH_COEFFS, height))
