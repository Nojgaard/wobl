"""First-order low-pass (EMA) filter, mirroring SimpleFOC's ``LowPassFilter``.

The firmware observer (``firmware/src/control/observer.cpp``) filters gyro
rates and wheel velocities with SimpleFOC's ``LowPassFilter``:

    alpha = Tf / (Tf + dt)
    y     = alpha * y_prev + (1 - alpha) * x

where ``Tf`` is the filter time constant in seconds.  With ``Tf = 0`` the
filter is a passthrough (alpha = 0), which matches the firmware observer's
default configuration.

Unlike the C++ version, ``dt`` is passed explicitly per call rather than read
from the microsecond clock internally — the host already has an exact ``dt``
from the telemetry timestamps.
"""

from __future__ import annotations


class LowPassFilter:
    """First-order low-pass filter with an explicit sample time.

    Args:
        Tf: Filter time constant in seconds. ``0.0`` = passthrough.
        initial: Initial filtered value.
    """

    def __init__(self, Tf: float, initial: float = 0.0) -> None:
        self.Tf = Tf
        self.value = initial

    def __call__(self, x: float, dt: float) -> float:
        """Filter ``x`` sampled ``dt`` seconds after the previous call."""
        alpha = self.Tf / (self.Tf + dt)
        self.value = alpha * self.value + (1.0 - alpha) * x
        return self.value
