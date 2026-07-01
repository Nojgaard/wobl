"""Rerun-backed recording I/O for WOBL.

Writing
-------
    from woblpy.record import Recorder

    with Recorder("my_app", live=True, save_path="data/run.rrd") as rec:
        rec.configure_series("imu/gyro/x", name="Gyro X", color=(255, 80, 80))
        # inside your loop:
        rec.log_many({"imu/gyro/x": gx, "imu/gyro/y": gy}, t_s)

Reading
-------
    from woblpy.record import load_as_dataframe

    df = load_as_dataframe("data/run.rrd")
    # df.columns = ["imu/gyro/x", "imu/gyro/y", ...]
    # NaN where an entity wasn't logged at that timestamp (e.g. different phases)
    gyro = df[["imu/gyro/x", "imu/gyro/y", "imu/gyro/z"]].dropna()
    accel = df[["imu/attitude/pitch", "imu/attitude/roll"]].dropna()

Notes
-----
- ``live=True`` spawns the Rerun viewer as a separate OS process (``rr.spawn``).
  No additional thread is needed; ``rr.log`` is called inline from your loop.
- When both ``live=False`` and ``save_path=None`` the recorder is a no-op and
  ``rerun`` is never imported.
- Timing uses whichever ``t_s`` value the caller provides (seconds).  For IMU
  data, derive it from ``DriveTelemetry.timestamp_ms`` to use the firmware
  hardware clock and avoid host-side serial jitter.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from woblpy.control.controller import Controller
from woblpy.hardware.protocol import DriveCommand, DriveTelemetry


class Recorder:
    """Wraps Rerun for scalar logging, optional live viewing, and .rrd saving."""

    def __init__(
        self,
        app_id: str,
        *,
        live: bool = True,
        save_path: str | Path | None = None,
    ) -> None:
        self._enabled = live or save_path is not None
        self._save_path = Path(save_path) if save_path is not None else None
        self._rr: Any = None

        if not self._enabled:
            return

        import rerun as rr  # type: ignore

        self._rr = rr
        rr.init(app_id, spawn=live)

        self.configure_series("imu/gyro/x",         name="Gyro X",   color=(255, 80,  80))
        self.configure_series("imu/gyro/y",         name="Gyro Y",   color=(80,  255, 80))
        self.configure_series("imu/attitude/pitch", name="Pitch",    color=(255, 160,  0))
        self.configure_series("imu/attitude/roll",  name="Roll",     color=(0,   200, 255))

        self.configure_series("wheel/cmd/left/velocity",  name="Left Wheel Cmd Vel",  color=(255, 80,  255))
        self.configure_series("wheel/cmd/right/velocity", name="Right Wheel Cmd Vel", color=(255, 80,  255))
        self.configure_series("wheel/telem/left/velocity",  name="Left Wheel Velocity",  color=(80,  80,  255))
        self.configure_series("wheel/telem/right/velocity", name="Right Wheel Velocity", color=(255, 200, 0))
        self.configure_series("wheel/telem/left/current",   name="Left Wheel Current",  color=(80,  255, 80))
        self.configure_series("wheel/telem/right/current",  name="Right Wheel Current", color=(255, 255, 80))

        self.configure_series("controller/fwd_velocity", name="Controller Fwd Vel", color=(255, 160, 0))
        self.configure_series("controller/fwd_velocity_raw", name="Controller Fwd Vel Raw", color=(255, 160, 100))

    # ------------------------------------------------------------------
    # Configuration helpers
    # ------------------------------------------------------------------

    def configure_series(
        self,
        entity: str,
        *,
        name: str,
        color: tuple[int, int, int],
    ) -> None:
        """Set display name and colour for a scalar series.

        Call once before logging data for the entity so the Rerun viewer
        renders it with a human-readable label and a distinct colour.
        """
        if self._rr is None:
            return
        self._rr.log(entity, self._rr.SeriesLines(names=name, colors=list(color)))

    # ------------------------------------------------------------------
    # Logging
    # ------------------------------------------------------------------

    def log_controller(self, telem: DriveTelemetry, cmd: DriveCommand, controller: Controller) -> None:
        """Convenience method to log controller-relevant telemetry and commands."""
        self.log_many(
            {
                "imu/gyro/x": controller.pitch_rate.value,
                "imu/gyro/y": controller.yaw_rate.value,
                "imu/attitude/pitch": controller.pitch,
                "imu/attitude/roll": controller.roll,
                "wheel/cmd/left/velocity": cmd.left_velocity,
                "wheel/cmd/right/velocity": cmd.right_velocity,
                "wheel/telem/left/velocity": telem.left_vel,
                "wheel/telem/right/velocity": telem.right_vel,
                "wheel/telem/left/current": telem.left_current,
                "wheel/telem/right/current": telem.right_current,
                "controller/fwd_velocity": controller.fwd_velocity.value,
                "controller/fwd_velocity_raw": controller.fwd_velocity_raw,
            },
            t_s=telem.timestamp_ms / 1000.0,
        )

    def log(self, entity: str, value: float, t_s: float) -> None:
        """Log a single scalar at timestamp ``t_s`` (seconds)."""
        if self._rr is None:
            return
        self._rr.set_time("t_s", duration=t_s)
        self._rr.log(entity, self._rr.Scalars(value))

    def log_many(self, values: dict[str, float], t_s: float) -> None:
        """Log multiple scalars at the same timestamp ``t_s`` (seconds)."""
        if self._rr is None:
            return
        self._rr.set_time("t_s", duration=t_s)
        for entity, value in values.items():
            self._rr.log(entity, self._rr.Scalars(value))

    def log_text(self, entity: str, text: str) -> None:
        """Log a text annotation (e.g. phase or orientation label)."""
        if self._rr is None:
            return
        self._rr.log(entity, self._rr.TextLog(text))

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def close(self) -> None:
        """Save the .rrd file if ``save_path`` was given.  Idempotent."""
        if self._rr is not None and self._save_path is not None:
            self._save_path.parent.mkdir(parents=True, exist_ok=True)
            self._rr.save(str(self._save_path))
            print(f"  Saved recording → {self._save_path}")
            self._save_path = None  # prevent double-save
            print("  Recording closed.")

    def __enter__(self) -> Recorder:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


# ---------------------------------------------------------------------------
# Loading
# ---------------------------------------------------------------------------

def load_as_dataframe(path: str | Path, *, timeline: str = "t_s") -> Any:
    """Load an .rrd file saved by :class:`Recorder` into a pandas DataFrame.

    Each logged entity becomes a column named by its entity path (e.g.
    ``"imu/gyro/x"``).  The DataFrame is indexed by the chosen timeline in
    seconds.  Entities recorded in different phases (e.g. gyro vs attitude)
    will have NaN for one another's timestamps — split them with ``.dropna()``:

    .. code-block:: python

        df = load_as_dataframe("data/run.rrd")
        gyro  = df[["imu/gyro/x", "imu/gyro/y", "imu/gyro/z"]].dropna()
        accel = df[["imu/attitude/pitch", "imu/attitude/roll"]].dropna()

    Parameters
    ----------
    path:
        Path to the ``.rrd`` file.
    timeline:
        Timeline name to use as the index.  Must match what was passed to
        ``rr.set_time`` when logging (default: ``"t_s"``).

    Returns
    -------
    pandas.DataFrame
        Rows sorted by timeline value.  Columns are entity paths; NaN where
        an entity had no sample at that timestamp.
    """
    import pandas as pd  # type: ignore
    import rerun.recording as rrec  # type: ignore

    rec = rrec.load_recording(str(path))

    series: dict[str, Any] = {}
    for chunk in rec.chunks():
        if chunk.is_static:
            continue
        rb = chunk.to_record_batch()
        col_names = rb.schema.names
        if timeline not in col_names or "Scalars:scalars" not in col_names:
            continue

        # t_s is duration[ns] stored as timedelta — convert to float seconds
        t_values = [v.as_py().total_seconds() for v in rb.column(timeline)]
        # Scalars are stored as list<double> — unwrap the single-element list
        s_values = [v.as_py()[0] for v in rb.column("Scalars:scalars")]

        name = chunk.entity_path.lstrip("/")
        series[name] = pd.Series(s_values, index=t_values, name=name)

    if not series:
        return pd.DataFrame()

    df = pd.concat(series.values(), axis=1)
    df.index.name = timeline
    df.sort_index(inplace=True)
    return df
