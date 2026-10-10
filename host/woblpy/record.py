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

Checkpoints
-----------
    rec.log_checkpoint(3.2)                        # mark a timestamp

    from woblpy.record import load_checkpoints, load_recording

    checkpoints = load_checkpoints("data/run.rrd")    # list[float], seconds
    df, checkpoints = load_recording("data/run.rrd")  # both at once

Notes
-----
- ``live=True`` spawns the Rerun viewer as a separate OS process (``rr.spawn``).
  No additional thread is needed; ``rr.log`` is called inline from your loop.
- When both ``live=False`` and ``save_path=None`` the recorder is a no-op and
  ``rerun`` is never imported.
- Timing uses whichever ``t_s`` value the caller provides (seconds).
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Self

from woblpy.control.motion_controller import MotionController

# Entity holding the checkpoint markers.
_CHECKPOINT_ENTITY = "checkpoints/marker"


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

        self.configure_series("imu/gyro/x", name="Gyro X", color=(255, 80, 80))
        self.configure_series("imu/gyro/y", name="Gyro Y", color=(80, 255, 80))
        self.configure_series("imu/attitude/pitch", name="Pitch", color=(255, 160, 0))
        self.configure_series("imu/attitude/roll", name="Roll", color=(0, 200, 255))

        self.configure_series(
            "wheel/cmd/left/velocity", name="Left Wheel Cmd Vel", color=(255, 80, 255)
        )
        self.configure_series(
            "wheel/cmd/right/velocity", name="Right Wheel Cmd Vel", color=(255, 80, 255)
        )
        self.configure_series(
            "wheel/telem/left/velocity", name="Left Wheel Velocity", color=(80, 80, 255)
        )
        self.configure_series(
            "wheel/telem/right/velocity",
            name="Right Wheel Velocity",
            color=(255, 200, 0),
        )

        self.configure_series(
            "controller/fwd_velocity", name="Controller Fwd Vel", color=(255, 160, 0)
        )

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

    def log_controller(
        self,
        obs: dict,
        controller: MotionController,
        left_vel: float,
        right_vel: float,
        t_s: float,
    ) -> None:
        """Log controller state, wheel commands and raw wheel velocities."""
        state = controller.state
        wheel_vel = obs["robot/joint_velocities"]
        self.log_many(
            {
                "imu/gyro/x": state.roll_rate,
                "imu/gyro/y": state.pitch_rate,
                "imu/attitude/pitch": state.pitch,
                "imu/attitude/roll": state.roll,
                "wheel/cmd/left/velocity": left_vel,
                "wheel/cmd/right/velocity": right_vel,
                "wheel/telem/left/velocity": wheel_vel[2],
                "wheel/telem/right/velocity": wheel_vel[3],
                "controller/fwd_velocity": state.forward_velocity,
            },
            t_s=t_s,
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

    def log_checkpoint(self, t_s: float) -> None:
        """Mark t_s (seconds) as a checkpoint."""
        if self._rr is None:
            return
        self._rr.set_time("t_s", duration=t_s)
        self._rr.log(_CHECKPOINT_ENTITY, self._rr.Scalars(1.0))

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

    def __enter__(self) -> Self:
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

    # An entity is split over several chunks; concatenate them and drop the
    # handful of timestamps that land in two chunks.
    parts: dict[str, list[Any]] = {}
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
        parts.setdefault(name, []).append(
            pd.Series(s_values, index=t_values, name=name)
        )

    if not parts:
        return pd.DataFrame()

    columns = []
    for chunks in parts.values():
        series = pd.concat(chunks).sort_index()
        columns.append(series[~series.index.duplicated(keep="first")])

    df = pd.concat(columns, axis=1)
    df.index.name = timeline
    df.sort_index(inplace=True)
    return df


def load_checkpoints(path: str | Path) -> list[float]:
    """Return the checkpoint timestamps (seconds), sorted ascending."""
    import rerun.recording as rrec  # type: ignore

    times: list[float] = []
    for chunk in rrec.load_recording(str(path)).chunks():
        if chunk.is_static or chunk.entity_path.lstrip("/") != _CHECKPOINT_ENTITY:
            continue
        rb = chunk.to_record_batch()
        if "t_s" in rb.schema.names:
            times.extend(v.as_py().total_seconds() for v in rb.column("t_s"))
    return sorted(times)


def load_recording(
    path: str | Path, *, timeline: str = "t_s"
) -> tuple[Any, list[float]]:
    """Load a recording as ``(dataframe, checkpoint timestamps)``."""
    return load_as_dataframe(path, timeline=timeline), load_checkpoints(path)
