#!/usr/bin/env python3
"""Plot a bench.csv telemetry file produced by wheel_tune.py.

Two subplots (left wheel, right wheel) each show:
  - Target velocity
  - Measured velocity (raw)
  - Smoothed velocity  (KalmanFilter — same type as the LQR controller uses)

Usage
-----
    uv run scripts/plot_bench.py [bench.csv] [--q 1.0] [--r 0.25]
    python scripts/plot_bench.py [bench.csv]   (default path: bench.csv)
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

sys.path.insert(0, str(Path(__file__).parent.parent))

from woblpy.control.kalman_filter import KalmanFilter

_DEFAULT_CSV = Path(__file__).parent.parent / "bench.csv"
_Q_DEFAULT = 1.0    # process noise — tune upward for faster response
_R_DEFAULT = 0.25   # measurement noise ≈ variance of encoder noise (σ≈0.54 rad/s on bench data)
_TAU_DEFAULT = 0.02  # plant time constant in seconds. Measured 63%-rise time of ~20 ms on bench data.

def _apply_kalman(
    series: pd.Series, t: pd.Series, q: float, r: float,
    target: pd.Series | None = None, tau: float | None = None,
) -> list[float]:
    kf = KalmanFilter(q, r)
    out: list[float] = []
    prev_t = float(t.iloc[0])
    for i, (val, ts) in enumerate(zip(series, t)):
        dt = max(float(ts) - prev_t, 1e-6)
        prev_t = float(ts)
        tgt = float(target.iloc[i]) if target is not None else None
        out.append(kf.update(float(val), dt=dt, target=tgt, tau=tau))
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv", nargs="?", default=str(_DEFAULT_CSV), help="Path to telemetry CSV.")
    parser.add_argument("--q", type=float, default=_Q_DEFAULT, metavar="Q",
                        help="Kalman process noise (default: %(default)s). Higher = faster response.")
    parser.add_argument("--r", type=float, default=_R_DEFAULT, metavar="R",
                        help="Kalman measurement noise (default: %(default)s). Higher = more smoothing.")
    parser.add_argument("--tau", type=float, default=_TAU_DEFAULT, metavar="TAU",
                        help="Plant time constant in seconds (e.g. 0.020). When set, the filter "
                             "predicts velocity heading toward the target at this rate, "
                             "reducing lag on commanded steps. Omit to use the default random-walk model.")
    args = parser.parse_args()

    csv_path = Path(args.csv)
    if not csv_path.exists():
        print(f"File not found: {csv_path}", file=sys.stderr)
        sys.exit(1)

    df = pd.read_csv(csv_path)

    has_step = "step" in df.columns

    target_col = df["target"] if "target" in df.columns else None

    df["left_smooth"]  = _apply_kalman(df["left_vel"],  df["t_s"], args.q, args.r, target_col, args.tau)
    df["right_smooth"] = _apply_kalman(df["right_vel"], df["t_s"], args.q, args.r, target_col, args.tau)

    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)

    _plot_wheel(axes[0], df, vel_col="left_vel",  smooth_col="left_smooth",
                title="Left wheel", has_step=has_step)
    _plot_wheel(axes[1], df, vel_col="right_vel", smooth_col="right_smooth",
                title="Right wheel", has_step=has_step)

    axes[1].set_xlabel("Time (s)")
    tau_str = f", tau={args.tau}" if args.tau is not None else ""
    fig.suptitle(f"Wheel velocity \u2014 {csv_path.name}  (q={args.q}, r={args.r}{tau_str})", fontsize=13)
    fig.tight_layout()
    plt.show()


def _plot_wheel(
    ax: "plt.Axes",
    df: pd.DataFrame,
    vel_col: str,
    smooth_col: str,
    title: str,
    has_step: bool,
) -> None:
    t = df["t_s"]

    ax.plot(t, df["target"],    color="silver",  linewidth=1.5, linestyle="--", label="Target",  zorder=2)
    ax.plot(t, df[vel_col],     color="#4db8ff", linewidth=0.8, alpha=0.6,      label="Measured", zorder=3)
    ax.plot(t, df[smooth_col],  color="#ff8c00", linewidth=1.8,                 label="Smoothed", zorder=4)

    if has_step:
        _draw_step_labels(ax, df)

    ax.set_ylabel("Velocity (rad/s)")
    ax.set_title(title)
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.axhline(0, color="black", linewidth=0.5, alpha=0.3)


def _draw_step_labels(ax: "plt.Axes", df: pd.DataFrame) -> None:
    """Draw a light vertical line and label at the start of each bench step."""
    prev_label: str | None = None
    y_top = ax.get_ylim()[1]
    for _, row in df.iterrows():
        label = str(row["step"])
        if label != prev_label:
            ax.axvline(float(row["t_s"]), color="gray", linewidth=0.6, linestyle=":", alpha=0.7)
            ax.text(
                float(row["t_s"]) + 0.05, y_top * 0.9,
                label, fontsize=6.5, color="gray", rotation=90, va="top",
            )
            prev_label = label


if __name__ == "__main__":
    main()