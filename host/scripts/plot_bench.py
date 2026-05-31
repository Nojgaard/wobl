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
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).parent.parent))

from woblpy.control.kalman_filter import KalmanFilter

_DEFAULT_CSV = Path(__file__).parent.parent / "bench.csv"
_Q_DEFAULT = 2.0    # process noise — tune upward for faster response
_R_DEFAULT = 0.25   # measurement noise ≈ variance of encoder noise (σ≈0.54 rad/s on bench data)
#_TAU_DEFAULT = 0.02  # plant time constant in seconds. Measured 63%-rise time of ~20 ms on bench data.
_TAU_DEFAULT = None

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

    _print_summary(df)

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


def _print_summary(df: pd.DataFrame) -> None:
    """Print a short noise-and-lag summary to stdout."""
    print("\n\u2500\u2500 Telemetry summary " + "\u2500" * 44)
    cols = [
        ("Left",  "left_vel",  "left_smooth"),
        ("Right", "right_vel", "right_smooth"),
    ]
    t = df["t_s"].to_numpy(dtype=float)
    has_target = "target" in df.columns
    target = df["target"].to_numpy(dtype=float) if has_target else None

    def compute_noise_and_lag(signal: np.ndarray, t: np.ndarray, target: np.ndarray | None) -> tuple[float, str, float]:
        if target is None:
            return float('nan'), "N/A (no target)", 0.0
        # Lag: 63%-rise time across every step transition
        lag_ms: list[float] = []
        transitions = np.flatnonzero(np.diff(target) != 0)
        for idx in transitions:
            v_start   = signal[idx]
            v_end     = target[idx + 1]
            step_size = v_end - v_start
            if abs(step_size) < 0.05:
                continue
            threshold = v_start + 0.63 * step_size
            end = int(np.searchsorted(t, t[idx] + 2.0))
            window_s = signal[idx + 1 : end]
            window_t = t[idx + 1 : end]
            if step_size > 0:
                cross = np.flatnonzero(window_s >= threshold)
            else:
                cross = np.flatnonzero(window_s <= threshold)
            if cross.size:
                lag_ms.append((window_t[cross[0]] - t[idx]) * 1000.0)
        if lag_ms:
            mean_lag = float(np.mean(lag_ms))
            std_lag  = float(np.std(lag_ms))
            lag_str  = f"{mean_lag:5.1f} \u00b1 {std_lag:.1f} ms  (n={len(lag_ms)})"
        else:
            mean_lag = 0.0
            lag_str  = "N/A (no step transitions found)"
        # Noise: std dev of residuals (lag-compensated signal − target)
        # Shift signal by mean lag (in seconds) using interpolation
        lag_s = mean_lag / 1000.0
        t_shifted = t - lag_s
        # Interpolate signal to shifted time base
        signal_shifted = np.interp(t, t_shifted, signal, left=np.nan, right=np.nan)
        # Only compare where both are valid
        valid = ~np.isnan(signal_shifted)
        noise_std = float(np.std(signal_shifted[valid] - target[valid]))
        return noise_std, lag_str, mean_lag

    for label, vel_col, smooth_col in cols:
        measured  = df[vel_col].to_numpy(dtype=float)
        smoothed  = df[smooth_col].to_numpy(dtype=float)

        # Measured velocity vs target
        noise_meas, lag_meas, lag_val_meas = compute_noise_and_lag(measured, t, target)
        # Smoothed velocity vs target
        noise_smooth, lag_smooth, lag_val_smooth = compute_noise_and_lag(smoothed, t, target)

        print(f"  {label:5s}  measured: noise \u03c3 = {noise_meas:.4f} rad/s    63%-rise lag = {lag_meas}")
        print(f"         smoothed: noise \u03c3 = {noise_smooth:.4f} rad/s    63%-rise lag = {lag_smooth}")

    print("\u2500" * 63 + "\n")


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