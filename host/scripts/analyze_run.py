import matplotlib.pyplot as plt
import numpy as np
from matplotlib import gridspec

from woblpy.record import load_as_dataframe


def main() -> None:
    df = load_as_dataframe("data/run_mk4.rrd")

    pitch = df["imu/pitch"].dropna()
    pitch_rate = df["imu/pitch_rate"].dropna()
    out_left = df["output/wheel/left"].dropna()
    out_right = df["output/wheel/right"].dropna()
    vel_body = df["body/forward_velocity"].dropna()
    vel_target = df["target/forward_velocity"].dropna()

    fig = plt.figure(figsize=(13, 10), tight_layout=True)
    fig.suptitle("Balance analysis – torque mode", fontweight="bold")
    gs = gridspec.GridSpec(4, 1, figure=fig)

    ax0 = fig.add_subplot(gs[0])
    ax0.plot(pitch.index, pitch.values, color="tab:orange", label="pitch (rad)")
    ax0.axhline(0, color="gray", linewidth=0.6, linestyle="--")
    ax0.set_ylabel("Pitch (rad)")
    ax0.legend(loc="upper right")
    ax0.grid(True, alpha=0.3)

    # Numerical derivative of pitch as a sanity-check against the gyro signal
    pitch_rate_derived = np.gradient(pitch.values, pitch.index.to_numpy())

    # Align derived and gyro signals on shared timestamps to compute scale ratio
    pr_on_pitch_idx = pitch_rate.reindex(pitch.index, method="nearest", tolerance=0.05)
    valid = ~np.isnan(pr_on_pitch_idx.values) & (np.abs(pitch_rate_derived) > 1e-6)
    if valid.sum() > 10:
        scale_ratio = np.median(
            pitch_rate_derived[valid] / pr_on_pitch_idx.values[valid]
        )
        print(f"d(pitch)/dt  vs  gyro scale ratio (median): {scale_ratio:.1f}x")
        print(
            f"  → gyro is {scale_ratio:.1f}x too small  (gyro_scale in imu.cpp is off by this factor)"
        )
    else:
        scale_ratio = 1.0

    ax1 = fig.add_subplot(gs[1], sharex=ax0)
    ax1.plot(
        pitch_rate.index,
        pitch_rate.values,
        color="tab:red",
        label="pitch rate",
    )
    ax1.axhline(0, color="gray", linewidth=0.6, linestyle="--")
    ax1.set_ylabel("Pitch rate (rad/s)")
    ax1.legend(loc="upper right")
    ax1.grid(True, alpha=0.3)

    ax2 = fig.add_subplot(gs[2], sharex=ax0)
    # Average the two wheel outputs to get net forward torque drive
    out_avg = (
        out_left.reindex(out_left.index.union(out_right.index)).ffill()
        + out_right.reindex(out_left.index.union(out_right.index)).ffill()
    ) / 2
    ax2.plot(
        out_left.index,
        out_left.values,
        color="tab:blue",
        alpha=0.6,
        label="output left",
    )
    ax2.plot(
        out_right.index,
        out_right.values,
        color="tab:cyan",
        alpha=0.6,
        label="output right",
    )
    ax2.plot(
        out_avg.index,
        out_avg.values,
        color="tab:purple",
        linewidth=1.5,
        label="output avg",
    )
    ax2.axhline(0, color="gray", linewidth=0.6, linestyle="--")
    ax2.set_ylabel("Wheel output")
    ax2.legend(loc="upper right")
    ax2.grid(True, alpha=0.3)

    ax3 = fig.add_subplot(gs[3], sharex=ax0)
    ax3.plot(
        vel_body.index, vel_body.values, color="tab:green", label="body fwd vel (m/s)"
    )
    ax3.plot(
        vel_target.index,
        vel_target.values,
        color="tab:olive",
        linestyle="--",
        label="target fwd vel (m/s)",
    )
    ax3.axhline(0, color="gray", linewidth=0.6, linestyle="--")
    ax3.set_ylabel("Fwd velocity (m/s)")
    ax3.set_xlabel("Time (s)")
    ax3.legend(loc="upper right")
    ax3.grid(True, alpha=0.3)

    # Print phase relationship to help spot oscillation cause
    common = pitch.index.intersection(out_avg.index)
    if len(common) > 10:
        p = pitch.reindex(common).ffill().values
        o = out_avg.reindex(common).ffill().values
        corr = np.corrcoef(p, o)[0, 1]
        print(
            f"Pitch–output correlation: {corr:+.3f}  "
            f"({'in-phase (proportional gain dominant)' if corr > 0.3 else 'out-of-phase (derivative/rate dominant)' if corr < -0.3 else 'weakly correlated'})"
        )

    plt.show()


if __name__ == "__main__":
    main()
