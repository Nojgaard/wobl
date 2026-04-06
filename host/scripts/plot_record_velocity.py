import sys

import numpy as np
import pyarrow
import rerun_sdk.rerun as rr


def extract_scalar(x):
    if isinstance(x, (list, tuple, np.ndarray)):
        return x[0] if len(x) else None
    if isinstance(x, pyarrow.Array):
        return x[0] if len(x) else None
    return None


def main(argv):
    wanted = [
        "/joint/command/velocity/left:Scalars:scalars",
        "/joint/command/velocity/right:Scalars:scalars",
        "/joint/state/velocity/left:Scalars:scalars",
        "/joint/state/velocity/right:Scalars:scalars",
        "/joint/state/effort/left:Scalars:scalars",
        "/joint/state/effort/right:Scalars:scalars",
    ]

    recording = rr.dataframe.load_recording("data/wheel_test.rrd")
    view = recording.view(index="log_time", contents="/**").fill_latest_at()

    df = view.select(columns=wanted).read_pandas().reset_index()

    for col in wanted:
        df[col] = df[col].apply(extract_scalar)

    import matplotlib.pyplot as plt

    print(df)
    res = estimate_Kt_from_mass_df(df)
    print(res)

    plt.figure(figsize=(12, 6))

    for col in wanted:
        plt.plot(df["index"], df[col], label=col)

    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (rad/s)")
    plt.title("Joint Velocities from Recording")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


def estimate_Kt_from_mass_df(
    df,
    time_col="index",
    omega_col="/joint/state/velocity/left:Scalars:scalars",
    current_col="/joint/state/effort/left:Scalars:scalars",
    mass=0.35,  # total robot mass (kg) — replace with your value
    r=0.04,  # wheel radius (m)
    min_speed_for_sample=0.1,
):
    """
    Estimate Kt using the approximation tau ~ m * r^2 * alpha.
    df: pandas.DataFrame with time (s), omega (rad/s), current (A).
    """
    t = df[time_col].to_numpy()
    omega = df[omega_col].to_numpy()
    I_meas = df[current_col].to_numpy()

    # compute angular acceleration (central differences)
    alpha = np.gradient(omega, t)

    # torque estimate (simple mass-based)
    tau_est = mass * (r**2) * alpha

    # avoid division by tiny currents or noisy windows: choose samples with sufficient current and acceleration
    valid = (
        (np.abs(I_meas) > 0.05)
        & (np.abs(alpha) > 0.01)
        & (np.abs(omega) > min_speed_for_sample)
    )
    if np.sum(valid) < 10:
        print("Warning: few valid samples found — check thresholds or data quality")

    Kt_samples = tau_est[valid] / I_meas[valid]

    # robust statistic
    Kt_median = np.median(Kt_samples)
    Kt_mean = np.mean(Kt_samples)

    return {
        "Kt_median": Kt_median,
        "Kt_mean": Kt_mean,
        "Kt_samples": Kt_samples,
        "valid_count": int(np.sum(valid)),
    }


if __name__ == "__main__":
    main(sys.argv)
