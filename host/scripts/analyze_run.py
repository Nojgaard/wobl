from woblpy.control.diff_drive_kinematics import DiffDriveKinematics
from woblpy.record import load_as_dataframe

# Time window for aggregate stats (seconds)
STATS_WINDOW = (224.0, 226.0)


def print_stats(df, t_start: float, t_end: float) -> None:
    """Print mean ± std (min / max) for key columns in the given time window."""
    mask = (df.index >= t_start) & (df.index <= t_end)
    window = df[mask]

    if window.empty:
        print(f"\n[stats] No data in window [{t_start}, {t_end}]")
        return

    print(f"\n=== Aggregate Stats [{t_start}–{t_end}s] ({len(window)} samples) ===")
    print(f"{'Metric':<30} {'Mean':>10} {'Std':>10} {'Min':>10} {'Max':>10}")
    print("-" * 74)

    for col, label in [
        ("imu/attitude/pitch", "Pitch (rad)"),
        ("imu/gyro/x", "Pitch Rate (rad/s)"),
        ("controller/fwd_velocity", "Fwd Velocity (m/s)"),
        ("wheel/telem/left/velocity", "Left Wheel Vel (rad/s)"),
        ("wheel/telem/right/velocity", "Right Wheel Vel (rad/s)"),
        ("wheel/telem/left/current", "Left Current (A)"),
        ("wheel/telem/right/current", "Right Current (A)"),
    ]:
        if col in window.columns:
            s = window[col].dropna()
            if len(s) > 0:
                print(
                    f"{label:<30} {s.mean():>10.4f} {s.std():>10.4f} {s.min():>10.4f} {s.max():>10.4f}"
                )
            else:
                print(f"{label:<30} {'(all NaN)':>44}")
    print()


def main() -> None:
    import matplotlib.pyplot as plt

    df = load_as_dataframe("data/bringup.rrd")
    print("Available columns:")
    print(df.columns.tolist())
    print(df.head())

    drive = DiffDriveKinematics(0.3, 0.04, 20.0)
    # Plot key signals to diagnose oscillation
    fig, axs = plt.subplots(5, 1, figsize=(12, 14), sharex=True)

    # 1. Pitch angle
    if "imu/attitude/pitch" in df.columns:
        axs[0].plot(df.index, df["imu/attitude/pitch"], label="Pitch (rad)")
        axs[0].set_ylabel("Pitch (rad)")

        axs[0].legend()
    else:
        axs[0].set_visible(False)

    # 2. Pitch rate
    if "imu/gyro/x" in df.columns:
        axs[1].plot(df.index, df["imu/gyro/x"], label="Pitch Rate (rad/s)")
        axs[1].set_ylabel("Pitch Rate (rad/s)")
        axs[1].legend()
    else:
        axs[1].set_visible(False)

    # 3. Forward velocity
    if (
        "controller/fwd_velocity" in df.columns
        and "wheel/cmd/left/velocity" in df.columns
        and "wheel/cmd/right/velocity" in df.columns
    ):
        # Compute commanded forward velocity for each row
        left = df["wheel/cmd/left/velocity"].to_numpy()
        right = df["wheel/cmd/right/velocity"].to_numpy()
        # Use vectorized computation for forward velocity
        fwd_velocity_cmd = (left + right) / 2 * drive.wheel_radius

        axs[2].plot(df.index, df["imu/attitude/pitch"], label="Pitch (rad)")
        axs[2].plot(df.index, df["controller/fwd_velocity"], label="Fwd Velocity (m/s)")
        axs[2].plot(
            df.index, fwd_velocity_cmd, label="Cmd Fwd Velocity (m/s)", linestyle="--"
        )
        axs[2].set_ylabel("Fwd Velocity (m/s)")
        axs[2].legend()

    else:
        axs[2].set_visible(False)

    # 4. Wheel velocities (left/right)
    plotted = False
    if "wheel/cmd/left/velocity" in df.columns:
        axs[3].plot(
            df.index, df["wheel/cmd/left/velocity"], label="Left Wheel Cmd (rad/s)"
        )
        axs[3].plot(
            df.index, df["wheel/telem/left/velocity"], label="Left Wheel Telem (rad/s)"
        )
        plotted = True
    if "wheel/cmd/right/velocity" in df.columns:
        axs[3].plot(
            df.index, df["wheel/cmd/right/velocity"], label="Right Wheel Cmd (rad/s)"
        )
        axs[3].plot(
            df.index,
            df["wheel/telem/right/velocity"],
            label="Right Wheel Telem (rad/s)",
        )
        plotted = True
    if plotted:
        axs[3].set_ylabel("Wheel Vel (rad/s)")
        axs[3].legend()
    else:
        axs[3].set_visible(False)

    # 5. Wheel current setpoints (left/right)
    if (
        "wheel/telem/left/current" in df.columns
        and "wheel/telem/right/current" in df.columns
    ):
        axs[4].plot(df.index, df["wheel/telem/left/current"], label="Left Current (A)")
        axs[4].plot(
            df.index, df["wheel/telem/right/current"], label="Right Current (A)"
        )
        axs[4].set_ylabel("Current (A)")
        axs[4].legend()
    else:
        axs[4].set_visible(False)

    axs[-1].set_xlabel("Timestamp (index)")
    plt.suptitle("Self-Balancing Robot: Key Signals")
    plt.tight_layout()

    print_stats(df, *STATS_WINDOW)

    plt.show()


if __name__ == "__main__":
    main()
