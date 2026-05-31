import pandas as pd

from woblpy.control.diff_drive_kinematics import DiffDriveKinematics
from woblpy.record import load_as_dataframe


def print_fwd_velocity_window(df, fwd_velocity_cmd, t_start, t_end):
    # Print pitch, measured, and commanded forward velocity for a time window
    mask = (df.index >= t_start) & (df.index <= t_end)
    print("time,pitch,measured_fwd_velocity,cmd_fwd_velocity")
    for t, pitch, measured, cmd in zip(df.index[mask],
                                        df['imu/attitude/pitch'][mask],
                                        df['controller/fwd_velocity'][mask],
                                        fwd_velocity_cmd[mask]):
        print(f"{t},{pitch},{measured},{cmd}")
        
def main() -> None:
    import matplotlib.pyplot as plt
    df = load_as_dataframe("data/bringup.rrd")
    print("Available columns:")
    print(df.columns.tolist())
    print(df.head())

    drive = DiffDriveKinematics(0.3, 0.04, 20.0)
    # Plot key signals to diagnose oscillation
    fig, axs = plt.subplots(4, 1, figsize=(12, 12), sharex=True)

    # 1. Pitch angle
    if 'imu/attitude/pitch' in df.columns:
        axs[0].plot(df.index, df['imu/attitude/pitch'], label='Pitch (rad)')
        axs[0].set_ylabel('Pitch (rad)')
        
        axs[0].legend()
    else:
        axs[0].set_visible(False)

    # 2. Pitch rate
    if 'imu/gyro/x' in df.columns:
        axs[1].plot(df.index, df['imu/gyro/x'], label='Pitch Rate (rad/s)')
        axs[1].set_ylabel('Pitch Rate (rad/s)')
        axs[1].legend()
    else:
        axs[1].set_visible(False)

    # 3. Forward velocity
    if 'controller/fwd_velocity' in df.columns and \
       'wheel/cmd/left/velocity' in df.columns and 'wheel/cmd/right/velocity' in df.columns:
        # Compute commanded forward velocity for each row
        left = df['wheel/cmd/left/velocity'].to_numpy()
        right = df['wheel/cmd/right/velocity'].to_numpy()
        # Use vectorized computation for forward velocity
        fwd_velocity_cmd = (left + right) / 2 * drive.wheel_radius

        axs[2].plot(df.index, df['imu/attitude/pitch'], label='Pitch (rad)')
        axs[2].plot(df.index, df['controller/fwd_velocity'], label='Fwd Velocity (m/s)')
        axs[2].plot(df.index, fwd_velocity_cmd, label='Cmd Fwd Velocity (m/s)', linestyle='--')
        axs[2].set_ylabel('Fwd Velocity (m/s)')
        axs[2].legend()

        # Print values in the requested time window
        #print_fwd_velocity_window(df, fwd_velocity_cmd, 734.5, 738)
        print_fwd_velocity_window(df, fwd_velocity_cmd, 1300, 1400)
    else:
        axs[2].set_visible(False)

    # 4. Wheel velocities (left/right)
    plotted = False
    if 'wheel/cmd/left/velocity' in df.columns:
        axs[3].plot(df.index, df['wheel/cmd/left/velocity'], label='Left Wheel Cmd (rad/s)')
        axs[3].plot(df.index, df['wheel/telem/left/velocity'], label='Left Wheel Telem (rad/s)')
        plotted = True
    if 'wheel/cmd/right/velocity' in df.columns:
        axs[3].plot(df.index, df['wheel/cmd/right/velocity'], label='Right Wheel Cmd (rad/s)')
        axs[3].plot(df.index, df['wheel/telem/right/velocity'], label='Right Wheel Telem (rad/s)')
        plotted = True
    if plotted:
        axs[3].set_ylabel('Wheel Vel (rad/s)')
        axs[3].legend()
    else:
        axs[3].set_visible(False)

    axs[-1].set_xlabel('Timestamp (index)')
    plt.suptitle('Self-Balancing Robot: Key Signals')
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
