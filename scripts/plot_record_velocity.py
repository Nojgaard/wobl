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
    ]

    recording = rr.dataframe.load_recording("data/wheel_test.rrd")
    view = recording.view(index="log_time", contents="/**").fill_latest_at()

    df = view.select(columns=wanted).read_pandas().reset_index()

    for col in wanted:
        df[col] = df[col].apply(extract_scalar)

    import matplotlib.pyplot as plt

    # Columns you want to plot (same "wanted" list you used earlier)
    wanted = [
        "/joint/command/velocity/left:Scalars:scalars",
        "/joint/command/velocity/right:Scalars:scalars",
        "/joint/state/velocity/left:Scalars:scalars",
        "/joint/state/velocity/right:Scalars:scalars",
    ]

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


if __name__ == "__main__":
    main(sys.argv)
