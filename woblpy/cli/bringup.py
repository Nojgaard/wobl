import argparse
import atexit
import os
import signal
import subprocess
import sys
import time

procs = []


def cleanup():
    """Gracefully shutdown all processes"""
    if not procs:
        return

    print("\nShutting down processes...")

    # Send SIGTERM to all processes
    for proc in procs:
        if proc.poll() is None:
            proc.terminate()

    # Wait for graceful shutdown
    start_time = time.time()
    while time.time() - start_time < 10:  # 10 second grace period
        if all(proc.poll() is not None for proc in procs):
            print("All processes terminated gracefully")
            break
        time.sleep(0.1)
    else:
        # Force kill remaining processes
        for proc in procs:
            if proc.poll() is None:
                proc.kill()

    procs.clear()
    print("All processes stopped.")


def signal_handler(sig, frame):
    cleanup()
    sys.exit(0)


def start_processes(commands, mode):
    """Start all processes for the given mode"""
    print(f"Starting {mode} mode...")

    env = os.environ.copy()
    env["WOBL_MODE"] = mode

    for cmd in commands:
        proc = subprocess.Popen(cmd, env=env, preexec_fn=os.setsid)
        procs.append(proc)

    print("Started processes. Press Ctrl+C to stop.")

    try:
        while procs:
            # Remove dead processes
            procs[:] = [p for p in procs if p.poll() is None]
            time.sleep(1)
        print("All processes exited")
    except KeyboardInterrupt:
        cleanup()


def main():
    # Setup signal handling
    atexit.register(cleanup)
    for sig in [signal.SIGINT, signal.SIGTERM]:
        signal.signal(sig, signal_handler)
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, signal_handler)

    # Parse arguments
    parser = argparse.ArgumentParser(description="Bringup script for WOBL robot")
    parser.add_argument("mode", choices=["real", "sim"])
    parser.add_argument(
        "--controller",
        choices=["default", "stand", "wheel_eval"],
        default="default",
        help="Choose controller type: default or stand controller",
    )
    parser.add_argument(
        "--record",
        action="store_true",
        help="Start a recording process (headless in real, with viewer in sim)",
    )
    args = parser.parse_args()

    # Define controller paths
    controller_paths = {
        "default": "woblpy/control/controller_node.py",
        "stand": "woblpy/control/stand_controller_node.py",
        "wheel_eval": "woblpy/control/wheel_eval_controller.py",
    }

    # Define commands for each mode
    commands = {
        "real": [
            [sys.executable, controller_paths[args.controller]],
            ["build/woblcpp-linux/scripts/imu_node"],
            ["build/woblcpp-linux/scripts/motor_node"],
        ],
        "sim": [
            [sys.executable, controller_paths[args.controller]],
            [sys.executable, "woblpy/sim/sim_node.py"],
        ],
    }

    # Optionally add recording process:
    if args.record:
        if args.mode == "real":
            # headless in real
            commands["real"].append(
                [sys.executable, "woblpy/cli/record.py", "--headless"]
            )
        else:
            # not headless in sim (launch viewer)
            commands["sim"].append([sys.executable, "woblpy/cli/record.py"])

    start_processes(commands[args.mode], args.mode)


if __name__ == "__main__":
    main()
