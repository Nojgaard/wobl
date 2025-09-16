import argparse
import atexit
import signal
import subprocess
import sys
import time

procs = []


def cleanup():
    """Clean up all running processes"""
    if not procs:
        return

    print("\nShutting down processes...")
    for proc in procs[:]:  # Copy list to avoid modification during iteration
        if proc.poll() is None:  # Process is still running
            print(f"Terminating process {proc.pid}")
            proc.terminate()
            try:
                proc.wait(timeout=5)
                print(f"Process {proc.pid} terminated gracefully")
            except subprocess.TimeoutExpired:
                print(f"Force killing process {proc.pid}")
                proc.kill()
                proc.wait()  # Ensure it's really dead
    procs.clear()
    print("All processes stopped.")


def signal_handler(sig, frame):
    """Handle Ctrl+C"""
    cleanup()
    sys.exit(0)


def spin_procs():
    print("Started controller, IMU node, and servo node.")
    print("Press Ctrl+C to stop all processes")
    try:
        while True:
            # Check if any process has died unexpectedly
            for proc in procs:
                if proc.poll() is not None:
                    print(f"Process {proc.pid} exited unexpectedly")
            time.sleep(1)
    except KeyboardInterrupt:
        cleanup()


def start_real():
    """Start processes for real hardware"""
    controller = subprocess.Popen([sys.executable, "woblpy/control/controller_node.py"])
    imu_node = subprocess.Popen(["build/woblcpp-linux/scripts/imu_node"])
    servo_node = subprocess.Popen(["build/woblcpp-linux/scripts/servo_node"])
    procs.extend([controller, imu_node, servo_node])
    spin_procs()


def start_sim():
    """Start processes for simulation"""
    controller = subprocess.Popen([sys.executable, "woblpy/control/controller_node.py"])
    sim = subprocess.Popen([sys.executable, "woblpy/sim/sim_node.py"])
    procs.extend([controller, sim])
    spin_procs()


if __name__ == "__main__":
    # Register cleanup function to run on exit
    atexit.register(cleanup)
    signal.signal(signal.SIGINT, signal_handler)

    parser = argparse.ArgumentParser(description="Bringup script for WOBL robot")
    parser.add_argument(
        "mode",
        choices=["real", "sim"],
        help="Mode to run: 'real' for hardware, 'sim' for simulation",
    )
    args = parser.parse_args()

    if args.mode == "real":
        start_real()
    elif args.mode == "sim":
        start_sim()
