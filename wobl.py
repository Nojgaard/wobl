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


def start():
    controller = subprocess.Popen([sys.executable, "woblpy/control/controller_node.py"])
    sim = subprocess.Popen([sys.executable, "woblpy/sim/sim_node.py"])
    procs.extend([controller, sim])
    print("Started controller and simulation processes.")
    print("Press Ctrl+C to stop all processes")

    # Keep the main process running until Ctrl+C
    try:
        while True:
            # Check if any process has died unexpectedly
            for proc in procs:
                if proc.poll() is not None:
                    print(f"Process {proc.pid} exited unexpectedly")
            time.sleep(1)
    except KeyboardInterrupt:
        cleanup()


def stop():
    cleanup()


if __name__ == "__main__":
    # Register cleanup function to run on exit
    atexit.register(cleanup)
    signal.signal(signal.SIGINT, signal_handler)

    parser = argparse.ArgumentParser()
    parser.add_argument("cmd", choices=["start", "stop"])
    args = parser.parse_args()

    if args.cmd == "start":
        start()
    elif args.cmd == "stop":
        stop()
