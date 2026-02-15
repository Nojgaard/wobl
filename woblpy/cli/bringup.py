import argparse
import atexit
import os
import signal
import subprocess
import sys
import time

procs = []


def cleanup(grace=5):
    if not procs:
        return
    print("\nShutting down processes...")
    for p in list(procs):
        if p.poll() is None:
            try:
                if os.name == "posix":
                    os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                elif os.name == "nt":
                    try:
                        p.send_signal(signal.CTRL_BREAK_EVENT)
                    except Exception:
                        p.terminate()
                else:
                    p.terminate()
            except Exception:
                try:
                    p.kill()
                except Exception:
                    pass
    deadline = time.time() + grace
    while time.time() < deadline and any(p.poll() is None for p in procs):
        time.sleep(0.1)
    for p in list(procs):
        if p.poll() is None:
            try:
                if os.name == "posix":
                    os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                else:
                    p.kill()
            except Exception:
                pass
    procs.clear()
    print("All processes stopped.")


def _on_signal(sig, frame):
    cleanup()
    sys.exit(0)


def start_processes(cmds, mode):
    print(f"Starting {mode} mode...")
    base_env = os.environ.copy()
    base_env["WOBL_MODE"] = mode
    for cmd in cmds:
        kwargs = {"env": base_env}
        if os.name == "posix":
            kwargs["preexec_fn"] = os.setsid
        elif os.name == "nt":
            kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP  # type: ignore
        procs.append(subprocess.Popen(cmd, **kwargs))  # type: ignore
    print("Started processes. Press Ctrl+C to stop.")
    try:
        while any(p.poll() is None for p in procs):
            time.sleep(0.5)
        print("All processes exited")
    except KeyboardInterrupt:
        cleanup()


def main():
    atexit.register(cleanup)
    for s in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(s, _on_signal)
        except Exception:
            pass
    if hasattr(signal, "SIGHUP"):
        try:
            signal.signal(signal.SIGHUP, _on_signal)  # type: ignore
        except Exception:
            pass

    p = argparse.ArgumentParser(description="Bringup script for WOBL robot")
    p.add_argument("mode", choices=["real", "sim"])
    p.add_argument(
        "--controller",
        choices=["default", "stand", "wheel_eval"],
        default="default",
        help="Choose controller type: default or stand controller",
    )
    p.add_argument(
        "--record",
        action="store_true",
        help="Start a recording process (headless in real, with viewer in sim)",
    )
    args = p.parse_args()

    controller_paths = {
        "default": "woblpy/control/controller_node.py",
        "stand": "woblpy/control/stand_controller_node.py",
        "wheel_eval": "woblpy/control/wheel_eval_controller.py",
    }

    commands = {
        "real": [
            [sys.executable, controller_paths[args.controller]],
            ["build/woblcpp-linux/scripts/servo_node"],
            ["build/woblcpp-linux/scripts/imu_node"],
        ],
        "sim": [
            [sys.executable, controller_paths[args.controller]],
            [sys.executable, "woblpy/sim/sim_node.py"],
        ],
    }

    if args.record:
        commands["real"].append([sys.executable, "woblpy/cli/record.py", "--headless"])
        commands["sim"].append([sys.executable, "woblpy/cli/record.py"])

    start_processes(commands[args.mode], args.mode)


if __name__ == "__main__":
    main()
