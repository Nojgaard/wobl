#!/usr/bin/env python3
"""Interactive wheel velocity PID tuning REPL for WOBL.

Commands
--------
  show                    — read back current tuning from both wheels
  set <param> <value>     — apply a parameter immediately (unpersisted)
  zero                    — zero all PID gains/LPF Tf (safe tuning start point)
  drive <velocity>        — spin both wheels at <velocity> rad/s, stream telemetry
  bench                   — run the hardcoded benchmark sequence (~24 s)
  stop                    — halt the wheels
  save                    — persist current values to NVS
  load                    — reload tuning from firmware NVS
  help                    — list commands
  quit / Ctrl-D           — exit

Parameters: p  i  d  ramp  tf  vel_lim  volt_lim

Usage
-----
    uv run scripts/wheel_tune.py [--port COM3] [--no-rerun]
"""

from __future__ import annotations

import argparse
import cmd
import csv
import dataclasses
import math
import sys
import threading
import time
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).parent.parent))

from woblpy.hardware.protocol import DriveCommand, WheelTuningPayload
from woblpy.hardware.wobl_serial import WoblSerial

WHEEL_LEFT = 1
WHEEL_RIGHT = 2

# ANSI colours
RESET  = "\033[0m"
BOLD   = "\033[1m"
GREEN  = "\033[32m"
YELLOW = "\033[33m"
RED    = "\033[31m"
CYAN   = "\033[36m"

# alias -> (WheelTuningPayload field name, human label)
_PARAMS: dict[str, tuple[str, str]] = {
    "p":        ("p",               "P gain"),
    "i":        ("i",               "I gain"),
    "d":        ("d",               "D gain"),
    "ramp":     ("output_ramp",     "Output ramp"),
    "tf":       ("lpf_velocity_tf", "LPF Tf"),
    "vel_lim":  ("velocity_limit",  "Vel limit"),
    "volt_lim": ("voltage_limit",   "Volt limit"),
}


# ---------------------------------------------------------------------------
# Drive thread
# ---------------------------------------------------------------------------

_DRIVE_PERIOD = 0.01  # 100 Hz

class _DriveThread(threading.Thread):
    """Background thread: spins one or both wheels and streams telemetry to
    the terminal (and optionally to a Rerun viewer)."""

    def __init__(
        self,
        hw: WoblSerial,
        velocity: float,
        target: str,
        rerun_enabled: bool,
        log_path: str | None = None,
        t_epoch: float = 0.0,
    ) -> None:
        super().__init__(daemon=True)
        self._hw = hw
        self._velocity = velocity
        self._target = target  # "left", "right", or "both"
        self._stop_event = threading.Event()
        self._rerun_enabled = rerun_enabled
        self._log_path = log_path
        self._t_epoch = t_epoch
        self._samples: list[dict] = []

    def stop(self) -> None:
        self._stop_event.set()

    def run(self) -> None:
        _rr: Any = None
        if self._rerun_enabled:
            import rerun as _rr  # type: ignore

        left_en  = self._target in ("left",  "both")
        right_en = self._target in ("right", "both")
        v = self._velocity
        drive_cmd = DriveCommand(
            left_enabled=left_en,   left_velocity=v  if left_en  else 0.0,
            right_enabled=right_en, right_velocity=v if right_en else 0.0,
        )
        stop_cmd = DriveCommand(
            left_enabled=False, left_velocity=0.0,
            right_enabled=False, right_velocity=0.0,
        )
        next_tick = time.perf_counter()

        try:
            while not self._stop_event.is_set():
                try:
                    telem = self._hw.step_drive(drive_cmd)
                except TimeoutError:
                    next_tick = time.perf_counter() + _DRIVE_PERIOD
                    continue

                t_s = time.perf_counter() - self._t_epoch
                lv, rv = telem.left_vel, telem.right_vel
                angle = telem.left_angle if left_en else telem.right_angle
                self._samples.append({
                    "t_s": t_s,
                    "left_vel": lv,
                    "right_vel": rv,
                    "left_angle": telem.left_angle,
                    "right_angle": telem.right_angle,
                    "target": v,
                })

                if left_en and right_en:
                    line = f"\r\033[2K  L: {lv:+7.3f}  R: {rv:+7.3f}  target: {v:+7.3f} rad/s"
                elif left_en:
                    line = f"\r\033[2K  L: {lv:+7.3f} A: {angle:+7.3f} target: {v:+7.3f} rad/s"
                else:
                    line = f"\r\033[2K  R: {rv:+7.3f}  target: {v:+7.3f} rad/s"
                sys.stdout.write(line)
                sys.stdout.flush()

                if _rr is not None:
                    _rr.set_time("t_s", duration=t_s)
                    _rr.log("wheel/velocity/target", _rr.Scalars(self._velocity))
                    _rr.log("wheel/velocity/left",   _rr.Scalars(lv))
                    _rr.log("wheel/velocity/right",  _rr.Scalars(rv))

                next_tick += _DRIVE_PERIOD
                sleep_for = next_tick - time.perf_counter()
                if sleep_for > 0:
                    time.sleep(sleep_for)
        finally:
            try:
                self._hw.step_drive(stop_cmd)
            except Exception:
                pass
            sys.stdout.write("\n")
            sys.stdout.flush()
            if self._log_path and self._samples:
                self._write_csv()

    def _write_csv(self) -> None:
        path = Path(self._log_path)  # type: ignore[arg-type]
        fields = list(self._samples[0].keys())
        with path.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=fields)
            w.writeheader()
            w.writerows(self._samples)
        print(f"{CYAN}  Logged {len(self._samples)} samples → {path}{RESET}")

    def print_stats(self) -> None:
        """Print velocity statistics for the completed run."""
        if not self._samples:
            return
        for wheel, key in (("Left", "left_vel"), ("Right", "right_vel")):
            vals = [s[key] for s in self._samples]
            n = len(vals)
            mean = sum(vals) / n
            std = math.sqrt(sum((x - mean) ** 2 for x in vals) / n)
            print(
                f"  {wheel:<6} vel  "
                f"mean={mean:+7.3f}  std={std:6.3f}  "
                f"min={min(vals):+7.3f}  max={max(vals):+7.3f}  "
                f"(n={n})"
            )


# ---------------------------------------------------------------------------
# Bench sequence
# ---------------------------------------------------------------------------

# (velocity_rad_s, duration_s, label)
_BENCH_SEQUENCE: list[tuple[float, float, str]] = [
    (0.0,  1.5,  "idle"),
    (3.0,  3.0,  "fwd"),
    (0.0,  0.5,  "settle"),
    (-3.0, 3.0,  "rev"),
    (0.0,  0.5,  "settle"),
    (5.0,  3.0,  "fwd_fast"),
    (0.0,  0.5,  "settle"),
    (-5.0, 3.0,  "rev_fast"),
    (0.0,  1.0,  "settle"),
    # rapid switching ±3 rad/s, 0.35 s per step
    (3.0,  0.35, "rapid+"), (-3.0, 0.35, "rapid-"),
    (3.0,  0.35, "rapid+"), (-3.0, 0.35, "rapid-"),
    (3.0,  0.35, "rapid+"), (-3.0, 0.35, "rapid-"),
    (3.0,  0.35, "rapid+"), (-3.0, 0.35, "rapid-"),
    (0.0,  1.0,  "done"),
]


class _BenchThread(threading.Thread):
    """Background thread: runs the hardcoded benchmark sequence and streams
    telemetry to the terminal (and optionally to a Rerun viewer)."""

    def __init__(
        self,
        hw: WoblSerial,
        target: str,
        rerun_enabled: bool,
        log_path: str | None = None,
        t_epoch: float = 0.0,
    ) -> None:
        super().__init__(daemon=True)
        self._hw = hw
        self._target = target  # "left", "right", or "both"
        self._stop_event = threading.Event()
        self._rerun_enabled = rerun_enabled
        self._log_path = log_path
        self._t_epoch = t_epoch
        self._samples: list[dict] = []

    def stop(self) -> None:
        self._stop_event.set()

    def run(self) -> None:
        _rr: Any = None
        if self._rerun_enabled:
            import rerun as _rr  # type: ignore

        left_en  = self._target in ("left",  "both")
        right_en = self._target in ("right", "both")

        stop_cmd = DriveCommand(
            left_enabled=False, left_velocity=0.0,
            right_enabled=False, right_velocity=0.0,
        )

        try:
            for v, duration, label in _BENCH_SEQUENCE:
                if self._stop_event.is_set():
                    break
                drive_cmd = DriveCommand(
                    left_enabled=left_en,   left_velocity=v  if left_en  else 0.0,
                    right_enabled=right_en, right_velocity=v if right_en else 0.0,
                )
                step_start = time.perf_counter()
                next_tick  = step_start

                while not self._stop_event.is_set():
                    try:
                        telem = self._hw.step_drive(drive_cmd)
                    except TimeoutError:
                        next_tick = time.perf_counter() + _DRIVE_PERIOD
                        continue

                    t_s = time.perf_counter() - self._t_epoch
                    lv, rv = telem.left_vel, telem.right_vel
                    self._samples.append({
                        "t_s":         t_s,
                        "left_vel":    lv,
                        "right_vel":   rv,
                        "left_angle":  telem.left_angle,
                        "right_angle": telem.right_angle,
                        "target":      v,
                        "step":        label,
                    })

                    if left_en and right_en:
                        line = (
                            f"\r\033[2K  [{label:<8}]  "
                            f"L: {lv:+7.3f}  R: {rv:+7.3f}  target: {v:+7.3f} rad/s"
                        )
                    elif left_en:
                        line = (
                            f"\r\033[2K  [{label:<8}]  "
                            f"L: {lv:+7.3f}  target: {v:+7.3f} rad/s"
                        )
                    else:
                        line = (
                            f"\r\033[2K  [{label:<8}]  "
                            f"R: {rv:+7.3f}  target: {v:+7.3f} rad/s"
                        )
                    sys.stdout.write(line)
                    sys.stdout.flush()

                    if _rr is not None:
                        _rr.set_time("t_s", duration=t_s)
                        _rr.log("wheel/velocity/target", _rr.Scalars(v))
                        _rr.log("wheel/velocity/left",   _rr.Scalars(lv))
                        _rr.log("wheel/velocity/right",  _rr.Scalars(rv))

                    next_tick += _DRIVE_PERIOD
                    sleep_for = next_tick - time.perf_counter()
                    if sleep_for > 0:
                        time.sleep(sleep_for)

                    if time.perf_counter() - step_start >= duration:
                        break
        finally:
            try:
                self._hw.step_drive(stop_cmd)
            except Exception:
                pass
            sys.stdout.write("\n")
            sys.stdout.flush()
            if self._log_path and self._samples:
                self._write_csv()

    def _write_csv(self) -> None:
        path = Path(self._log_path)  # type: ignore[arg-type]
        fields = list(self._samples[0].keys())
        with path.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=fields)
            w.writeheader()
            w.writerows(self._samples)
        print(f"{CYAN}  Logged {len(self._samples)} samples → {path}{RESET}")

    def print_stats(self) -> None:
        """Print per-step velocity statistics for the completed benchmark."""
        if not self._samples:
            return
        # Collect samples per step label, preserving sequence order
        steps: dict[str, list[dict]] = {}
        for s in self._samples:
            steps.setdefault(s["step"], []).append(s)
        print(f"\n  {'Step':<10} {'Wheel':<6} {'mean':>8} {'std':>7} {'min':>8} {'max':>8}  n")
        print(f"  {'─'*10} {'─'*6} {'─'*8} {'─'*7} {'─'*8} {'─'*8}  {'─'*4}")
        seen: set[str] = set()
        for s in self._samples:
            label = s["step"]
            if label in seen:
                continue
            seen.add(label)
            for wheel, key in (("Left", "left_vel"), ("Right", "right_vel")):
                vals = [x[key] for x in steps[label]]
                n    = len(vals)
                mean = sum(vals) / n
                std  = math.sqrt(sum((x - mean) ** 2 for x in vals) / n)
                print(
                    f"  {label:<10} {wheel:<6} "
                    f"{mean:>+8.3f} {std:>7.3f} "
                    f"{min(vals):>+8.3f} {max(vals):>+8.3f}  {n}"
                )
        print()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _header(text: str) -> None:
    print(f"\n{BOLD}{CYAN}{'─' * 56}{RESET}")
    print(f"{BOLD}{CYAN}  {text}{RESET}")
    print(f"{BOLD}{CYAN}{'─' * 56}{RESET}")


def _print_table(
    state: WheelTuningPayload,
    left: WheelTuningPayload | None,
    right: WheelTuningPayload | None,
) -> None:
    header = f"  {'Param':<12} {'State':>10} {'Left (fw)':>10} {'Right (fw)':>10}"
    print(f"{BOLD}{header}{RESET}")
    print(f"  {'─'*12} {'─'*10} {'─'*10} {'─'*10}")
    for alias, (field, _) in _PARAMS.items():
        sv = getattr(state, field)
        lv = getattr(left,  field) if left  is not None else None
        rv = getattr(right, field) if right is not None else None
        ls = f"{lv:.4f}" if lv is not None else "n/a"
        rs = f"{rv:.4f}" if rv is not None else "n/a"
        print(f"  {alias:<12} {sv:>10.4f} {ls:>10} {rs:>10}")
    print()


# ---------------------------------------------------------------------------
# REPL
# ---------------------------------------------------------------------------

class WheelTuneCLI(cmd.Cmd):
    # \001/\002 brackets tell readline not to count ANSI bytes in line length.
    prompt = f"\001{BOLD}\002tune> \001{RESET}\002"

    def __init__(self, hw: WoblSerial, drive_target: str, rerun_enabled: bool, log_path: str | None = None) -> None:
        super().__init__()
        self._hw = hw
        self._drive_target = drive_target  # "left", "right", or "both"
        self._rerun = rerun_enabled
        self._log_path = log_path
        self._rerun_initialized = False
        self._t_epoch: float = 0.0
        self._state = WheelTuningPayload()
        self._drive_thread: _DriveThread | _BenchThread | None = None

    @property
    def _is_driving(self) -> bool:
        return self._drive_thread is not None and self._drive_thread.is_alive()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def preloop(self) -> None:
        _header("WOBL Wheel Tuning REPL")
        print(f"  {CYAN}Params:{RESET}   p  i  d  ramp  tf  vel_lim  volt_lim")
        print(f"  {CYAN}Commands:{RESET} show  set  zero  wheel  drive  bench  stop  save  load  quit\n")
        try:
            self._state = self._hw.read_wheel_tuning(WHEEL_LEFT)
            print(f"{GREEN}✓ Initial tuning loaded from firmware.{RESET}")
        except Exception as exc:
            print(f"{YELLOW}⚠ Could not read initial tuning: {exc}{RESET}")
        self.do_show("")

    # ------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------

    def do_show(self, _arg: str) -> None:
        """Read back current tuning from both wheels and display a comparison table."""
        try:
            left  = self._hw.read_wheel_tuning(WHEEL_LEFT)
            right = self._hw.read_wheel_tuning(WHEEL_RIGHT)
        except Exception as exc:
            print(f"{RED}✗ Could not read tuning: {exc}{RESET}")
            return
        _print_table(self._state, left, right)

    def do_set(self, arg: str) -> None:
        """set <param> <value>  — apply immediately (unpersisted). Params: p i d tf vel_lim volt_lim"""
        parts = arg.split()
        if len(parts) != 2:
            print(f"{YELLOW}Usage: set <param> <value>{RESET}")
            return
        alias, raw = parts
        param = _PARAMS.get(alias)
        if param is None:
            print(f"{RED}Unknown param '{alias}'. Choose from: {', '.join(_PARAMS)}{RESET}")
            return
        field, label = param
        try:
            value = float(raw)
        except ValueError:
            print(f"{RED}Value must be a number.{RESET}")
            return
        setattr(self._state, field, value)
        if self._apply_state(persist=False):
            print(f"{GREEN}✓ {label} = {value:.4f}  (applied, not persisted){RESET}")
        else:
            print(f"{RED}✗ Write failed — check connection.{RESET}")

    def do_wheel(self, arg: str) -> None:
        """wheel <left|right|both>  — set the active drive target (tuning always applies to both)."""
        choice = arg.strip().lower()
        if choice not in ("left", "right", "both"):
            print(f"{YELLOW}Usage: wheel <left|right|both>  (current: {self._drive_target}){RESET}")
            return
        if self._is_driving:
            print(f"{YELLOW}Stop driving first before changing the wheel target.{RESET}")
            return
        self._drive_target = choice
        print(f"{GREEN}✓ Drive target: {choice}{RESET}")

    def do_drive(self, arg: str) -> None:
        """drive <velocity>  — spin the active wheel(s) at <velocity> rad/s and stream telemetry live."""
        if self._is_driving:
            print(f"{YELLOW}Already driving. Use 'stop' first.{RESET}")
            return
        try:
            v = float(arg)
        except ValueError:
            print(f"{RED}Usage: drive <velocity_rad_s>{RESET}")
            return
        if self._rerun:
            self._ensure_rerun()
        self._drive_thread = _DriveThread(
            self._hw, v, target=self._drive_target,
            rerun_enabled=self._rerun, log_path=self._log_path,
            t_epoch=self._t_epoch,
        )
        self._drive_thread.start()
        print(f"{GREEN}Driving {self._drive_target} at {v:.2f} rad/s — type 'stop' to halt{RESET}")

    def do_bench(self, _arg: str) -> None:
        """Run the hardcoded benchmark velocity sequence (~22 s) and stream telemetry."""
        if self._is_driving:
            print(f"{YELLOW}Already driving. Use 'stop' first.{RESET}")
            return
        if self._rerun:
            self._ensure_rerun()
        self._drive_thread = _BenchThread(
            self._hw, target=self._drive_target,
            rerun_enabled=self._rerun, log_path=self._log_path,
            t_epoch=self._t_epoch,
        )
        self._drive_thread.start()
        total = sum(d for _, d, _ in _BENCH_SEQUENCE)
        print(
            f"{GREEN}Benchmark started on {self._drive_target} "
            f"({total:.1f} s) — type 'stop' to abort early{RESET}"
        )

    def do_zero(self, _arg: str) -> None:
        """Zero all PID gains — safe starting point for tuning."""
        self._halt_drive()
        self._state.p = 0.0
        self._state.i = 0.0
        self._state.d = 0.0
        if self._apply_state(persist=False):
            print(f"{GREEN}✓ PID gains zeroed (applied, not persisted).{RESET}")
        else:
            print(f"{RED}✗ Write failed — check connection.{RESET}")
        self.do_show("")

    def do_stop(self, _arg: str) -> None:
        """Stop the wheels."""
        if self._halt_drive():
            print(f"{GREEN}✓ Wheels stopped.{RESET}")
        else:
            print(f"{YELLOW}Not currently driving.{RESET}")

    def do_save(self, _arg: str) -> None:
        """Persist current tuning values to NVS on the firmware."""
        if self._apply_state(persist=True):
            print(f"{GREEN}✓ Persisted to NVS.{RESET}")
        else:
            print(f"{RED}✗ Write failed — check connection.{RESET}")

    def do_load(self, _arg: str) -> None:
        """Reload tuning from firmware NVS and update local state."""
        try:
            self._state = self._hw.read_wheel_tuning(WHEEL_LEFT)
            print(f"{GREEN}✓ Tuning reloaded from firmware.{RESET}")
        except Exception as exc:
            print(f"{RED}✗ Could not read tuning: {exc}{RESET}")
            return
        self.do_show("")

    def do_quit(self, _arg: str) -> bool:  # type: ignore[override]
        """Exit the tuning REPL."""
        return True

    def do_EOF(self, arg: str) -> bool:  # type: ignore[override]
        print()
        return self.do_quit(arg)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _apply_state(self, persist: bool) -> bool:
        fields = {k: v for k, v in dataclasses.asdict(self._state).items() if k != "target"}
        try:
            ok_l = self._hw.write_wheel_tuning(target=WHEEL_LEFT,  **fields, persist=persist)
            ok_r = self._hw.write_wheel_tuning(target=WHEEL_RIGHT, **fields, persist=persist)
            return ok_l and ok_r
        except Exception:
            return False

    def _halt_drive(self) -> bool:
        """Stop the drive thread if running. Returns True if one was active."""
        t = self._drive_thread
        if t is not None and t.is_alive():
            t.stop()
            t.join(timeout=2.0)
            t.print_stats()
            self._drive_thread = None
            return True
        return False

    def _ensure_rerun(self) -> None:
        if self._rerun_initialized:
            return
        import rerun as rr  # type: ignore
        rr.init("wheel_tune", spawn=True)
        self._t_epoch = time.perf_counter()
        rr.log("wheel/velocity/target", rr.SeriesLines(names="Target", colors=[200, 200, 200]))
        rr.log("wheel/velocity/left",   rr.SeriesLines(names="Left",   colors=[0, 180, 255]))
        rr.log("wheel/velocity/right",  rr.SeriesLines(names="Right",  colors=[255, 140, 0]))
        self._rerun_initialized = True


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--port", default=None, help="Serial port (auto-detect if omitted).")
    parser.add_argument(
        "--wheel",
        choices=("left", "right", "both"),
        default="both",
        help="Initial drive target wheel (default: both). Can be changed at runtime with the 'wheel' command.",
    )
    parser.add_argument("--no-rerun", action="store_true", help="Disable Rerun visualisation.")
    parser.add_argument("--log", metavar="FILE", default=None, help="Write drive telemetry to a CSV file.")
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    hw = WoblSerial.open(port=args.port)
    cli = WheelTuneCLI(hw, drive_target=args.wheel, rerun_enabled=not args.no_rerun, log_path=args.log)
    try:
        cli.cmdloop()
    except KeyboardInterrupt:
        print()
    finally:
        cli._halt_drive()
        hw.close()
        print(f"{CYAN}Bye.{RESET}")


if __name__ == "__main__":
    main()
