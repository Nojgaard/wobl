"""Hardware-in-the-loop tests for the WOBL ESP32 firmware.

These tests require a physical ESP32 connected over USB serial.
They are excluded from the default test run and must be invoked explicitly:

    uv run pytest -m hardware
    uv run pytest -m hardware --port COM4

All tests keep wheel/servo outputs disabled — the robot does not need to be
in a stable position to run this suite.
"""

import math
import statistics
import time
from collections.abc import Generator

import pytest

from woblpy.hardware.protocol import DriveCommand, PoseCommand
from woblpy.hardware.wobl_serial import WoblSerial

# ---------------------------------------------------------------------------
# Session-scoped hardware fixture — one serial connection for all tests
# ---------------------------------------------------------------------------


@pytest.fixture(scope="session")
def hw(serial_port: str | None) -> Generator[WoblSerial, None, None]:
    """Open a WoblSerial connection for the entire test session."""
    device = WoblSerial.open(serial_port)
    yield device
    device.close()


# ---------------------------------------------------------------------------
# Connectivity / status
# ---------------------------------------------------------------------------


@pytest.mark.hardware
def test_status_responds(hw: WoblSerial) -> None:
    """Firmware must reply to MSG_STATUS_REQ within the default timeout."""
    status = hw.request_status()
    assert status is not None


@pytest.mark.hardware
def test_imu_initialised(hw: WoblSerial) -> None:
    """IMU status field must be zero (no error code)."""
    status = hw.request_status()
    assert status.imu_status == 0, (
        f"IMU reported error status {status.imu_status}"
    )


@pytest.mark.hardware
def test_task_rates(hw: WoblSerial) -> None:
    """IMU must be running >50 Hz and FOC loop >1000 Hz."""
    status = hw.request_status()
    assert status.imu_rate > 50.0, (
        f"IMU update rate too low: {status.imu_rate:.1f} Hz"
    )
    assert status.foc_rate > 1000.0, (
        f"FOC rate too low: {status.foc_rate:.1f} Hz"
    )


# ---------------------------------------------------------------------------
# IMU data sanity  (robot stationary, roughly upright or lying flat)
# ---------------------------------------------------------------------------


@pytest.mark.hardware
def test_quaternion_unit_length(hw: WoblSerial) -> None:
    """Orientation quaternion magnitude must be ~1.0 (not zeroed/garbled)."""
    cmd = DriveCommand(left_enabled=False, right_enabled=False)
    telem = hw.step_drive(cmd)
    x, y, z, w = telem.quat_xyzw
    magnitude = math.sqrt(w**2 + x**2 + y**2 + z**2)
    assert abs(magnitude - 1.0) < 0.05, (
        f"Quaternion magnitude {magnitude:.4f} deviates from 1.0"
    )


@pytest.mark.hardware
def test_gyro_stationary(hw: WoblSerial) -> None:
    """Gyro readings must be within ±0.5 rad/s when the robot is stationary."""
    cmd = DriveCommand(left_enabled=False, right_enabled=False)
    telem = hw.step_drive(cmd)
    for axis, value in zip("xyz", telem.gyro):
        assert abs(value) < 0.5, (
            f"Gyro {axis} = {value:.4f} rad/s exceeds stationary threshold"
        )


@pytest.mark.hardware
def test_pitch_roll_plausible(hw: WoblSerial) -> None:
    """Pitch and roll derived from the quaternion must be within ±90°."""
    import numpy as np
    from scipy.spatial.transform import Rotation as R

    cmd = DriveCommand(left_enabled=False, right_enabled=False)
    telem = hw.step_drive(cmd)
    x, y, z, w = telem.quat_xyzw
    roll, pitch, _ = R.from_quat([x, y, z, w]).as_euler("XYZ")
    assert abs(roll) < math.radians(90), f"Roll {math.degrees(roll):.1f}° out of range"
    assert abs(pitch) < math.radians(90), f"Pitch {math.degrees(pitch):.1f}° out of range"


# ---------------------------------------------------------------------------
# Drive round-trip latency
# ---------------------------------------------------------------------------


@pytest.mark.hardware
def test_drive_round_trip_latency(hw: WoblSerial) -> None:
    """20 consecutive drive round-trips must all complete under 20 ms."""
    cmd = DriveCommand(left_enabled=False, right_enabled=False)
    latencies: list[float] = []
    for _ in range(20):
        t0 = time.monotonic()
        hw.step_drive(cmd)
        latencies.append((time.monotonic() - t0) * 1000)

    p95 = sorted(latencies)[int(0.95 * len(latencies))]
    mean = statistics.mean(latencies)
    assert p95 < 20.0, (
        f"Drive round-trip p95 latency {p95:.1f} ms exceeds 20 ms "
        f"(mean={mean:.1f} ms)"
    )


@pytest.mark.hardware
def test_drive_timestamp_increasing(hw: WoblSerial) -> None:
    """Firmware timestamp must increase across consecutive drive replies."""
    cmd = DriveCommand(left_enabled=False, right_enabled=False)
    t0 = hw.step_drive(cmd).timestamp_ms
    time.sleep(0.05)
    t1 = hw.step_drive(cmd).timestamp_ms
    assert t1 > t0, (
        f"Firmware timestamp did not increase: {t0} → {t1}"
    )


# ---------------------------------------------------------------------------
# Pose round-trip
# ---------------------------------------------------------------------------


@pytest.mark.hardware
def test_pose_round_trip(hw: WoblSerial) -> None:
    """Pose command must elicit a response without timeout."""
    cmd = PoseCommand(left_enabled=False, right_enabled=False)
    telem = hw.step_pose(cmd)
    assert telem is not None


# ---------------------------------------------------------------------------
# CRC rejection — corrupt frame must not disrupt subsequent valid exchange
# ---------------------------------------------------------------------------


@pytest.mark.hardware
def test_corrupt_frame_ignored(hw: WoblSerial, serial_port: str | None) -> None:
    """A frame with a flipped CRC byte must be silently dropped.

    After injecting garbage, a valid STATUS_REQ must still receive a reply,
    confirming the firmware did not lock up or corrupt its RX state.
    """
    import serial as _serial

    from woblpy.hardware import cobs
    from woblpy.hardware.wobl_serial import _MSG_STATUS_REQ, _crc16

    # Build a valid STATUS_REQ payload, then corrupt the CRC
    payload = bytes([_MSG_STATUS_REQ])
    crc = _crc16(payload)
    # Flip the high byte of the CRC
    bad_crc_hi = (crc >> 8) ^ 0xFF
    corrupt_payload = payload + bytes([bad_crc_hi, crc & 0xFF])
    corrupt_frame = cobs.encode(corrupt_payload) + b"\x00"

    # Write directly to the underlying serial port
    hw._serial.write(corrupt_frame)
    time.sleep(0.02)

    # A valid STATUS_REQ must still get a response
    status = hw.request_status()
    assert status is not None, "Firmware failed to respond after a corrupt frame"
    assert status.imu_status == 0


# ---------------------------------------------------------------------------
# Calibration round-trip
# ---------------------------------------------------------------------------


@pytest.mark.hardware
def test_calib_read_write_round_trip(hw: WoblSerial) -> None:
    """Read current biases from the IMU, write them back, and verify the
    values are identical (safe no-op round-trip — biases do not change)."""
    original = hw.read_calib(target=0)

    success = hw.write_calib(target=0)
    assert success, "write_calib returned failure"

    readback = hw.read_calib(target=0)
    assert readback.gyro_offset == original.gyro_offset, (
        f"Gyro mismatch: {original.gyro_offset} → {readback.gyro_offset}"
    )
    assert readback.accel_offset == original.accel_offset, (
        f"Accel mismatch: {original.accel_offset} → {readback.accel_offset}"
    )
    assert readback.mag_offset == original.mag_offset, (
        f"Mag mismatch: {original.mag_offset} → {readback.mag_offset}"
    )
