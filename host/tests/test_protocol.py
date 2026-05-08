"""Tests for wire protocol pack / unpack.

Verifies that the struct format strings in wobl_serial match the sizes
documented in firmware/include/comm_task.hpp, and that round-tripping a
known command or telemetry value through pack → unpack preserves all fields.
"""

import struct

import pytest

from woblpy.hardware.wobl_serial import (
    _FMT_CALIB_ACK,
    _FMT_CALIB_PAYLOAD,
    _FMT_DRIVE_CMD,
    _FMT_DRIVE_TELEM,
    _FMT_POSE_CMD,
    _FMT_POSE_TELEM,
    _FMT_STATUS,
)


# ---------------------------------------------------------------------------
# Struct size checks (ground-truth from comm_task.hpp)
# ---------------------------------------------------------------------------

def test_drive_cmd_size() -> None:
    assert _FMT_DRIVE_CMD.size == 10


def test_drive_telem_size() -> None:
    assert _FMT_DRIVE_TELEM.size == 48


def test_pose_cmd_size() -> None:
    assert _FMT_POSE_CMD.size == 10


def test_pose_telem_size() -> None:
    assert _FMT_POSE_TELEM.size == 26


def test_status_size() -> None:
    assert _FMT_STATUS.size == 26


def test_calib_payload_size() -> None:
    assert _FMT_CALIB_PAYLOAD.size == 25


def test_calib_ack_size() -> None:
    assert _FMT_CALIB_ACK.size == 1


# ---------------------------------------------------------------------------
# DriveCommand round-trip
# ---------------------------------------------------------------------------

def test_drive_cmd_pack_unpack() -> None:
    packed = _FMT_DRIVE_CMD.pack(1, 1.5, 0, -2.0)
    assert len(packed) == 10
    en_l, vel_l, en_r, vel_r = _FMT_DRIVE_CMD.unpack(packed)
    assert en_l == 1
    assert abs(vel_l - 1.5) < 1e-6
    assert en_r == 0
    assert abs(vel_r - (-2.0)) < 1e-6


def test_drive_cmd_disabled() -> None:
    packed = _FMT_DRIVE_CMD.pack(0, 0.0, 0, 0.0)
    en_l, vel_l, en_r, vel_r = _FMT_DRIVE_CMD.unpack(packed)
    assert en_l == 0 and en_r == 0
    assert vel_l == pytest.approx(0.0)
    assert vel_r == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# DriveTelemetry round-trip
# ---------------------------------------------------------------------------

def test_drive_telem_pack_unpack() -> None:
    values = (1.0, 0.0, 0.0, 0.0,   # quat wxyz
              0.1, -0.2, 0.3,        # gyro xyz
              0.5, 1.2, -0.5, -1.2,  # la, lv, ra, rv
              12345)                  # timestamp_ms
    packed = _FMT_DRIVE_TELEM.pack(*values)
    assert len(packed) == 48
    unpacked = _FMT_DRIVE_TELEM.unpack(packed)
    assert unpacked[-1] == 12345
    for a, b in zip(unpacked[:-1], values[:-1]):
        assert abs(a - b) < 1e-6


# ---------------------------------------------------------------------------
# PoseTelemetry round-trip
# ---------------------------------------------------------------------------

def test_pose_telem_pack_unpack() -> None:
    packed = _FMT_POSE_TELEM.pack(1, 0, 0.3, 0.1, 25.0, -0.3, -0.1, 30.0)
    assert len(packed) == 26
    lv, rv, lp, lvl, le, rp, rvl, re = _FMT_POSE_TELEM.unpack(packed)
    assert lv == 1 and rv == 0
    assert abs(lp - 0.3) < 1e-6
    assert abs(le - 25.0) < 1e-6


# ---------------------------------------------------------------------------
# StatusData round-trip
# ---------------------------------------------------------------------------

def test_status_pack_unpack() -> None:
    packed = _FMT_STATUS.pack(0, 100.0, 4000.0, 100.0, 0, 0, 1, 1)
    assert len(packed) == 26
    imu_st, imu_hz, foc_hz, whl_hz, l_st, r_st, l_ok, r_ok = (
        _FMT_STATUS.unpack(packed)
    )
    assert abs(foc_hz - 4000.0) < 1e-3
    assert l_ok == 1 and r_ok == 1


# ---------------------------------------------------------------------------
# CalibPayload round-trip
# ---------------------------------------------------------------------------

def test_calib_payload_pack_unpack() -> None:
    packed = _FMT_CALIB_PAYLOAD.pack(2, 0.01, -0.02, 0.03, 0.1, -0.2, 0.3)
    assert len(packed) == 25
    target, gx, gy, gz, ax, ay, az = _FMT_CALIB_PAYLOAD.unpack(packed)
    assert target == 2
    assert abs(gz - 0.03) < 1e-6
    assert abs(az - 0.3) < 1e-6
