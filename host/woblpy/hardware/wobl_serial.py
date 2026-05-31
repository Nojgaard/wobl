"""Serial driver for the WOBL ESP32 firmware (comm_task protocol).

Wire format (from firmware/include/comm_task.hpp):
  [COBS-encoded payload] [0x00 delimiter]

Payload layout (before COBS):
  [type: u8] [body bytes...] [crc_hi: u8] [crc_lo: u8]

CRC: CRC-16/CCITT-FALSE — poly 0x1021, init 0xFFFF, no reflection.
Computed over [type byte] + [body bytes].

All struct fields are little-endian and packed (no padding).
"""

from __future__ import annotations

import queue
import struct
import threading
from typing import Optional

import serial
import serial.tools.list_ports

from woblpy.hardware import cobs
from woblpy.hardware.protocol import (CalibPayload, DriveCommand,
                                      DriveTelemetry, PoseCommand,
                                      PoseTelemetry, StatusData,
                                      WheelCalibPayload, WheelTuningPayload)

# ---------------------------------------------------------------------------
# Message type IDs (mirrors MsgType enum in comm_task.hpp)
# ---------------------------------------------------------------------------
_MSG_CMD_DRIVE = 0x01
_MSG_TELEM_DRIVE = 0x02
_MSG_CMD_POSE = 0x03
_MSG_TELEM_POSE = 0x04
_MSG_STATUS_REQ = 0x05
_MSG_STATUS = 0x06
_MSG_CALIB_WRITE = 0x07
_MSG_CALIB_ACK = 0x08
_MSG_CALIB_READ_REQ = 0x09
_MSG_CALIB_DATA = 0x0A
_MSG_WHEEL_CALIB_CMD = 0x0B
_MSG_WHEEL_CALIB_READ_REQ = 0x0C
_MSG_WHEEL_CALIB_DATA = 0x0D
_MSG_WHEEL_TUNING_WRITE = 0x0E
_MSG_WHEEL_TUNING_READ_REQ = 0x0F
_MSG_WHEEL_TUNING_DATA = 0x10

_WHEEL_TARGET_LEFT = 1
_WHEEL_TARGET_RIGHT = 2

# ---------------------------------------------------------------------------
# Packed struct formats (little-endian, matches __attribute__((packed)) layout)
# Sizes: DRIVE_CMD=10, DRIVE_TELEM=48, POSE_CMD=10, POSE_TELEM=26,
#        STATUS=26, CALIB_PAYLOAD=25, CALIB_ACK=1
# ---------------------------------------------------------------------------
_FMT_DRIVE_CMD = struct.Struct("<BfBf")       # enabled_l, vel_l, enabled_r, vel_r
_FMT_DRIVE_TELEM = struct.Struct("<11fI")     # 4 quat(xyzw) + 3 gyr + la + lv + ra + rv + ts_ms
_FMT_POSE_CMD = struct.Struct("<BfBf")        # enabled_l, pos_l, enabled_r, pos_r
_FMT_POSE_TELEM = struct.Struct("<BB6f")      # valid_l, valid_r, lp, lv, le, rp, rv, re
_FMT_STATUS = struct.Struct("<ifffiiBB")      # imu_st, imu_hz, foc_hz, whl_hz, l_st, r_st, l_ok, r_ok
_FMT_CALIB_PAYLOAD = struct.Struct("<B9i")    # target, gyr[3], acc[3], mag[3] — int32_t LSBs
_FMT_CALIB_ACK = struct.Struct("<B")          # success
_FMT_WHEEL_CALIB_CMD = struct.Struct("<B")    # target
_FMT_WHEEL_CALIB_DATA = struct.Struct("<Bfi") # target, zero angle, sensor direction
_FMT_WHEEL_TUNING_WRITE = struct.Struct("<B6fB") # target, p/i/d/tf/vel_limit/volt_limit, persist
_FMT_WHEEL_TUNING_DATA = struct.Struct("<B6f") # target, p/i/d/tf/vel_limit/volt_limit

# VIDs for common ESP32 USB-serial bridge chips
_ESP32_VIDS = {
    0x303A,  # Espressif native USB (ESP32-S2/S3/C3)
    0x10C4,  # Silicon Labs CP2102/CP2104
    0x1A86,  # WCH CH340/CH341
}


def _crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly=0x1021, init=0xFFFF, no reflection."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1
        crc &= 0xFFFF
    return crc


class WoblSerial:
    """Serial driver for the WOBL ESP32 firmware.

    A background thread continuously reads bytes from the serial port,
    splits on the 0x00 COBS delimiter, COBS-decodes and CRC-verifies each
    frame, then routes the body into a per-message-type SimpleQueue.

    step_drive() and step_pose() are independent blocking calls that send
    a command and wait for the corresponding telemetry reply.  They are
    safe to call from different threads or at different rates.
    """

    _RESPONSE_TIMEOUT = 0.5  # seconds

    def __init__(self, port: str) -> None:
        self._serial = serial.Serial(port, baudrate=1_000_000, timeout=0)
        self._running = True
        self._queues: dict[int, queue.SimpleQueue[bytes]] = {
            _MSG_TELEM_DRIVE: queue.SimpleQueue(),
            _MSG_TELEM_POSE: queue.SimpleQueue(),
            _MSG_STATUS: queue.SimpleQueue(),
            _MSG_CALIB_ACK: queue.SimpleQueue(),
            _MSG_CALIB_DATA: queue.SimpleQueue(),
            _MSG_WHEEL_CALIB_DATA: queue.SimpleQueue(),
            _MSG_WHEEL_TUNING_DATA: queue.SimpleQueue(),
        }
        self._tx_lock = threading.Lock()
        self._serial.reset_input_buffer()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    # ------------------------------------------------------------------
    # Construction helpers
    # ------------------------------------------------------------------

    @classmethod
    def open(cls, port: Optional[str] = None) -> "WoblSerial":
        """Open a connection, auto-detecting the ESP32 port when *port* is None."""
        if port is None:
            port = cls._detect_port()
        return cls(port)

    @staticmethod
    def _detect_port() -> str:
        candidates = [
            p.device
            for p in serial.tools.list_ports.comports()
            if p.vid in _ESP32_VIDS
        ]
        if not candidates:
            raise RuntimeError(
                "No ESP32 serial port found. "
                "Connect the device or pass --port explicitly."
            )
        if len(candidates) > 1:
            print(
                f"[WoblSerial] Multiple candidates: {candidates}; "
                f"using {candidates[0]}"
            )
        return candidates[0]

    # ------------------------------------------------------------------
    # Background reader
    # ------------------------------------------------------------------

    def _read_loop(self) -> None:
        buf = bytearray()
        while self._running:
            chunk = self._serial.read(256)
            if chunk:
                for b in chunk:
                    if b == 0x00:
                        if buf:
                            self._process_frame(bytes(buf))
                            buf.clear()
                    else:
                        buf.append(b)

    def _process_frame(self, raw: bytes) -> None:
        try:
            decoded = cobs.decode(raw)
        except Exception:
            return  # malformed frame (e.g. garbage from ESP32 boot output)
        if len(decoded) < 3:
            return
        expected = _crc16(decoded[:-2])
        received = (decoded[-2] << 8) | decoded[-1]
        if expected != received:
            return
        msg_type = decoded[0]
        body = decoded[1:-2]
        q = self._queues.get(msg_type)
        if q is not None:
            q.put(body)

    # ------------------------------------------------------------------
    # TX helper
    # ------------------------------------------------------------------

    def _send(self, msg_type: int, body: bytes = b"") -> None:
        payload = bytes([msg_type]) + body
        crc = _crc16(payload)
        payload += bytes([crc >> 8, crc & 0xFF])
        frame = cobs.encode(payload) + b"\x00"
        with self._tx_lock:
            self._serial.write(frame)

    def _recv(self, msg_type: int) -> bytes:
        try:
            return self._queues[msg_type].get(timeout=self._RESPONSE_TIMEOUT)
        except queue.Empty as exc:
            raise TimeoutError(
                f"No response for message type 0x{msg_type:02X} "
                f"within {self._RESPONSE_TIMEOUT}s"
            ) from exc

    # ------------------------------------------------------------------
    # Hardware protocol
    # ------------------------------------------------------------------

    def step_drive(self, cmd: DriveCommand) -> DriveTelemetry:
        body = _FMT_DRIVE_CMD.pack(
            int(cmd.left_enabled), cmd.left_velocity,
            int(cmd.right_enabled), cmd.right_velocity,
        )
        self._send(_MSG_CMD_DRIVE, body)
        raw = self._recv(_MSG_TELEM_DRIVE)
        x, y, z, w, gx, gy, gz, la, lv, ra, rv, ts = _FMT_DRIVE_TELEM.unpack(raw)
        return DriveTelemetry(
            quat_xyzw=(x, y, z, w),
            gyro=(gx, gy, gz),
            left_angle=la,
            left_vel=lv,
            right_angle=ra,
            right_vel=rv,
            timestamp_ms=ts,
        )

    def step_pose(self, cmd: PoseCommand) -> PoseTelemetry:
        body = _FMT_POSE_CMD.pack(
            int(cmd.left_enabled), cmd.left_pos_rad,
            int(cmd.right_enabled), cmd.right_pos_rad,
        )
        self._send(_MSG_CMD_POSE, body)
        raw = self._recv(_MSG_TELEM_POSE)
        lv, rv, lp, lvl, le, rp, rvl, re = _FMT_POSE_TELEM.unpack(raw)
        return PoseTelemetry(
            left_valid=bool(lv),
            right_valid=bool(rv),
            left_pos=lp,
            left_vel=lvl,
            left_effort=le,
            right_pos=rp,
            right_vel=rvl,
            right_effort=re,
        )

    def request_status(self) -> StatusData:
        self._send(_MSG_STATUS_REQ)
        raw = self._recv(_MSG_STATUS)
        imu_st, imu_hz, foc_hz, whl_hz, l_st, r_st, l_ok, r_ok = (
            _FMT_STATUS.unpack(raw)
        )
        return StatusData(
            imu_status=imu_st,
            imu_rate=imu_hz,
            foc_rate=foc_hz,
            wheel_rate=whl_hz,
            left_wheel_status=l_st,
            right_wheel_status=r_st,
            left_servo_ok=bool(l_ok),
            right_servo_ok=bool(r_ok),
        )

    def write_calib(self, target: int) -> bool:
        """Trigger the device to persist its current calibration state to NVS."""
        self._send(_MSG_CALIB_WRITE, bytes([target]))
        raw = self._recv(_MSG_CALIB_ACK)
        (success,) = _FMT_CALIB_ACK.unpack(raw)
        return bool(success)

    def read_calib(self, target: int) -> CalibPayload:
        self._send(_MSG_CALIB_READ_REQ, bytes([target]))
        raw = self._recv(_MSG_CALIB_DATA)
        t, gx, gy, gz, ax, ay, az, mx, my, mz = _FMT_CALIB_PAYLOAD.unpack(raw)
        return CalibPayload(
            target=t,
            gyro_offset=(gx, gy, gz),
            accel_offset=(ax, ay, az),
            mag_offset=(mx, my, mz),
        )

    def write_wheel_calibration(self, target: int) -> bool:
        if target not in (_WHEEL_TARGET_LEFT, _WHEEL_TARGET_RIGHT):
            raise ValueError("wheel target must be 1 (left) or 2 (right)")

        body = _FMT_WHEEL_CALIB_CMD.pack(target)
        self._send(_MSG_WHEEL_CALIB_CMD, body)
        raw = self._recv(_MSG_CALIB_ACK)
        (success,) = _FMT_CALIB_ACK.unpack(raw)
        return bool(success)

    def read_wheel_calibration(self, target: int) -> WheelCalibPayload:
        if target not in (_WHEEL_TARGET_LEFT, _WHEEL_TARGET_RIGHT):
            raise ValueError("wheel target must be 1 (left) or 2 (right)")

        self._send(_MSG_WHEEL_CALIB_READ_REQ, bytes([target]))
        raw = self._recv(_MSG_WHEEL_CALIB_DATA)
        t, zero_electric_angle, sensor_direction = _FMT_WHEEL_CALIB_DATA.unpack(raw)
        return WheelCalibPayload(
            target=t,
            zero_electric_angle=zero_electric_angle,
            sensor_direction=sensor_direction,
        )

    def write_wheel_tuning(
        self,
        target: int,
        *,
        p: float,
        i: float,
        d: float,
        lpf_velocity_tf: float,
        velocity_limit: float,
        voltage_limit: float,
        persist: bool = False,
    ) -> bool:
        if target not in (_WHEEL_TARGET_LEFT, _WHEEL_TARGET_RIGHT):
            raise ValueError("wheel target must be 1 (left) or 2 (right)")

        body = _FMT_WHEEL_TUNING_WRITE.pack(
            target,
            p,
            i,
            d,
            lpf_velocity_tf,
            velocity_limit,
            voltage_limit,
            int(persist),
        )
        self._send(_MSG_WHEEL_TUNING_WRITE, body)
        raw = self._recv(_MSG_CALIB_ACK)
        (success,) = _FMT_CALIB_ACK.unpack(raw)
        return bool(success)

    def read_wheel_tuning(self, target: int) -> WheelTuningPayload:
        if target not in (_WHEEL_TARGET_LEFT, _WHEEL_TARGET_RIGHT):
            raise ValueError("wheel target must be 1 (left) or 2 (right)")

        self._send(_MSG_WHEEL_TUNING_READ_REQ, bytes([target]))
        raw = self._recv(_MSG_WHEEL_TUNING_DATA)
        t, p, i, d, lpf_tf, velocity_limit, voltage_limit = _FMT_WHEEL_TUNING_DATA.unpack(raw)
        return WheelTuningPayload(
            target=t,
            p=p,
            i=i,
            d=d,
            lpf_velocity_tf=lpf_tf,
            velocity_limit=velocity_limit,
            voltage_limit=voltage_limit,
        )

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def close(self) -> None:
        self._running = False
        self._reader.join(timeout=1.0)
        self._serial.close()

    def __enter__(self) -> "WoblSerial":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
