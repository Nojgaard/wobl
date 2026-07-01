#include "comm_task.hpp"
#include "cobs.hpp"

#include <Arduino.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
static constexpr uint32_t kBaudRate = 1000000;
static constexpr uint32_t kHeartbeatTimeoutMs = 200;
static constexpr size_t kRxBufSize = 256;
static constexpr float kMaxWheelVelocity = 50.0f; // rad/s

// ---------------------------------------------------------------------------
// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflect)
// Computed over [type byte][body bytes].
// ---------------------------------------------------------------------------
static uint16_t crc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int j = 0; j < 8; j++) {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
  }
  return crc;
}

// ---------------------------------------------------------------------------
// Packet TX helper
// Payload layout (before COBS): [type][body...][crc_hi][crc_lo]
// ---------------------------------------------------------------------------
// COBS overhead: at most ceil(n/254) extra bytes; +1 is sufficient for payloads
// ≤ 254 bytes, +2 covers up to 508 bytes. kRxBufSize=256 → max encoded = 258.
static constexpr size_t kMaxCobsEncodedSize = kRxBufSize + 2;
static uint8_t txEncoded[kMaxCobsEncodedSize];

static void sendPacket(uint8_t type, const void *body, size_t bodyLen) {
  // Build raw payload in a local buffer
  uint8_t payload[kRxBufSize];
  size_t payloadLen = 1 + bodyLen + 2;

  payload[0] = type;
  if (body && bodyLen > 0) {
    memcpy(payload + 1, body, bodyLen);
  }

  uint16_t crc = crc16(payload, 1 + bodyLen);
  payload[1 + bodyLen] = (uint8_t)(crc >> 8);
  payload[1 + bodyLen + 1] = (uint8_t)(crc & 0xFF);

  size_t encodedLen = cobs_encode(payload, payloadLen, txEncoded);
  Serial.write(txEncoded, encodedLen);
  Serial.write((uint8_t)0x00);
}

// ---------------------------------------------------------------------------
// Handlers
// ---------------------------------------------------------------------------
static void handleDriveCmd(SharedState *state, const uint8_t *body,
                           size_t len) {
  if (len != sizeof(WireDriveCmd))
    return;

  WireDriveCmd cmd;
  memcpy(&cmd, body, sizeof(cmd));

  // Clamp velocities before writing to shared state
  auto clamp = [](float v, float limit) -> float {
    return v < -limit ? -limit : (v > limit ? limit : v);
  };

  WheelsCommand wc;
  wc.left.enabled = cmd.leftWheel.enabled != 0;
  wc.left.velocity = clamp(cmd.leftWheel.velocity, kMaxWheelVelocity);
  wc.right.enabled = cmd.rightWheel.enabled != 0;
  wc.right.velocity = clamp(cmd.rightWheel.velocity, kMaxWheelVelocity);
  state->commands.wheels.write(wc);
}

static void sendDriveTelem(SharedState *state) {
  IMU::Data imu = state->telemetry.imu.read();
  WheelsData wheels = state->telemetry.wheels.read();

  WireDriveTelem telem;
  telem.quatXYZW[0] = imu.orientation[0]; // ICM stores [q1,q2,q3,q0] → x
  telem.quatXYZW[1] = imu.orientation[1]; // y
  telem.quatXYZW[2] = imu.orientation[2]; // z
  telem.quatXYZW[3] = imu.orientation[3]; // w
  telem.gyr[0] = imu.gyr[0];
  telem.gyr[1] = imu.gyr[1];
  telem.gyr[2] = imu.gyr[2];
  telem.leftAngle = wheels.left.angle;
  telem.leftVel = wheels.left.velocity;
  telem.leftCurrent = wheels.left.current;
  telem.rightAngle = wheels.right.angle;
  telem.rightVel = wheels.right.velocity;
  telem.rightCurrent = wheels.right.current;
  telem.timestampMs = (uint32_t)millis();

  sendPacket(MSG_TELEM_DRIVE, &telem, sizeof(telem));
}

static void handlePoseCmd(SharedState *state, const uint8_t *body, size_t len) {
  if (len != sizeof(WirePoseCmd))
    return;

  WirePoseCmd cmd;
  memcpy(&cmd, body, sizeof(cmd));

  ServosCommand sc;
  sc.left.enabled = cmd.leftServo.enabled != 0;
  sc.left.positionRad = cmd.leftServo.positionRad;
  sc.right.enabled = cmd.rightServo.enabled != 0;
  sc.right.positionRad = cmd.rightServo.positionRad;
  state->commands.servos.write(sc);
}

static void sendPoseTelem(SharedState *state) {
  ServosData servos = state->telemetry.servos.read();

  WirePoseTelem telem;
  telem.leftValid = servos.left.valid ? 1 : 0;
  telem.rightValid = servos.right.valid ? 1 : 0;
  telem.leftPos = servos.left.positionRad;
  telem.leftVel = servos.left.velocityRps;
  telem.leftEffort = servos.left.effortPct;
  telem.rightPos = servos.right.positionRad;
  telem.rightVel = servos.right.velocityRps;
  telem.rightEffort = servos.right.effortPct;

  sendPacket(MSG_TELEM_POSE, &telem, sizeof(telem));
}

static void handleStatusReq(SharedState *state, size_t bodyLen) {
  if (bodyLen != 0)
    return;

  IMUStatus imuSt = state->status.imu.read();
  WheelsStatus wheelSt = state->status.wheels.read();
  ServosStatus servoSt = state->status.servos.read();

  WireStatus s;
  s.imuStatus = imuSt.status;
  s.imuUpdateRate = imuSt.updateRate;
  s.wheelsFocRate = wheelSt.focRate;
  s.wheelsUpdateRate = wheelSt.updateRate;
  s.wheelsLeftStatus = wheelSt.leftStatus;
  s.wheelsRightStatus = wheelSt.rightStatus;
  s.servoLeftOk = servoSt.leftOk ? 1 : 0;
  s.servoRightOk = servoSt.rightOk ? 1 : 0;

  sendPacket(MSG_STATUS, &s, sizeof(s));
}

static void handleCalibWrite(SharedState *state, const uint8_t *body, size_t len) {
  if (len != 1)
    return;

  uint8_t target = body[0];
  uint8_t success = 0;

  if (target == 0) { // IMU
    state->calibration.imuReq.write(CalibReq{true});
    success = 1;
  }

  WireCalibAck ack = {success};
  sendPacket(MSG_CALIB_ACK, &ack, sizeof(ack));
}

static void handleCalibReadReq(SharedState *state, const uint8_t *body, size_t len) {
  if (len != 1)
    return;

  WireCalibPayload payload = {};
  payload.target = body[0];

  if (payload.target == 0) { // IMU
    IMU::Calibration cal = state->calibration.imuData.read();
    memcpy(payload.gyroOffset,  cal.gyro,  sizeof(cal.gyro));
    memcpy(payload.accelOffset, cal.accel, sizeof(cal.accel));
    memcpy(payload.magOffset,   cal.mag,   sizeof(cal.mag));
  }

  sendPacket(MSG_CALIB_DATA, &payload, sizeof(payload));
}

static bool isWheelTargetValid(uint8_t target) {
  return target == WHEEL_TARGET_LEFT || target == WHEEL_TARGET_RIGHT;
}

static void handleWheelCalibCmd(SharedState *state, const uint8_t *body, size_t len) {
  if (len != sizeof(WireWheelCalibCmd))
    return;

  WireWheelCalibCmd cmd;
  memcpy(&cmd, body, sizeof(cmd));

  uint8_t success = 0;
  if (isWheelTargetValid(cmd.target)) {
    CalibReq req;
    req.pending = true;

    if (cmd.target == WHEEL_TARGET_LEFT) {
      state->calibration.leftWheelCalibReq.write(req);
    } else {
      state->calibration.rightWheelCalibReq.write(req);
    }
    success = 1;
  }

  WireCalibAck ack = {success};
  sendPacket(MSG_CALIB_ACK, &ack, sizeof(ack));
}

static void handleWheelCalibReadReq(SharedState *state, const uint8_t *body, size_t len) {
  if (len != 1)
    return;

  uint8_t target = body[0];
  if (!isWheelTargetValid(target))
    return;

  Wheel::Calibration cal = (target == WHEEL_TARGET_LEFT)
                               ? state->calibration.leftWheelCalibData.read()
                               : state->calibration.rightWheelCalibData.read();

  WireWheelCalibData payload{};
  payload.target = target;
  payload.zeroElectricAngle = cal.zero_electric_angle;
  payload.sensorDirection = static_cast<int32_t>(cal.sensor_direction);
  sendPacket(MSG_WHEEL_CALIB_DATA, &payload, sizeof(payload));
}

static void handleWheelTuningWrite(SharedState *state, const uint8_t *body, size_t len) {
  if (len != sizeof(WireWheelTuning))
    return;

  WireWheelTuning msg;
  memcpy(&msg, body, sizeof(msg));

  uint8_t success = 0;
  if (isWheelTargetValid(msg.target)) {
    auto clamp = [](float v, float lo, float hi) -> float {
      return v < lo ? lo : (v > hi ? hi : v);
    };

    WheelTuningReq req;
    req.pending = true;
    req.persist = msg.persist != 0;
    req.tuning.p = clamp(msg.p, 0.0f, 20.0f);
    req.tuning.i = clamp(msg.i, 0.0f, 50.0f);
    req.tuning.d = clamp(msg.d, 0.0f, 5.0f);
    req.tuning.output_ramp = clamp(msg.outputRamp, 0.0f, 500.0f);
    req.tuning.lpf_velocity_tf = clamp(msg.lpfVelocityTf, 0.001f, 0.2f);
    req.tuning.velocity_limit = clamp(msg.velocityLimit, 1.0f, 80.0f);
    req.tuning.voltage_limit = clamp(msg.voltageLimit, 0.5f, 12.0f);

    if (msg.target == WHEEL_TARGET_LEFT) {
      state->calibration.leftWheelTuningReq.write(req);
    } else {
      state->calibration.rightWheelTuningReq.write(req);
    }
    success = 1;
  }

  WireCalibAck ack = {success};
  sendPacket(MSG_CALIB_ACK, &ack, sizeof(ack));
}

static void handleWheelTuningReadReq(SharedState *state, const uint8_t *body, size_t len) {
  if (len != 1)
    return;

  uint8_t target = body[0];
  if (!isWheelTargetValid(target))
    return;

  Wheel::VelocityTuning tuning = (target == WHEEL_TARGET_LEFT)
                                     ? state->calibration.leftWheelTuningData.read()
                                     : state->calibration.rightWheelTuningData.read();

  WireWheelTuningData payload{};
  payload.target = target;
  payload.p = tuning.p;
  payload.i = tuning.i;
  payload.d = tuning.d;
  payload.outputRamp = tuning.output_ramp;
  payload.lpfVelocityTf = tuning.lpf_velocity_tf;
  payload.velocityLimit = tuning.velocity_limit;
  payload.voltageLimit = tuning.voltage_limit;
  sendPacket(MSG_WHEEL_TUNING_DATA, &payload, sizeof(payload));
}

// ---------------------------------------------------------------------------
// Packet dispatcher — called after CRC is verified
// decoded[0] = type, decoded[1..n-3] = body, decoded[n-2..n-1] = CRC (consumed)
// ---------------------------------------------------------------------------
static void dispatch(SharedState *state,
                     const uint8_t *decoded, size_t decLen,
                     uint32_t &lastDriveTime, uint32_t &lastPoseTime) {
  if (decLen < 3)
    return; // minimum: type(1) + crc(2)

  uint8_t type = decoded[0];
  const uint8_t *body = decoded + 1;
  size_t bodyLen = decLen - 3; // strip type and 2 CRC bytes

  switch (type) {
  case MSG_CMD_DRIVE:
    handleDriveCmd(state, body, bodyLen);
    lastDriveTime = millis();
    sendDriveTelem(state);
    break;
  case MSG_CMD_POSE:
    handlePoseCmd(state, body, bodyLen);
    lastPoseTime = millis();
    sendPoseTelem(state);
    break;
  case MSG_STATUS_REQ:
    handleStatusReq(state, bodyLen);
    break;
  case MSG_CALIB_WRITE:
    handleCalibWrite(state, body, bodyLen);
    break;
  case MSG_CALIB_READ_REQ:
    handleCalibReadReq(state, body, bodyLen);
    break;
  case MSG_WHEEL_CALIB_CMD:
    handleWheelCalibCmd(state, body, bodyLen);
    break;
  case MSG_WHEEL_CALIB_READ_REQ:
    handleWheelCalibReadReq(state, body, bodyLen);
    break;
  case MSG_WHEEL_TUNING_WRITE:
    handleWheelTuningWrite(state, body, bodyLen);
    break;
  case MSG_WHEEL_TUNING_READ_REQ:
    handleWheelTuningReadReq(state, body, bodyLen);
    break;
  default:
    break;
  }
}

// ---------------------------------------------------------------------------
// Frame processing — COBS-decode, verify CRC, dispatch
// ---------------------------------------------------------------------------
static void processFrame(SharedState *state,
                         const uint8_t *raw, size_t rawLen,
                         uint32_t &lastDriveTime, uint32_t &lastPoseTime) {
  static uint8_t decoded[kRxBufSize];
  size_t decLen = cobs_decode(raw, rawLen, decoded);
  if (decLen < 3)
    return;
  uint16_t expected = crc16(decoded, decLen - 2);
  uint16_t received = ((uint16_t)decoded[decLen - 2] << 8) | decoded[decLen - 1];
  if (expected == received)
    dispatch(state, decoded, decLen, lastDriveTime, lastPoseTime);
}

// ---------------------------------------------------------------------------
// Task
// ---------------------------------------------------------------------------
void commTaskInit(SharedState &state) { Serial.begin(kBaudRate); }

void commTask(void *parameters) {
  auto state = static_cast<SharedState *>(parameters);

  static uint8_t rxBuf[kRxBufSize];
  size_t rxLen = 0;

  uint32_t lastDriveTime = millis();
  uint32_t lastPoseTime = millis();

  for (;;) {
    // --- RX: accumulate bytes until 0x00 delimiter ---
    while (Serial.available()) {
      uint8_t b = (uint8_t)Serial.read();

      if (b == 0x00) {
        if (rxLen > 0) {
          processFrame(state, rxBuf, rxLen, lastDriveTime, lastPoseTime);
          rxLen = 0;
        }
      } else {
        rxBuf[rxLen++] = b;
        if (rxLen >= kRxBufSize)
          rxLen = 0; // overflow guard
      }
    }

    // --- Heartbeat: disable actuators if no CMD received ---
    uint32_t now = millis();
    if (now - lastDriveTime > kHeartbeatTimeoutMs)
      state->commands.wheels.write(WheelsCommand{});
    if (now - lastPoseTime > kHeartbeatTimeoutMs)
      state->commands.servos.write(ServosCommand{});

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}