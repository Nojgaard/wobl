#include "subsystems/servo_subsystem.hpp"
#include "debug.hpp"

static constexpr int kServoRxPin = 16;
static constexpr int kServoTxPin = 17;
static constexpr long kServoBaud = 1000000;
static constexpr float kMaxVelocityRps = 1.0f;
static constexpr float kMaxAccelerationRps2 = 0.2f;

static constexpr Servo::Config kLeftHipConfig{
    .id = 0,
    .maxVelocityRps = 1.0f,
    .maxAccelerationRps2 = 0.2f,
    .coordSign = -1.0f,
};
static constexpr Servo::Config kRightHipConfig{
    .id = 5,
    .maxVelocityRps = 1.0f,
    .maxAccelerationRps2 = 0.2f,
    .coordSign = 1.0f,
};

ServoSubsystem::ServoSubsystem()
    : _leftHip(kLeftHipConfig), _rightHip(kRightHipConfig), _servoSerial(1) {}

void ServoSubsystem::init() {
  _servoSerial.begin(kServoBaud, SERIAL_8N1, kServoRxPin, kServoTxPin);
  _bus.pSerial = &_servoSerial;

  Status status;
  status.left = _leftHip.init(_bus);
  status.right = _rightHip.init(_bus);
  _status.write(status);

  DPRINTF("Left hip init: %d\n", status.left);
  DPRINTF("Right hip init: %d\n", status.right);

  Command command{
      .left = {.enabled = false, .positionRad = _leftHip.data().positionRad},
      .right = {.enabled = false, .positionRad = _rightHip.data().positionRad},
  };
  _command.write(command);

  _lastCommandTime = millis();
  _lastFeedbackTime = millis();
}

void ServoSubsystem::syncCommand() {
  long now = millis();
  if (now - _lastCommandTime < 10) {
    return;
  }

  Command command = _command.read();
  _leftHip.command(command.left);
  _rightHip.command(command.right);

  Status status = _status.read();
  status.cmdSyncRateHz = 1000.0 / (now - _lastCommandTime);
  _status.write(status);

  _lastCommandTime = now;
}

void ServoSubsystem::syncTelemetry() {
  long now = millis();
  if (now - _lastFeedbackTime < 50) {
    return;
  }

  _leftHip.update();
  _rightHip.update();

  Telemetry telemetry{_leftHip.data(), _rightHip.data()};
  _telemetry.write(telemetry);

  Status status = _status.read();
  status.telSyncRateHz = 1000.0 / (now - _lastFeedbackTime);

  if (_leftHip.data().valid && _rightHip.data().valid) {
    status.voltage = (_leftHip.voltage() + _rightHip.voltage()) / 2.0f;
  } else {
    status.voltage = 0.0f;
  }

  _status.write(status);

  _lastFeedbackTime = now;
}

void ServoSubsystem::loop() {
  for (;;) {
    auto s = status();
    if (!s.left || !s.right) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    syncCommand();
    syncTelemetry();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

ServoSubsystem::Status ServoSubsystem::status() { return _status.read(); }

void ServoSubsystem::command(const Command &cmd) { _command.write(cmd); }

ServoSubsystem::Telemetry ServoSubsystem::telemetry() {
  return _telemetry.read();
}