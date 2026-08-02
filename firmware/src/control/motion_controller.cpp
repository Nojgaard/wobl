#include "control/motion_controller.hpp"

void MotionController::init() {
  _lastUpdateTimeMs = millis();
  _positionError = 0.0f;

  _config.write(Config{
      .pitchOffset = 0.07f,
      .ctrlScale = 1.0f,
      .pitchKp = -15.248f,
      .pitchRateKp = -1.354f,
      .positionKp = -3.333f,
      .velocityKp = -4.017f,
  });
}

MotionController::Status MotionController::status() { return _status.read(); }

void MotionController::command(const Command &command) {
  _command.write(command);
}
void MotionController::config(const Config &config) { _config.write(config); }

MotionController::Command MotionController::command() {
  return _command.read();
}
MotionController::Telemetry MotionController::telemetry() {
  return _telemetry.read();
}
MotionController::Config MotionController::config() { return _config.read(); }

WheelSubsystem::Command MotionController::balance(const Command &cmd, const Observer::State &state, float dt) {
  auto cfg = _config.read();

  if (!cmd.enable)
    return WheelSubsystem::Command{};

  float pitchError = state.pitch - cfg.pitchOffset;
  float pitchRateError = state.pitchRate;
  float velocityError = state.forwardVelocity - cmd.forwardVelocity;

  _positionError += velocityError * dt;
  _positionError = std::clamp(_positionError, -0.1f, 0.1f);

  float ctrlFwdVel = -cfg.pitchKp * pitchError;
  ctrlFwdVel -= cfg.pitchRateKp * pitchRateError;
  ctrlFwdVel -= cfg.positionKp * _positionError;
  ctrlFwdVel -= cfg.velocityKp * velocityError;

  float ctrlTurnVel = cmd.turnVelocity;

  float ctrlLeft = ctrlFwdVel * cfg.ctrlScale + ctrlTurnVel;
  float ctrlRight = ctrlFwdVel * cfg.ctrlScale - ctrlTurnVel;
  return WheelSubsystem::Command{
      .left = Wheel::Command{.enabled = true, .velocity = ctrlLeft},
      .right = Wheel::Command{.enabled = true, .velocity = ctrlRight}};
}

void MotionController::sync(const Command &cmd, const Observer::State &state,
                            const ControlOutput &controlOutput, float dt) {
  _status.write(Status{.syncRateHz = 1.0f / dt});
  _telemetry.write(Telemetry{
      .timestampMs = millis(),
      .command = cmd,
      .state = state,
      .output = controlOutput,
  });
}

MotionController::ControlOutput
MotionController::update(const ImuSubsystem::Telemetry &imuTelemetry,
                         const WheelSubsystem::Telemetry &wheelTelemetry,
                         const ServoSubsystem::Telemetry &servoTelemetry) {
  unsigned long now = millis();
  float dt = (now - _lastUpdateTimeMs) / 1000.0f;
  _lastUpdateTimeMs = now;
  dt = std::min(dt, 0.05f);

  auto cmd = _command.read();
  ControlOutput output = {};

  if (!cmd.enable) {
    _positionError = 0.0f;
    return output;
  }

  auto state = observer.update(imuTelemetry, wheelTelemetry, servoTelemetry, dt);
  output.wheels = balance(cmd, state, dt);

  output.servos = ServoSubsystem::Command{
      .left = Servo::Command{.enabled = true, .positionRad = 0.1f},
      .right = Servo::Command{.enabled = true, .positionRad = 0.1f},
  };

  sync(cmd, state, output, dt);

  return output;
}
