#include "control/motion_controller.hpp"

void MotionController::init() {
  _lastUpdateTimeMs = millis();
  _positionError = 0.0f;

  _config.write(Config{
      .pitchOffset = 0.07f,
      .pitchKp = -4.11468176f,
      .pitchRateKp = -0.45214264f,
      .positionKp = -0.81649658f,
      .velocityKp = -1.2660191f,
  });
}

void MotionController::command(const Command &command) { _command.write(command); }
void MotionController::config(const Config &config) { _config.write(config); }

MotionController::Command MotionController::command() { return _command.read(); }
MotionController::Telemetry MotionController::telemetry() { return _telemetry.read(); }
MotionController::Config MotionController::config() { return _config.read(); }

void MotionController::observe(const ImuSubsystem::Telemetry &imuTelemetry,
                         const WheelSubsystem::Telemetry &wheelTelemetry,
                         const ServoSubsystem::Telemetry &servoTelemetry,
                         float dt) {
  _pitch = imuTelemetry.pitch;
  _roll = imuTelemetry.roll;
  _pitchRate.update(imuTelemetry.pitchRate);
  _rollRate.update(imuTelemetry.rollRate);

  auto bodyVel = DiffDriveKinematics::toBodyVel(wheelTelemetry.left.velocity,
                                                wheelTelemetry.right.velocity);
  _forwardVelocity.update(bodyVel.forwardVelocity, dt);
  _turnVelocity.update(bodyVel.yawRate, dt);
}

WheelSubsystem::Command MotionController::balance(const Command &cmdBody, float dt) {
  auto cfg = _config.read();

  if (!cmdBody.enable)
    return WheelSubsystem::Command{};

  float pitchError = _pitch - cfg.pitchOffset;
  float pitchRateError = _pitchRate.value();
  float velocityError = _forwardVelocity.value() - cmdBody.forwardVelocity;

  _positionError += velocityError * dt;
  _positionError = std::clamp(_positionError, -0.3f, 0.3f);

  float ctrlFwdVel = -cfg.pitchKp * pitchError;
  ctrlFwdVel -= cfg.pitchRateKp * pitchRateError;
  ctrlFwdVel -= cfg.positionKp * _positionError;
  ctrlFwdVel -= cfg.velocityKp * velocityError;

  float ctrlTurnVel = cmdBody.turnVelocity;

  auto ctrlWheelVel = DiffDriveKinematics::toWheelVel(ctrlFwdVel, ctrlTurnVel);
  return WheelSubsystem::Command{
      .left = Wheel::Command{.enabled = true, .velocity = ctrlWheelVel.leftRps},
      .right =
          Wheel::Command{.enabled = true, .velocity = ctrlWheelVel.rightRps}};
}

void MotionController::sync(float dt) {
  _status.write(Status{.syncRateHz = 1.0f / dt});
  _telemetry.write(Telemetry{
      .pitch = _pitch,
      .pitchRate = _pitchRate.value(),
      .roll = _roll,
      .rollRate = _rollRate.value(),
      .bodyVel = BodyVelocity{.forwardVelocity = _forwardVelocity.value(),
                              .yawRate = _turnVelocity.value()}});
}

MotionController::ControlOutput
MotionController::update(const ImuSubsystem::Telemetry &imuTelemetry,
                   const WheelSubsystem::Telemetry &wheelTelemetry,
                   const ServoSubsystem::Telemetry &servoTelemetry) {
  unsigned long now = millis();
  float dt = (now - _lastUpdateTimeMs) / 1000.0f;
  _lastUpdateTimeMs = now;
  dt = std::min(dt, 0.05f);

  observe(imuTelemetry, wheelTelemetry, servoTelemetry, dt);

  auto cmdBody = _command.read();
  if (!cmdBody.enable) {
    _positionError = 0.0f;
    return ControlOutput{
        .wheels = WheelSubsystem::Command{},
        .servos = ServoSubsystem::Command{}};
  }
  auto wheelCmd = balance(cmdBody, dt);
  sync(dt);

  ServoSubsystem::Command servoCmd{
      .left = Servo::Command{.enabled = true, .positionRad = 0.1f},
      .right = Servo::Command{.enabled = true, .positionRad = 0.1f},
  };

  return ControlOutput{.wheels = wheelCmd, .servos = servoCmd};
}
