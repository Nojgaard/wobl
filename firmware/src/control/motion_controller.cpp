#include "control/motion_controller.hpp"
#include "control/leg_kinematics.hpp"
#include "control/pitch_offset.hpp"
#include "control/wheel_kinematics.hpp"

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

  _command.write(Command{
      .enable = false,
      .roll = 0.0f,
      .height = LegKinematics::NOMINAL_HEIGHT,
      .forwardVelocity = 0.0f,
      .turnVelocity = 0.0f,
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

WheelSubsystem::Command MotionController::balance(const Command &cmd,
                                                  const Observer::State &state,
                                                  float dt) {
  auto cfg = _config.read();

  if (!cmd.enable)
    return WheelSubsystem::Command{};

  float pitchOffset = PitchOffset::fromHeight(
      (state.leftLegHeight + state.rightLegHeight) / 2.0f);

  float pitchError = state.pitch - pitchOffset;
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

ServoSubsystem::Command MotionController::pose(const Command &cmd,
                                               const Observer::State &state,
                                               float dt) {
  if (!cmd.enable)
    return ServoSubsystem::Command{};

  // The observed roll is caused by both terrain slope and the left/right
  // leg-height difference. Estimate the terrain contribution as the residual
  // between the observed height difference and that implied by the roll.
  float obs_dh = state.leftLegHeight - state.rightLegHeight;
  float terrain_dh = WheelKinematics::WHEEL_BASE * sinf(state.roll) - obs_dh;

  // Compute the height difference needed for the commanded roll, then
  // compensate for the estimated terrain.
  float hff = WheelKinematics::WHEEL_BASE * sinf(cmd.roll) - terrain_dh;

  float kp = 0.3;
  float kd = 0.1;

  float dh = hff + kp * (cmd.roll - state.roll) - kd * state.rollRate;
  dh = 0; // disable for now to just test height adjustment

  float lh = cmd.height + 0.5f * dh;
  float rh = cmd.height - 0.5f * dh;

  float la = LegKinematics::toAngle(lh);
  float ra = LegKinematics::toAngle(rh);

  return ServoSubsystem::Command{
      .left = Servo::Command{.enabled = true, .positionRad = la},
      .right = Servo::Command{.enabled = true, .positionRad = ra}};
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

  auto state =
      observer.update(imuTelemetry, wheelTelemetry, servoTelemetry, dt);

  auto cmd = _command.read();
  ControlOutput output = {};

  if (!cmd.enable) {
    _positionError = 0.0f;
    sync(cmd, state, output, dt);
    return output;
  }

  output.wheels = balance(cmd, state, dt);
  output.servos = pose(cmd, state, dt);

  /*output.servos = ServoSubsystem::Command{
      .left = Servo::Command{.enabled = true, .positionRad = 0.1f},
      .right = Servo::Command{.enabled = true, .positionRad = 0.1f},
  };*/

  sync(cmd, state, output, dt);

  return output;
}
