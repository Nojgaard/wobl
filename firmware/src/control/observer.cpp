#include "control/observer.hpp"

static constexpr float WHEEL_BASE = 0.3f;    // meters
static constexpr float WHEEL_RADIUS = 0.04f; // meters

Observer::State
Observer::update(const ImuSubsystem::Telemetry &imuTelemetry,
                 const WheelSubsystem::Telemetry &wheelTelemetry,
                 const ServoSubsystem::Telemetry &servoTelemetry, float dt) {
  State state;

  _rollRate.Ts = dt;
  _pitchRate.Ts = dt;
  _leftWheelVelocity.Ts = dt;
  _rightWheelVelocity.Ts = dt;

  state.roll = imuTelemetry.roll;
  state.pitch = imuTelemetry.pitch;

  state.rollRate = _rollRate(imuTelemetry.rollRate);
  state.pitchRate = _pitchRate(imuTelemetry.pitchRate);

  float lv = _leftWheelVelocity(wheelTelemetry.left.velocity);
  float rv = _rightWheelVelocity(wheelTelemetry.right.velocity);
  state.forwardVelocity = (lv + rv) / 2.0f * WHEEL_RADIUS;
  state.turnVelocity = (rv - lv) / WHEEL_BASE * WHEEL_RADIUS;

  return state;
}

Observer::Config Observer::config() {
  return Config{.tcRates = _rollRate.Tf, .tcVelocities = _leftWheelVelocity.Tf};
}

void Observer::config(const Config &cfg) {
  _rollRate.Tf = cfg.tcRates;
  _pitchRate.Tf = cfg.tcRates;
  _leftWheelVelocity.Tf = cfg.tcVelocities;
  _rightWheelVelocity.Tf = cfg.tcVelocities;
}