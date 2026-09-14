#include "control/observer.hpp"
#include "control/leg_kinematics.hpp"

static constexpr float WHEEL_BASE = 0.3f;    // meters
static constexpr float WHEEL_RADIUS = 0.04f; // meters

Observer::State
Observer::update(const ImuSubsystem::Telemetry &imuTelemetry,
                 const WheelSubsystem::Telemetry &wheelTelemetry,
                 const ServoSubsystem::Telemetry &servoTelemetry, float dt) {
  State state;

  state.roll = imuTelemetry.roll;
  state.pitch = imuTelemetry.pitch;

  state.rollRate = imuTelemetry.rollRate;
  state.pitchRate = imuTelemetry.pitchRate;

  float lv = wheelTelemetry.left.velocity;
  float rv = wheelTelemetry.right.velocity;
  state.forwardVelocity = (lv + rv) / 2.0f * WHEEL_RADIUS;
  state.turnVelocity = (rv - lv) / WHEEL_BASE * WHEEL_RADIUS;

  state.leftLegHeight = LegKinematics::toHeight(servoTelemetry.left.positionRad);
  state.rightLegHeight = LegKinematics::toHeight(servoTelemetry.right.positionRad);

  return state;
}