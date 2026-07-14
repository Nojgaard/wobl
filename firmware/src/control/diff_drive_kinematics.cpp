#include "control/diff_drive_kinematics.hpp"
#include <algorithm>

static constexpr float WHEEL_BASE = 0.3f;    // meters
static constexpr float WHEEL_RADIUS = 0.04f; // meters

BodyVelocity DiffDriveKinematics::toBodyVel(float leftWheelRps,
                                            float rightWheelRps) {
  float forwardVelocity = (leftWheelRps + rightWheelRps) / 2.0f * WHEEL_RADIUS;
  float yawRate = (rightWheelRps - leftWheelRps) / WHEEL_BASE * WHEEL_RADIUS;
  return {forwardVelocity, yawRate};
}

WheelVelocity DiffDriveKinematics::toWheelVel(float forwardVelocity,
                                              float yawRate) {
  float yawRps = yawRate * WHEEL_BASE / 2.0f;
  float leftWheelRps = (forwardVelocity - yawRps) / WHEEL_RADIUS;
  float rightWheelRps = (forwardVelocity + yawRps) / WHEEL_RADIUS;
  return {leftWheelRps, rightWheelRps};
}
