#pragma once

#include "subsystems/imu_subsystem.hpp"
#include "subsystems/servo_subsystem.hpp"
#include "subsystems/wheel_subsystem.hpp"
#include <common/lowpass_filter.h>

class Observer {
public:
  struct State {
    float roll;
    float pitch;

    float rollRate;
    float pitchRate;

    float forwardVelocity;
    float turnVelocity;

    float leftLegHeight;
    float rightLegHeight;
  };

  State update(const ImuSubsystem::Telemetry &imuTelemetry,
               const WheelSubsystem::Telemetry &wheelTelemetry,
               const ServoSubsystem::Telemetry &servoTelemetry, float dt);

private:
};