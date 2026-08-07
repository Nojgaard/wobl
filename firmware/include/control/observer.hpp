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
  };

  struct Config {
    float tcRates;
    float tcVelocities;
  };

  State update(const ImuSubsystem::Telemetry &imuTelemetry,
               const WheelSubsystem::Telemetry &wheelTelemetry,
               const ServoSubsystem::Telemetry &servoTelemetry, float dt);

  Config config();
  void config(const Config &cfg);

private:
  LowPassFilter _pitchRate{0.0f};
  LowPassFilter _rollRate{0.0f};

  LowPassFilter _leftWheelVelocity{0.0f};
  LowPassFilter _rightWheelVelocity{0.0f};
};