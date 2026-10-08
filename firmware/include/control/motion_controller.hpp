#pragma once
#include "control/observer.hpp"
#include "protected.hpp"
#include "subsystems/imu_subsystem.hpp"
#include "subsystems/servo_subsystem.hpp"
#include "subsystems/wheel_subsystem.hpp"
#include <SimpleFOC.h>

class MotionController {
public:
  struct Status {
    float syncRateHz;
  };

  struct Command {
    bool enable;

    // Pose
    float roll;
    float height;

    // Velocity
    float forwardVelocity;
    float turnVelocity;
  };

  struct ControlOutput {
    WheelSubsystem::Command wheels;
    ServoSubsystem::Command servos;
  };

  struct Telemetry {
    unsigned long timestampMs;

    Command command;
    Observer::State state;
    ControlOutput output;
  };

  struct PoseGains {
    float roll;
    float rollRate;
  };

  struct BalanceGains {
    float outputScale;
    float pitch;
    float pitchRate;
    float position;
    float velocity;
  };

  struct Config {
    BalanceGains balanceGains;
    PoseGains poseGains;
  };

  Observer observer;

  void init();
  void command(const Command &command);
  void config(const Config &config);

  Status status();
  Command command();
  Telemetry telemetry();
  Config config();

  ControlOutput update(const ImuSubsystem::Telemetry &imuTelemetry,
                       const WheelSubsystem::Telemetry &wheelTelemetry,
                       const ServoSubsystem::Telemetry &servoTelemetry);

private:
  WheelSubsystem::Command balance(const Command &cmd,
                                  const Observer::State &state, float dt);

  ServoSubsystem::Command pose(const Command &cmd, const Observer::State &state,
                               float dt);

  void sync(const Command &cmd, const Observer::State &state,
            const ControlOutput &controlOutput, float dt);

  Protected<Status> _status;
  Protected<Command> _command;
  Protected<Telemetry> _telemetry;
  Protected<Config> _config;

  float _positionError = 0.0f;
  unsigned long _lastUpdateTimeMs = 0;
};