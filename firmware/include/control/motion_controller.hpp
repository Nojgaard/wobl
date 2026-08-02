#pragma once
#include "control/diff_drive_kinematics.hpp"
#include "control/kalman_filter.hpp"
#include "control/linear_filter.hpp"
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

  struct Config {
    float pitchOffset;
    float ctrlScale;

    float pitchKp;
    float pitchRateKp;
    float positionKp;
    float velocityKp;
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
  void sync(const Command &cmd, const Observer::State &state,
            const ControlOutput &controlOutput, float dt);

  Protected<Status> _status;
  Protected<Command> _command;
  Protected<Telemetry> _telemetry;
  Protected<Config> _config;

  float _positionError = 0.0f;
  unsigned long _lastUpdateTimeMs = 0;
};