#pragma once
#include "control/diff_drive_kinematics.hpp"
#include "control/kalman_filter.hpp"
#include "control/linear_filter.hpp"
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
    float pitch;
    float pitchRate;

    float roll;
    float rollRate;

    // Raw input velocities from the wheels, in rad/s
    WheelVelocity wheelVel;

    // Processed body velocities from the wheels, in m/s and rad/s
    BodyVelocity bodyVel;

    // Target body velocities from pilot, in m/s and rad/s
    BodyVelocity targetBodyVel;

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

  void init();
  void command(const Command &command);
  void config(const Config &config);

  Command command();
  Telemetry telemetry();
  Config config();

  ControlOutput update(const ImuSubsystem::Telemetry &imuTelemetry,
                       const WheelSubsystem::Telemetry &wheelTelemetry,
                       const ServoSubsystem::Telemetry &servoTelemetry);

private:
  void observe(const ImuSubsystem::Telemetry &imuTelemetry,
               const WheelSubsystem::Telemetry &wheelTelemetry,
               const ServoSubsystem::Telemetry &servoTelemetry, float dt);

  WheelSubsystem::Command balance(const Command &cmd, float dt);
  void sync(const WheelSubsystem::Telemetry &wheelTelemetry,
            const ControlOutput &controlOutput, float dt);

  Protected<Status> _status;
  Protected<Command> _command;
  Protected<Telemetry> _telemetry;
  Protected<Config> _config;

  float _pitch = 0;
  float _roll = 0;
  LinearFilter _rollRate{0.00f, 0.0f};

  LinearFilter _pitchRate{0.0f, 0.0f};
  KalmanFilter _forwardVelocity{2.0f, 0.25f};
  KalmanFilter _turnVelocity{2.0f, 0.25f};

  float _targetFwdVel = 0.0f;
  float _targetTurnVel = 0.0f;
  LowPassFilter _targetFwdVelLPF{0.1f};
  LowPassFilter _targetTurnVelLPF{0.1f};

  float _positionError = 0.0f;
  unsigned long _lastUpdateTimeMs = 0;
};