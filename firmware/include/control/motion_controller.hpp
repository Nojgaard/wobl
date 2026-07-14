#pragma once
#include "control/diff_drive_kinematics.hpp"
#include "control/kalman_filter.hpp"
#include "control/linear_filter.hpp"
#include "protected.hpp"
#include "subsystems/imu_subsystem.hpp"
#include "subsystems/servo_subsystem.hpp"
#include "subsystems/wheel_subsystem.hpp"

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

  struct Telemetry {
    float pitch;
    float pitchRate;

    float roll;
    float rollRate;

    BodyVelocity bodyVel;
  };

  struct Config {
    float pitchOffset;

    float pitchKp;
    float pitchRateKp;
    float positionKp;
    float velocityKp;
  };

  struct ControlOutput {
    WheelSubsystem::Command wheels;
    ServoSubsystem::Command servos;
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
  void sync(float dt);

  Protected<Status> _status;
  Protected<Command> _command;
  Protected<Telemetry> _telemetry;
  Protected<Config> _config;

  float _pitch = 0;
  float _roll = 0;
  LinearFilter _rollRate{0.1f, 0.0f};
  LinearFilter _pitchRate{0.1f, 0.0f};
  KalmanFilter _forwardVelocity{2.0f, 0.25f};
  KalmanFilter _turnVelocity{2.0f, 0.25f};

  float _positionError = 0.0f;
  unsigned long _lastUpdateTimeMs = 0;
};