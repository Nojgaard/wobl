#pragma once

#include "drivers/wheel.hpp"
#include "protected.hpp"

class WheelSubsystem {
public:
  struct Status {
    float syncRateHz;
    float updateRateHz;
    byte left;
    byte right;
  };

  struct Command {
    Wheel::Command left;
    Wheel::Command right;
  };

  struct Telemetry {
    Wheel::Data left;
    Wheel::Data right;
  };

  struct WheelTuningReq {
    Wheel::VelocityTuning tuning;
    bool persist;
  };

  WheelSubsystem();
  void init();
  void loop();

  Status status();
  void command(const Command &command);
  Telemetry telemetry();
  Wheel::VelocityTuning tuning(Wheel::Id wheelId);
  Wheel::Calibration calibration(Wheel::Id wheelId);

  void calibrate();
  void tune(const Wheel::VelocityTuning &tuning, bool persist);

private:
  void serviceRequests();
  void sync();

  enum class State {
    Uninitialized,
    Operational,
    Tuning,
    Calibrating,
  };

  Protected<State> _state;
  Protected<Status> _status;
  Protected<Command> _command;
  Protected<Telemetry> _telemetry;
  Protected<WheelTuningReq> _tuning;

  Wheel _wheelLeft;
  Wheel _wheelRight;

  TwoWire _wireLeft;
  TwoWire _wireRight;

  int _updateCountSinceLastSync = 0;
  long _lastSyncTimeMs = 0;
};