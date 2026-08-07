#pragma once

#include "common/lowpass_filter.h"
#include "protected.hpp"
#include "robot.hpp"

class Pilot {
public:
  struct Status {
    float syncRate;
  };

  Pilot(Robot &robot) : _robot(robot) {}

  void init();
  void update();
  Pilot::Status status();

  void disconnect();
  void scanForDevices(bool on);
  void forgetDevices();

private:
  Protected<Status> _status;
  bool _enableController = false;

  unsigned long _lastUpdateMs = 0;
  LowPassFilter _tarFwdVel{0.0f};
  LowPassFilter _tarTurnVel{0.0f};

  bool _pressedStart = false;

  Robot &_robot;
};