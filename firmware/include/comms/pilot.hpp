#pragma once

#include "robot.hpp"
#include "common/lowpass_filter.h"

class Pilot {
public:
  Pilot(Robot& robot) : _robot(robot) {}
  
  void init();
  void update();

private:
  bool _enableController = false;

  unsigned long _lastUpdateMs = 0;
  LowPassFilter _tarFwdVel{0.05f};
  LowPassFilter _tarTurnVel{0.05f};

  bool _pressedStart = false;

  Robot& _robot;
};