#pragma once

#include "robot.hpp"

class Pilot {
public:
  Pilot(Robot& robot) : _robot(robot) {}
  
  void init();
  void update();

private:
  bool _enableController = false;
  float _targetFwdVel = 0.0f;
  float _targetTurnVel = 0.0f;

  bool _pressedStart = false;

  Robot& _robot;
};