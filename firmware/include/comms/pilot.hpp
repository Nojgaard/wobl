#pragma once

#include "robot.hpp"

class Pilot {
public:
  Pilot(Robot& robot) : _robot(robot) {}
  
  void init();
  void update();

private:
  bool pressedStart = false;

  Robot& _robot;
};