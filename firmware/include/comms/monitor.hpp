#pragma once

#include "robot.hpp"

class Monitor {
public:
  enum class Display { NONE, SERVO, ROBOT };

  Monitor(Robot &robot) : _robot(robot) {}

  void init();
  void update();

  Display mode = Display::NONE;

private:
  Robot &_robot;
};