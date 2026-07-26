#pragma once
#include "robot.hpp"

class Broadcaster {
public:
  Broadcaster(Robot &robot) : _robot(robot) {}
  void init();
  void update();

  void enable(bool on);

private:
  bool _enabled = false;
  Robot &_robot;
};