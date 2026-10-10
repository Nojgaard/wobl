#pragma once
#include "robot.hpp"

class Broadcaster {
public:
  Broadcaster(Robot &robot) : _robot(robot) {}
  void init();
  void update();

  void enable(bool on);
  bool enabled() { return _enabled; }

  bool saveSsid(const char *newSsid);
  bool savePassword(const char *newPassword);

private:
  bool _enabled = false;
  Robot &_robot;
};