#pragma once

#include "common/lowpass_filter.h"
#include "protected.hpp"
#include "robot.hpp"
#include "control/leg_kinematics.hpp"
#include "comms/broadcaster.hpp"

class Pilot {
public:
  struct Status {
    float syncRate;
  };

  Pilot(Robot &robot, Broadcaster &broadcaster) : _robot(robot), _broadcaster(broadcaster) {}

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
  float _tarHeight = LegKinematics::NOMINAL_HEIGHT;

  bool _pressedStart = false;
  bool _pressedSelect = false;
  int8_t _pressedDpad = 0;

  Robot &_robot;
  Broadcaster &_broadcaster;
};