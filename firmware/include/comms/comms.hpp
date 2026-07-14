#pragma once
#include "comms/pilot.hpp"
#include "comms/console.hpp"

class Comms {
public:
  Comms(Robot &robot) : pilot(robot), console(&robot) {}
  void init();

  Pilot pilot;
  Console console;
};