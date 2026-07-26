#pragma once
#include "comms/broadcaster.hpp"
#include "comms/console.hpp"
#include "comms/pilot.hpp"

class Comms {
public:
  Comms(Robot &robot)
      : broadcaster(robot), pilot(robot), console(&robot, &broadcaster) {}
  void init();

  Broadcaster broadcaster;
  Pilot pilot;
  Console console;
};