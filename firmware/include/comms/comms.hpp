#pragma once
#include "comms/broadcaster.hpp"
#include "comms/console.hpp"
#include "comms/monitor.hpp"
#include "comms/pilot.hpp"

class Comms {
public:
  Comms(Robot &robot)
      : broadcaster(robot), monitor(robot), pilot(robot),
        console(robot, broadcaster, pilot, monitor) {}
  void init();

  Broadcaster broadcaster;
  Monitor monitor;
  Pilot pilot;
  Console console;
};