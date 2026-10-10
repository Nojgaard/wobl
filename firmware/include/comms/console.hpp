#pragma once

#include "comms/broadcaster.hpp"
#include "comms/monitor.hpp"
#include "comms/pilot.hpp"
#include "robot.hpp"

class Console {
public:
  Console(Robot &robot, Broadcaster &broadcaster, Pilot &pilot,
          Monitor &monitor);

  void init();
  void update();

  Robot &robot;
  Broadcaster &broadcaster;
  Pilot &pilot;
  Monitor &monitor;

private:
  void dispatch(const char *args);

  constexpr static int MAX_CMD_SIZE = 40;
  // Received serial message - waiting for newline
  char _received_chars[MAX_CMD_SIZE] = {0};
  // Number of characters received from serial
  int _received_count = 0;
};