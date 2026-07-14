#include "robot.hpp"
#include "comms/comms.hpp"

#include <Arduino.h>

static Robot robot;
static Comms comms(robot);

void setup() {
  Serial.begin(115200);

  comms.init();
  robot.init();
}

void loop() { vTaskDelete(NULL); }