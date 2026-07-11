#include "robot.hpp"
#ifndef DEBUG
#include "comm_task.hpp"
#endif

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static Robot robot;

void setup() {
#ifdef DEBUG
  Serial.begin(115200);
#else
  commTaskInit(robot);
  xTaskCreatePinnedToCore(commTask, "comm", 4096, &robot, 10, NULL, 0);
#endif

  delay(500);
  robot.init();
}

void loop() { vTaskDelete(NULL); }