#include "imu_task.hpp"
#include "wheel_task.hpp"
#include "servo_task.hpp"
#ifndef DEBUG
#include "comm_task.hpp"
#endif

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

long lastUpdateTime = 0;

static SharedState state;

void setup() {
#ifdef DEBUG
  Serial.begin(115200);
#else
  commTaskInit(state);
  xTaskCreatePinnedToCore(commTask, "comm", 4096, &state, 10, NULL, 0);
#endif

  delay(500);
  wheelTaskInit(state);
  servoTaskInit(state);
  imuTaskInit(state);

  xTaskCreatePinnedToCore(wheelTask, "foc",   4096, &state, 24, NULL, 1);
  xTaskCreatePinnedToCore(imuTask,   "imu",   4096, &state, 10, NULL, 0);
  xTaskCreatePinnedToCore(servoTask, "servo", 4096, &state, 10, NULL, 0);

}

void loop() { vTaskDelete(NULL); }