#include "imu_task.hpp"
#include "wheel_task.hpp"
#include "servo_task.hpp"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_bt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

long lastUpdateTime = 0;

static SharedState state;

void setup() {
  WiFi.mode(WIFI_OFF);
  esp_bt_controller_disable();
  Serial.begin(115200);

  wheelTaskInit(state);
  imuTaskInit(state);
  servoTaskInit(state);

  xTaskCreatePinnedToCore(wheelTask, "foc",   4096, &state, 24, NULL, 1);
  xTaskCreatePinnedToCore(imuTask,   "imu",   4096, &state, 10, NULL, 0);
  xTaskCreatePinnedToCore(servoTask, "servo", 4096, &state, 10, NULL, 0);
}

void loop() { vTaskDelete(NULL); }