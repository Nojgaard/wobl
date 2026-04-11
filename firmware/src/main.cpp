#include "imu_task.hpp"
#include "wheel_task.hpp"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_bt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <SCServo.h>

long lastUpdateTime = 0;

static SharedState state;
SMS_STS st;

void setup() {
  WiFi.mode(WIFI_OFF);
  esp_bt_controller_disable();
  Serial.begin(115200);

  wheelTaskInit(state);
  imuTaskInit(state);

  xTaskCreatePinnedToCore(wheelTask, "foc", 4096, &state, 24, NULL, 1);
  xTaskCreatePinnedToCore(imuTask, "imu", 4096, &state, 10, NULL, 0);
  st.CalibrationOfs(1);
}

void loop() { vTaskDelete(NULL); }