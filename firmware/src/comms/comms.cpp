#include "comms/comms.hpp"

static void loop(void *param) {
  Comms *comms = static_cast<Comms *>(param);
  unsigned long lastUpdateTime = millis();

  for (;;) {
    unsigned long now = millis();

    if (now - lastUpdateTime >= 50) {
      comms->pilot.update();
      comms->console.update();
      lastUpdateTime = now;
    }

    comms->broadcaster.update();

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void Comms::init() {
  pilot.init();
  console.init();
  broadcaster.init();

  xTaskCreatePinnedToCore(loop, "comms", 4096, this, 5, NULL, 0);

  Serial.println("[COMMS] Initialized");
}