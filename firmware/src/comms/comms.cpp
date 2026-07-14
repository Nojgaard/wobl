#include "comms/comms.hpp"

static void loop(void *param) {
  Comms *comms = static_cast<Comms *>(param);

  for (;;) {
    comms->pilot.update();
    comms->console.update();
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void Comms::init() {
  pilot.init();
  console.init();
  
  xTaskCreatePinnedToCore(loop, "comms", 4096, this, 5, NULL, 0);

  Serial.println("[COMMS] Initialized");
}