#include "debug.hpp"
#include "imu.hpp"
#include "shared_state.hpp"

static IMU imu;
static constexpr uint8_t kCSPin = 5;

static long lastUpdateTime = 0;

static IMUStatus status{
    .status = 0,
    .updateRate = 0.0,
};
static IMU::Data data;

void imuTaskInit(SharedState &state) {
  SPI.begin();
  status.status = imu.initialize(SPI, kCSPin);
  if (status.status != 1) {
    DPRINTF("Failed to initialize IMU: %d", status.status);
    while (true) {
      delay(1000);
    }
  } else {
    DPRINTLN("IMU initialized successfully");
  }

  lastUpdateTime = millis();
  state.status.imu.write(status);
}

static void update(SharedState &state) {
  if (imu.try_read(data)) {
    lastUpdateTime = millis();

    state.telemetry.imu.write(data);
  }
}

void imuTask(void *parameters) {
  auto state = static_cast<SharedState *>(parameters);

  while (true) {
    update(*state);
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}