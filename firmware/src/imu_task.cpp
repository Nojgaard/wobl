#include "debug.hpp"
#include "imu.hpp"
#include "shared_state.hpp"

static IMU imu;
static constexpr uint8_t kCSPin = 5;

static long lastStatusTime = 0;
static constexpr long kStatusIntervalMs = 1000;
static int readCount = 0;

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

  lastStatusTime = millis();
  state.status.imu.write(status);
}

static void update(SharedState &state) {
  if (imu.try_read(data)) {
    readCount++;
    state.telemetry.imu.write(data);
  }

  long now = millis();
  long elapsed = now - lastStatusTime;
  if (elapsed >= kStatusIntervalMs) {
    status.updateRate = readCount * 1000.0 / elapsed;
    readCount = 0;
    lastStatusTime = now;
    state.status.imu.write(status);
  }
}

void imuTask(void *parameters) {
  auto state = static_cast<SharedState *>(parameters);

  while (true) {
    update(*state);
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}