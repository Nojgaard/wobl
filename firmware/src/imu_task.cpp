#include "debug.hpp"
#include "imu.hpp"
#include "imu_task.hpp"

static IMU imu;

static constexpr uint8_t kCSPin = 5;

static long lastStatusTime = 0;
static constexpr long kStatusIntervalMs = 1000;
static int readCount = 0;

static IMUStatus imuStatus{ .status = 0, .updateRate = 0.0 };
static IMU::Data data;

void imuTaskInit(SharedState &state) {
  bool spiSuccess = SPI.begin();
  if (!spiSuccess) {
    DPRINTLN("Failed to initialize SPI");
    imuStatus.status = -1;
    state.status.imu.write(imuStatus);
    return;
  }
  
  imuStatus.status = imu.initialize(SPI, kCSPin);
  if (imuStatus.status != 1) {
    DPRINTF("Failed to initialize IMU: %d", imuStatus.status);
  } else {
    DPRINTLN("IMU initialized successfully");
    auto cal = imu.load_biases();
    state.calibration.imuData.write(cal);
  }
  lastStatusTime = millis();
  state.status.imu.write(imuStatus);
}

static void update(SharedState &state) {
  if (imuStatus.status != 1) {
    return;
  }

  CalibReq req = state.calibration.imuReq.read();
  if (req.pending) {
    state.calibration.imuReq.write(CalibReq{});
    auto cal = imu.save_biases();
    state.calibration.imuData.write(cal);
  }

  if (imu.try_read(data)) {
    readCount++;
    state.telemetry.imu.write(data);
  }

  long now = millis();
  long elapsed = now - lastStatusTime;
  if (elapsed >= kStatusIntervalMs) {
    imuStatus.updateRate = readCount * 1000.0 / elapsed;
    readCount = 0;
    lastStatusTime = now;
    state.status.imu.write(imuStatus);
  }
}

void imuTask(void *parameters) {
  auto state = static_cast<SharedState *>(parameters);

  while (true) {
    update(*state);
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}