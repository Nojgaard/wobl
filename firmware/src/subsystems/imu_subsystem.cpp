#include "subsystems/imu_subsystem.hpp"
#include "debug.hpp"

static constexpr uint8_t kCSPin = 5;

static constexpr long kStatusIntervalMs = 500;

void ImuSubsystem::init() {
  bool spiSuccess = SPI.begin();
  if (!spiSuccess) {
    DPRINTLN("Failed to initialize SPI");
    _status.write({.status = 5, .syncRateHz = 0.0});
    return;
  }

  Status status;
  status.status = _imu.initialize(SPI, kCSPin);
  _status.write(status);
  
  if (status.status != 1) {
    DPRINTF("Failed to initialize IMU: %d", status.status);
    return;
  }

  DPRINTLN("IMU initialized successfully");
  _calibration.write(_imu.load_biases());

  _state.write(State::Operational);

  _lastSyncTime = millis();
}

void ImuSubsystem::update() {
  if (!_imu.status()) {
    _status.write({.status = _imu.status(), .syncRateHz = 0.0});
    return;
  }

  IMU::Data data;
  if (_imu.try_read(data)) {
    _readsSinceLastSync++;
    _telemetry.write(data);
  }

  long now = millis();
  long elapsed = now - _lastSyncTime;
  if (elapsed < kStatusIntervalMs) {
    return;
  }

  State state = _state.read();
  if (state == State::Calibrating) {
    _calibration.write(_imu.save_biases());
    _state.write(State::Operational);
  }

  Status status{
      .status = _imu.status(),
      .syncRateHz = (float)_readsSinceLastSync * 1000.0f / elapsed,
  };
  _status.write(status);
  _readsSinceLastSync = 0;
  _lastSyncTime = millis();
}

void ImuSubsystem::calibrate() {
  State state = _state.read();
  if (state != State::Operational) {
    DPRINTF("[imu] cannot calibrate, state=%d\r\n", static_cast<int>(state));
    return;
  }
  _state.write(State::Calibrating);
}

void ImuSubsystem::loop() {
  if (_status.read().status != 1) {
    DPRINTLN("IMU not initialized, skipping loop.");
    return;
  }

  for (;;) {
    update();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

ImuSubsystem::Status ImuSubsystem::status() { return _status.read(); }

IMU::Data ImuSubsystem::telemetry() { return _telemetry.read(); }

IMU::Calibration ImuSubsystem::calibration() { return _calibration.read(); }