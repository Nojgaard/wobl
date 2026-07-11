#include "subsystems/wheel_subsystem.hpp"
#include "config.hpp"
#include "debug.hpp"

WheelSubsystem::WheelSubsystem()
    : _wheelLeft(leftWheelConfig), _wheelRight(rightWheelConfig), _wireLeft(0),
      _wireRight(1) {}

void WheelSubsystem::init() {
  constexpr int CLOCK_SPEED = 400000; // 400 kHz
  constexpr float VOLTAGE_SUPPLY = 12.0;
  constexpr float VOLTAGE_LIMIT = 5.0;

  _wireLeft.begin(21, 22, CLOCK_SPEED);
  _wireRight.begin(25, 26, CLOCK_SPEED);

#ifdef DEBUG
  SimpleFOCDebug::enable(&Serial);
#endif

  Status status;
  vTaskDelay(pdMS_TO_TICKS(200));
  status.left = _wheelLeft.init(VOLTAGE_SUPPLY, VOLTAGE_LIMIT, _wireLeft);
  vTaskDelay(pdMS_TO_TICKS(10));
  status.right = _wheelRight.init(VOLTAGE_SUPPLY, VOLTAGE_LIMIT, _wireRight);

  DPRINTF("[wheels] left  init=%d ok=%d\r\n", status.left, _wheelLeft.isOk());
  DPRINTF("[wheels] right init=%d ok=%d\r\n", status.right, _wheelRight.isOk());

  _updateCountSinceLastSync = 0;
  _lastSyncTimeMs = millis();
  _status.write(status);

  _state.write(State::Operational);

  SimpleFOCDebug::enable(nullptr); // disable debug after init
}

WheelSubsystem::Status WheelSubsystem::status() { return _status.read(); }

void WheelSubsystem::command(const Command &command) {
  _command.write(command);
}

WheelSubsystem::Telemetry WheelSubsystem::telemetry() {
  return _telemetry.read();
}

void WheelSubsystem::calibrate() {
  State state = _state.read();
  if (state != State::Operational) {
    DPRINTF("[wheels] cannot calibrate, state=%d\r\n", static_cast<int>(state));
    return;
  }
  _state.write(State::Calibrating);
}

void WheelSubsystem::tune(const Wheel::VelocityTuning &tuning, bool persist) {
  State state = _state.read();
  if (state != State::Operational) {
    DPRINTF("[wheels] cannot tune, state=%d\r\n", static_cast<int>(state));
    return;
  }

  _tuning.write({tuning, persist});
  _state.write(State::Tuning);
}

Wheel::VelocityTuning WheelSubsystem::tuning(Wheel::Id wheelId) {
  if (wheelId == Wheel::Id::Left) {
    return _wheelLeft.tuning();
  } else if (wheelId == Wheel::Id::Right) {
    return _wheelRight.tuning();
  } else {
    return {};
  }
}

Wheel::Calibration WheelSubsystem::calibration(Wheel::Id wheelId) {
  if (wheelId == Wheel::Id::Left) {
    return _wheelLeft.calibration();
  } else if (wheelId == Wheel::Id::Right) {
    return _wheelRight.calibration();
  } else {
    return {};
  }
}

void WheelSubsystem::serviceRequests() {
  State state = _state.read();
  if (state == State::Calibrating) {
    _wheelLeft.calibrate();
    _wheelRight.calibrate();
    _state.write(State::Operational);
  } else if (state == State::Tuning) {
    WheelTuningReq tuningReq = _tuning.read();
    _wheelLeft.tune(tuningReq.tuning, tuningReq.persist);
    _wheelRight.tune(tuningReq.tuning, tuningReq.persist);
    _state.write(State::Operational);
  }
}

void WheelSubsystem::sync() {
  // Command + telemetry sync at 250 Hz
  long now = millis();
  if (now - _lastSyncTimeMs < 4) {
    return;
  }

  serviceRequests();

  auto command = _command.read();
  _wheelLeft.command(command.left.enabled, command.left.velocity);
  _wheelRight.command(command.right.enabled, command.right.velocity);

  Telemetry telemetry{_wheelLeft.data(), _wheelRight.data()};
  _telemetry.write(telemetry);

  Status status;
  status.syncRateHz = 1000.0 / (now - _lastSyncTimeMs);
  status.updateRateHz = status.syncRateHz * _updateCountSinceLastSync;
  status.left = _wheelLeft.isOk();
  status.right = _wheelRight.isOk();
  _status.write(status);

  _updateCountSinceLastSync = 0;
  _lastSyncTimeMs = now;
}

void WheelSubsystem::loop() {
  for (;;) {
    _wheelLeft.update();
    _wheelRight.update();
    _updateCountSinceLastSync += 1;

    sync();
  }
}