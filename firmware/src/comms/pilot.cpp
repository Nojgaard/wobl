#include "comms/pilot.hpp"
#include "control/leg_kinematics.hpp"
#include <Arduino.h>
#include <Bluepad32.h>

static ControllerPtr gamepad = nullptr;

static constexpr float AXIS_DEADZONE = 30;
static constexpr float AXIS_MAX = 520;

static constexpr float MAX_FWD_VEL = 0.3f; // m/s
static constexpr float MAX_TURN_VEL = 0.5f;

static constexpr float TRIGGER_DEADZONE = 50;
static constexpr float TRIGGER_MAX = 1023;

static constexpr float HEIGHT_VEL = 0.03f; // m/s
static constexpr float MAX_ROLL = 0.2f;    // rad

float normalizeAxis(int32_t value) {
  if (abs(value) < AXIS_DEADZONE) {
    return 0.0f;
  }
  return (float)value / AXIS_MAX;
}

float normalizeTrigger(int32_t value) {
  if (value < TRIGGER_DEADZONE) {
    return 0.0f;
  }
  return (float)value / TRIGGER_MAX;
}

bool hasData() {
  return gamepad && gamepad->isConnected() && gamepad->hasData();
}

void onConnectedController(ControllerPtr ctl) {
  Serial.printf("Controller connected — %s\n", ctl->getModelName());
  gamepad = ctl;
}

void onDisconnectedController(ControllerPtr ctl) {
  Serial.printf("Controller disconnected — %s\n", ctl->getModelName());
  gamepad = nullptr;
}

void Pilot::init() {
  Serial.printf("BP32 Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t *addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2],
                addr[3], addr[4], addr[5]);

  BP32.setup(&onConnectedController, &onDisconnectedController, true);
  // BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);

  _lastUpdateMs = millis();
  _status.write({});
}

void Pilot::disconnect() {
  if (gamepad) {
    gamepad->disconnect();
  }
}

void Pilot::scanForDevices(bool enabled) {
  BP32.enableNewBluetoothConnections(enabled);
}

void Pilot::forgetDevices() { BP32.forgetBluetoothKeys(); }

Pilot::Status Pilot::status() { return _status.read(); }

void Pilot::update() {
  if (!BP32.update() || !hasData()) {
    return;
  }

  unsigned long now = millis();
  float dt = (now - _lastUpdateMs) / 1000.0f;
  _lastUpdateMs = now;

  if (_pressedStart && !gamepad->miscStart()) {
    _enableController = !(_robot.controller.command().enable);
    Serial.printf("Controller %s\n",
                  _enableController ? "ENABLED" : "DISABLED");
  }
  _pressedStart = gamepad->miscStart();

  // On select, toggle broadcast telemetry over WIFI
  if (_pressedSelect && !gamepad->miscSelect()) {
    scanForDevices(false); // BLE scanning blocks WIFI
    _broadcaster.enable(!_broadcaster.enabled());
  }
  _pressedSelect = gamepad->miscSelect();

  _tarFwdVel.Ts = dt;
  _tarTurnVel.Ts = dt;
  float tarFwdVel = _tarFwdVel(normalizeAxis(-gamepad->axisY()) * MAX_FWD_VEL);
  float tarTurnVel =
      _tarTurnVel(normalizeAxis(gamepad->axisRX()) * MAX_TURN_VEL);

  int heightDir =
      static_cast<int>(gamepad->r1()) - static_cast<int>(gamepad->l1());
  _tarHeight += heightDir * HEIGHT_VEL * dt;

  if ((_pressedDpad & DPAD_UP) && !(gamepad->dpad() & DPAD_UP)) {
    _tarHeight += 0.005f;
  }

  if ((_pressedDpad & DPAD_DOWN) && !(gamepad->dpad() & DPAD_DOWN)) {
    _tarHeight -= 0.005f;
  }
  _pressedDpad = gamepad->dpad();

  _tarHeight = std::clamp(_tarHeight, LegKinematics::HEIGHT_MIN,
                          LegKinematics::HEIGHT_MAX);

  float roll =
      (normalizeTrigger(gamepad->brake()) - normalizeTrigger(gamepad->throttle())) *
      MAX_ROLL;

  _robot.controller.command({.enable = _enableController,
                             .roll = roll,
                             .height = _tarHeight,
                             .forwardVelocity = tarFwdVel,
                             .turnVelocity = tarTurnVel});

  _status.write({.syncRate = 1.0f / dt});

  /*static unsigned long lastPrintMs = 0;
  if (now - lastPrintMs > 1000) {
    Serial.printf("buttons: 0x%04x, axis L: %4d, %4d, "
                  "axis R: %4d, %4d, start: 0x%04x, sel: 0x%04x %.4f\n",
                  gamepad->buttons(), gamepad->axisX(), gamepad->axisY(),
                  gamepad->axisRX(), gamepad->axisRY(), gamepad->miscStart(),
                  gamepad->miscSelect(), dt);
    lastPrintMs = now;
  }*/
}