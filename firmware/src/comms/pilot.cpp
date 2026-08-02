#include "comms/pilot.hpp"
#include <Arduino.h>
#include <Bluepad32.h>

static ControllerPtr gamepad = nullptr;

static constexpr float AXIS_DEADZONE = 30;
static constexpr float AXIS_MAX = 520;

static constexpr float MAX_FWD_VEL = 0.2f;  // m/s
static constexpr float MAX_TURN_VEL = 0.08f; 

float normalizeAxis(int32_t value) {
  if (abs(value) < AXIS_DEADZONE) {
    return 0.0f;
  }
  return (float)value / AXIS_MAX;
}

bool hasData() {
  return gamepad && gamepad->isConnected() && gamepad->hasData();
}

void onConnectedController(ControllerPtr ctl) {
  Serial.printf("Controller connected — %s\n", ctl->getModelName());
  if (ctl->isGamepad()) {
    gamepad = ctl;
  }
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

  BP32.setup(&onConnectedController, &onDisconnectedController, false);
  // BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);

  _lastUpdateMs = millis();
}

void Pilot::update() {
  unsigned long now = millis();
  float dt = (now - _lastUpdateMs) / 1000.0f;
  _lastUpdateMs = now;
  
  if (!BP32.update() || !hasData()) {
    return;
  }

  if (_pressedStart && !gamepad->miscStart()) {
    _enableController = !(_robot.controller.command().enable);
    Serial.printf("Controller %s\n",
                  _enableController ? "ENABLED" : "DISABLED");
  }
  _pressedStart = gamepad->miscStart();

  _tarFwdVel.Ts = dt;
  _tarTurnVel.Ts = dt;
  float tarFwdVel = _tarFwdVel(normalizeAxis(-gamepad->axisY()) * MAX_FWD_VEL);
  float tarTurnVel = _tarTurnVel(normalizeAxis(gamepad->axisRX()) * MAX_TURN_VEL);

  _robot.controller.command({.enable = _enableController,
                             .forwardVelocity = tarFwdVel,
                             .turnVelocity = tarTurnVel});

  /*Serial.printf("buttons: 0x%04x, axis L: %4d, %4d, "
                "axis R: %4d, %4d, start: 0x%04x, sel: 0x%04x\n",
                gamepad->buttons(),
                gamepad->axisX(), gamepad->axisY(), gamepad->axisRX(),
                gamepad->axisRY(), gamepad->miscStart(),
     gamepad->miscSelect());*/
}