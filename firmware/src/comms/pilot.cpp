#include "comms/pilot.hpp"
#include <Arduino.h>
#include <Bluepad32.h>

static ControllerPtr gamepad = nullptr;

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

  BP32.setup(&onConnectedController, &onDisconnectedController, true);
  //BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);
}

void Pilot::update() {
  if (!BP32.update() || !hasData()) {
    return;
  }

  if (pressedStart && !gamepad->miscStart()) {
    bool enable = !(_robot.controller.command().enable);
    _robot.controller.command({enable, 0.0f, 0.0f});
    Serial.printf("Controller %s\n", enable ? "ENABLED" : "DISABLED");
  }

  pressedStart = gamepad->miscStart();

  /*Serial.printf("buttons: 0x%04x, axis L: %4d, %4d, "
                "axis R: %4d, %4d, start: 0x%04x, sel: 0x%04x\n",
                gamepad->buttons(),
                gamepad->axisX(), gamepad->axisY(), gamepad->axisRX(),
                gamepad->axisRY(), gamepad->miscStart(), gamepad->miscSelect());*/
}