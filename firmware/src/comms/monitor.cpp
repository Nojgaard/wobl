#include "comms/monitor.hpp"
#include <Arduino.h>

constexpr unsigned long kUpdatePeriodMs = 250;

void Monitor::init() {}

void Monitor::update() {
  static unsigned long lastUpdateTime = 0;
  unsigned long now = millis();

  if (mode == Display::NONE || now - lastUpdateTime < kUpdatePeriodMs)
    return;

  lastUpdateTime = now;

  switch (mode) {
  case Display::SERVO: {
    auto telemetry = _robot.servos.telemetry();
    Serial.printf("SERVO L[p=%.3f e=%.1f] R[p=%.3f e=%.1f]\n",
                  telemetry.left.positionRad, telemetry.left.effortPct,
                  telemetry.right.positionRad, telemetry.right.effortPct);
    break;
  }
  case Display::ROBOT: {
    auto telem = _robot.controller.telemetry();
    Serial.printf("ROBOT hL=%.3f hR=%.3f r=%.3f p=%.3f\n",
                  telem.state.leftLegHeight, telem.state.rightLegHeight,
                  telem.state.roll, telem.state.pitch);
    break;
  }
  case Display::NONE:
    break;
  }
}
