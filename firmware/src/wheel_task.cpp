#include "wheel_task.hpp"
#include "shared_state.hpp"
#include "wheel.hpp"
#include "debug.hpp"

Wheel::Config leftWheelConfig{
    .polePairs = 7, .pinA = 12, .pinB = 14, .pinC = 27, .pinEnable = 13};

Wheel::Config rightWheelConfig{
    .polePairs = 7, .pinA = 4, .pinB = 33, .pinC = 32, .pinEnable = 2};

WheelsStatus status{
    .updateRate = 0.0,
    .leftStatus = 0,
    .rightStatus = 0,
};

static Wheel leftWheel(leftWheelConfig);
static Wheel rightWheel(rightWheelConfig);

static TwoWire wire0(0);
static TwoWire wire1(1);

static constexpr long kClockSpeed = 400000; // 400 kHz
static constexpr float kVoltageSupply = 12.0;
static constexpr float kVoltageLimit = 3.0;

static long lastUpdateTime = 0;

void wheelTaskInit(SharedState &state) {
  wire0.begin(21, 22, kClockSpeed);
  wire1.begin(25, 26, kClockSpeed);

  #ifdef DEBUG
    SimpleFOCDebug::enable(&Serial);
  #endif

  status.leftStatus = leftWheel.init(kVoltageSupply, kVoltageLimit, wire0);
  status.rightStatus = rightWheel.init(kVoltageSupply, kVoltageLimit, wire1);

  DPRINTF("Left wheel status: %d", status.leftStatus);
  DPRINTF("Right wheel status: %d", status.rightStatus);

  lastUpdateTime = millis();
  state.status.wheels.write(status);

  SimpleFOCDebug::enable(nullptr); // disable debug after init
}

static void update(SharedState &state) {
  leftWheel.update();
  rightWheel.update();

  // Command + telemetry sync at 200 Hz
  long now = millis();
  if (now - lastUpdateTime < 5) {
    return;
  }

  WheelsCommand command = state.commands.wheels.read();
  leftWheel.command(command.left.enabled, command.left.velocity);
  rightWheel.command(command.right.enabled, command.right.velocity);

  state.telemetry.wheels.write({.left = leftWheel.data(), .right = rightWheel.data()});

  status.updateRate = 1000.0 / (now - lastUpdateTime);
  lastUpdateTime = now;
  state.status.wheels.write(status);
}

void wheelTask(void *parameters) {
  auto state = static_cast<SharedState *>(parameters);

  for (;;) {
    update(*state);
  }
}