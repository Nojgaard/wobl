#include "servo_task.hpp"
#include "debug.hpp"
#include "servo.hpp"

static constexpr int kServoRxPin = 16;
static constexpr int kServoTxPin = 17;
static constexpr long kServoBaud = 1000000;
static constexpr float kMaxVelocityRps = 1.0f;
static constexpr float kMaxAccelerationRps2 = 0.2f;

static Servo leftHip({.id = 0,
                      .maxVelocityRps = kMaxVelocityRps,
                      .maxAccelerationRps2 = kMaxAccelerationRps2,
                      .coordSign = -1.0f});
static Servo rightHip({.id = 5,
                       .maxVelocityRps = kMaxVelocityRps,
                       .maxAccelerationRps2 = kMaxAccelerationRps2,
                       .coordSign = 1.0f});

static HardwareSerial servoSerial(1); // UART1
static SMS_STS bus;

static long lastCommandTime = 0;
static long lastFeedbackTime = 0;

void servoTaskInit(SharedState &state) {
  servoSerial.begin(kServoBaud, SERIAL_8N1, kServoRxPin, kServoTxPin);
  bus.pSerial = &servoSerial;

  bool leftOk = leftHip.init(bus);
  bool rightOk = rightHip.init(bus);
  DPRINTF("Left hip init: %d", leftOk);
  DPRINTF("Right hip init: %d", rightOk);

  lastCommandTime = millis();
  lastFeedbackTime = millis();
  state.status.servos.write({.leftOk = leftOk, .rightOk = rightOk});
}

static void update(SharedState &state) {
  long now = millis();

  // Commands at 100 Hz — WritePosEx only fires when steps actually change
  if (now - lastCommandTime >= 10) {
    ServosCommand cmd = state.commands.servos.read();
    leftHip.command(cmd.left);
    rightHip.command(cmd.right);
    lastCommandTime = now;
  }

  // Feedback at ~20 Hz — FeedBack() is the expensive UART read
  if (now - lastFeedbackTime >= 50) {
    leftHip.update();
    rightHip.update();

    const Servo::Data &ld = leftHip.data();
    const Servo::Data &rd = rightHip.data();
    state.telemetry.servos.write({
        .left = {ld.valid, ld.positionRad, ld.velocityRps, ld.effortPct},
        .right = {rd.valid, rd.positionRad, rd.velocityRps, rd.effortPct},
    });
    lastFeedbackTime = now;
  }
}

void servoTask(void *parameters) {
  auto state = static_cast<SharedState *>(parameters);
  for (;;) {
    update(*state);
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}
