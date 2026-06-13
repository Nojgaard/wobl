/**
 * Dual-motor KV rating estimator for WOBL wheels.
 *
 * KV is printed automatically every 2 s for both motors.
 * Serial commands (no space after letter, e.g. "T2" not "T 2"):
 *   T<volts>   — change target voltage (default 1 V, min 0.1 V)
 *   K          — print KV once immediately
 *
 * Build:   pio run -e kv_rating
 * Upload:  pio run -e kv_rating -t upload
 * Monitor: pio device monitor -b 115200
 */

#include "config.hpp"
#include <SimpleFOC.h>

static TwoWire wire0(0), wire1(1);

MagneticSensorI2C sensorLeft(AS5600_I2C);
MagneticSensorI2C sensorRight(AS5600_I2C);

BLDCDriver3PWM driverLeft(leftWheelConfig.pinA, leftWheelConfig.pinB,
                          leftWheelConfig.pinC, leftWheelConfig.pinEnable);
BLDCDriver3PWM driverRight(rightWheelConfig.pinA, rightWheelConfig.pinB,
                           rightWheelConfig.pinC, rightWheelConfig.pinEnable);

BLDCMotor motorLeft(leftWheelConfig.polePairs);
BLDCMotor motorRight(rightWheelConfig.polePairs);

float targetVoltage = 1.0f;

Commander command(Serial);

void doTarget(char *cmd) {
  command.scalar(&targetVoltage, cmd);
  Serial.printf("target voltage -> %.2f V\r\n", targetVoltage);
}
void printKV(char *) {
  float omL = motorLeft.shaft_velocity;
  float omR = motorRight.shaft_velocity;
  float kv = 30.0f / (PI * targetVoltage);
  Serial.printf("left   omega=%6.1f rad/s  KV=%.1f rpm/V\r\n", omL, omL * kv);
  Serial.printf("right  omega=%6.1f rad/s  KV=%.1f rpm/V\r\n", omR, omR * kv);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("WOBL dual-wheel KV estimator — lift both wheels!");

  Serial.println("Send any key to start tests...");
  while (!Serial.available()) {
    delay(100);
  }
  Serial.flush();
  Serial.println("Starting...");

  Serial.println("Commands: T <volts>  K");

  wire0.begin(21, 22, 400000);
  wire1.begin(25, 26, 400000);

  sensorLeft.init(&wire0);
  sensorRight.init(&wire1);

  motorLeft.linkSensor(&sensorLeft);
  motorRight.linkSensor(&sensorRight);

  driverLeft.voltage_power_supply = 12.0f;
  driverRight.voltage_power_supply = 12.0f;
  driverLeft.voltage_limit = 3.0f;
  driverRight.voltage_limit = 3.0f;
  driverLeft.init();
  driverRight.init();

  motorLeft.linkDriver(&driverLeft);
  motorRight.linkDriver(&driverRight);

  motorLeft.controller = MotionControlType::torque;
  motorRight.controller = MotionControlType::torque;
  motorLeft.torque_controller = TorqueControlType::voltage;
  motorRight.torque_controller = TorqueControlType::voltage;

  motorLeft.init();
  motorRight.init();
  motorLeft.LPF_velocity.Tf = 0.1f;
  motorRight.LPF_velocity.Tf = 0.1f;
  motorLeft.initFOC();
  motorRight.initFOC();

  command.add('T', doTarget, "target voltage");
  command.add('K', printKV, "print KV");
}

static uint32_t lastPrint = 0;

void loop() {
  motorLeft.loopFOC();
  motorLeft.move(targetVoltage);
  motorRight.loopFOC();
  motorRight.move(targetVoltage);
  command.run();

  if (millis() - lastPrint >= 2000) {
    lastPrint = millis();
    printKV(nullptr);
  }
}
