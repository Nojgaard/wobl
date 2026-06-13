/**
 * Dual-motor torque-matching test for WOBL wheels.
 *
 * Drives both wheels with the same estimated-current torque target and
 * prints their velocities every 2 s so you can verify they match.
 * SimpleFOC estimated-current mode: phase_resistance + KV_rating are set,
 * so move(A) is converted internally to voltage via U = R*I + Ke*ω.
 *
 * Serial commands (no space, e.g. "T0.5" not "T 0.5"):
 *   T<amps>   — change target current (default 0.3 A, min 0.05 A)
 *   P         — print velocities once immediately
 *
 * Build:   pio run -e torque_match
 * Upload:  pio run -e torque_match -t upload
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

BLDCMotor motorLeft(leftWheelConfig.polePairs,
                    leftWheelConfig.phaseResistance,
                    leftWheelConfig.kvRating);
BLDCMotor motorRight(rightWheelConfig.polePairs,
                     rightWheelConfig.phaseResistance,
                     rightWheelConfig.kvRating);

float targetCurrent = 0.0f;

static float sumL = 0, sumR = 0;
static uint32_t sampleCount = 0;

Commander command(Serial);

void doTarget(char *cmd) {
  command.scalar(&targetCurrent, cmd);
  Serial.printf("target current -> %.3f A\r\n", targetCurrent);
}

void printVelocities(char *) {
  float omL = (sampleCount > 0) ? sumL / sampleCount : motorLeft.shaft_velocity;
  float omR = (sampleCount > 0) ? sumR / sampleCount : motorRight.shaft_velocity;
  Serial.printf("left=%6.1f rad/s  right=%6.1f rad/s  delta=%+.1f rad/s  (n=%lu)\r\n",
                omL, omR, omL - omR, sampleCount);
}

void initMotor(BLDCMotor &motor, BLDCDriver3PWM &driver, MagneticSensorI2C &sensor,
               const Wheel::Config &cfg) {
  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);

  motor.phase_resistance = cfg.phaseResistance;
  //motor.KV_rating = cfg.kvRating;
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::estimated_current;

  // Apply stored calibration so initFOC() skips the physical alignment sweep.
  // Without this, the motor moves during init and leaves shaft_velocity stuck
  // at a non-zero value (SimpleFOC only updates velocity when delta_angle != 0).
  motor.sensor_direction = cfg.sensor_direction;
  motor.zero_electric_angle = cfg.zero_electric_angle;

  motor.LPF_velocity.Tf = 0.02f;

  driver.voltage_power_supply = 12.0f;
  driver.voltage_limit = 3.0f;
  driver.init();

  motor.init();
  motor.initFOC();
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("WOBL torque-match test — lift both wheels!");

  Serial.println("Send any key to start...");
  while (!Serial.available()) {
    delay(100);
  }
  Serial.flush();
  Serial.println("Starting...");

  Serial.println("Commands: T<amps>  P");

  SimpleFOCDebug::enable(&Serial);

  wire0.begin(21, 22, 400000);
  wire1.begin(25, 26, 400000);

  sensorLeft.min_elapsed_time = 0.001f;
  sensorLeft.init(&wire0);
  sensorRight.min_elapsed_time = 0.001f;
  sensorRight.init(&wire1);

  Serial.println("Init Left Motor...");
  initMotor(motorLeft, driverLeft, sensorLeft, leftWheelConfig);

  Serial.println("Init Right Motor...");
  initMotor(motorRight, driverRight, sensorRight, rightWheelConfig);

  command.add('T', doTarget, "target current");
  command.add('P', printVelocities, "print velocities");
}

static uint32_t lastPrint = 0;

void loop() {
  motorLeft.loopFOC();
  motorLeft.move(targetCurrent);
  motorRight.loopFOC();
  motorRight.move(targetCurrent);
  command.run();

  sumL += motorLeft.shaft_velocity;
  sumR += motorRight.shaft_velocity;
  sampleCount++;

  if (millis() - lastPrint >= 2000) {
    lastPrint = millis();
    printVelocities(nullptr);
    sumL = 0; sumR = 0; sampleCount = 0;
  }
}
