#include "robot.hpp"

static void loopWheels(void *param) {
  static_cast<WheelSubsystem *>(param)->loop();
}

static void loopImu(void *param) { static_cast<ImuSubsystem *>(param)->loop(); }

static void loopServos(void *param) {
  static_cast<ServoSubsystem *>(param)->loop();
}

void Robot::init() {
  wheels.init();
  imu.init();
  servos.init();

  xTaskCreatePinnedToCore(loopWheels, "foc", 4096, &wheels, 24, NULL, 1);
  xTaskCreatePinnedToCore(loopImu, "imu", 4096, &imu, 10, NULL, 0);
  xTaskCreatePinnedToCore(loopServos, "servo", 4096, &servos, 10, NULL, 0);
}