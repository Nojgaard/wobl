#include "robot.hpp"

static void loopWheels(void *param) {
  static_cast<WheelSubsystem *>(param)->loop();
}

static void loopImu(void *param) { static_cast<ImuSubsystem *>(param)->loop(); }

static void loopServos(void *param) {
  static_cast<ServoSubsystem *>(param)->loop();
}

static void loopControl(void *param) {
  Robot *robot = static_cast<Robot *>(param);

  for (;;) {
    if (!robot->isOperational()) {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    auto ctrlOut = robot->controller.update(robot->imu.telemetry(),
                                            robot->wheels.telemetry(),
                                            robot->servos.telemetry());
    robot->wheels.command(ctrlOut.wheels);
    robot->servos.command(ctrlOut.servos);
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

bool Robot::isOperational() {
  return imu.status().status &&   //
         wheels.status().left &&  //
         wheels.status().right && //
         servos.status().left &&  //
         servos.status().right;
}

void Robot::init() {
  wheels.init();
  imu.init();
  servos.init();
  controller.init();

  xTaskCreatePinnedToCore(loopWheels, "foc", 4096, &wheels, 24, NULL, 1);
  xTaskCreatePinnedToCore(loopImu, "imu", 4096, &imu, 16, NULL, 0);
  xTaskCreatePinnedToCore(loopServos, "servo", 4096, &servos, 4, NULL, 0);
  xTaskCreatePinnedToCore(loopControl, "controller", 4096, this, 10, NULL, 0);
}