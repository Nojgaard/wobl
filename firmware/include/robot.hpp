#pragma once

#include "subsystems/imu_subsystem.hpp"
#include "subsystems/wheel_subsystem.hpp"
#include "subsystems/servo_subsystem.hpp"

#include "control/motion_controller.hpp"

class Robot {
public:
  void init();

  bool isOperational();

  ImuSubsystem imu;
  WheelSubsystem wheels;
  ServoSubsystem servos;

  MotionController controller;
private:
};