#pragma once

#include "subsystems/imu_subsystem.hpp"
#include "subsystems/wheel_subsystem.hpp"
#include "subsystems/servo_subsystem.hpp"

class Robot {
public:
  void init();

  ImuSubsystem imu;
  WheelSubsystem wheels;
  ServoSubsystem servos;
private:
};