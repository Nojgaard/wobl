#pragma once
#include "protected.hpp"
#include "calib_state.hpp"
#include "drivers/imu.hpp"
#include "drivers/wheel.hpp"
#include "drivers/servo.hpp"

struct IMUStatus {
  int status;
  float updateRate;
};

struct WheelsStatus {
  float focRate;
  float updateRate;
  int leftStatus;
  int rightStatus;
};

struct WheelsData {
  Wheel::Data left;
  Wheel::Data right;
};

struct ServosData {
  Servo::Data left;
  Servo::Data right;
};

struct SensorState {
  Protected<IMU::Data>  imu;
  Protected<WheelsData> wheels;
  Protected<ServosData> servos;
};

struct WheelsCommand {
  Wheel::Command left;
  Wheel::Command right;
};

struct ServosCommand {
  Servo::Command left;
  Servo::Command right;
};

struct ActuatorCommands {
  Protected<WheelsCommand> wheels;
  Protected<ServosCommand> servos;
};

struct ServosStatus {
  bool leftOk;
  bool rightOk;
};

struct DeviceStatus {
  Protected<IMUStatus> imu;
  Protected<WheelsStatus> wheels;
  Protected<ServosStatus> servos;
};

struct SharedState {
  DeviceStatus     status;
  CalibState       calibration;
  SensorState      telemetry;
  ActuatorCommands commands;
};