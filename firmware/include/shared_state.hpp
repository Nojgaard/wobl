#pragma once
#include "imu.hpp"
#include "wheel.hpp"
#include "servo.hpp"

template <typename T> class Protected {
  T _val;
  portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;

public:
  void write(const T &v) {
    taskENTER_CRITICAL(&_mux);
    _val = v;
    taskEXIT_CRITICAL(&_mux);
  }
  T read() {
    taskENTER_CRITICAL(&_mux);
    T tmp = _val;
    taskEXIT_CRITICAL(&_mux);
    return tmp;
  }

public:
  Protected() = default;
  Protected(const Protected &) = delete;
  Protected &operator=(const Protected &) = delete;
};

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
  Protected<IMU::Data> imu;
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
  DeviceStatus status;
  SensorState telemetry;
  ActuatorCommands commands;
};