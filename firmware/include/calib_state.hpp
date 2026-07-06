#pragma once

#include "protected.hpp"
#include "drivers/imu.hpp"
#include "drivers/wheel.hpp"

// Bare trigger: device task saves its current learned state to NVS.
struct CalibReq {
  bool pending = false;
};

struct WheelTuningReq {
  bool pending = false;
  bool persist = false;
  Wheel::VelocityTuning tuning;
};

struct CalibState {
  Protected<IMU::Calibration> imuData;
  Protected<CalibReq>         imuReq;

  Protected<CalibReq> servosCalibReq;

  Protected<Wheel::Calibration> leftWheelCalibData;
  Protected<Wheel::Calibration> rightWheelCalibData;
  Protected<CalibReq>           leftWheelCalibReq;
  Protected<CalibReq>           rightWheelCalibReq;

  Protected<Wheel::VelocityTuning> leftWheelTuningData;
  Protected<Wheel::VelocityTuning> rightWheelTuningData;
  Protected<WheelTuningReq>        leftWheelTuningReq;
  Protected<WheelTuningReq>        rightWheelTuningReq;
};
