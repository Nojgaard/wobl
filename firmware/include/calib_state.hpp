#pragma once

#include "protected.hpp"
#include "imu.hpp"

// Bare trigger: device task saves its current learned state to NVS.
struct CalibReq {
  bool pending = false;
};

struct CalibState {
  Protected<IMU::Calibration> imuData;
  Protected<CalibReq>         imuReq;
  // Wheel and servo calibration slots added when implemented.
};
