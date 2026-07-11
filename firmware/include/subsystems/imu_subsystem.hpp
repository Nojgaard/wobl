#pragma once

#include "drivers/imu.hpp"
#include "protected.hpp"

class ImuSubsystem {
public:
  struct Status {
    byte status;
    float syncRateHz;
  };

  enum class State {
    Operational,
    Calibrating,
  };

  void init();
  void loop();
  void calibrate();

  Status status();
  IMU::Data telemetry();
  IMU::Calibration calibration();

private:
  void update();

  Protected<State> _state;
  Protected<Status> _status;
  Protected<IMU::Data> _telemetry;
  Protected<IMU::Calibration> _calibration;

  IMU _imu;
  long _lastSyncTime = 0;
  int _readsSinceLastSync = 0;
};