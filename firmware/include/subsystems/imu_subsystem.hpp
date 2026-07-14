#pragma once

#include "drivers/imu.hpp"
#include "protected.hpp"

class ImuSubsystem {
public:
  struct Status {
    uint8_t status;
    float syncRateHz;
  };

  struct Telemetry {
    float roll, pitch, yaw;             // euler angles (rad)
    float rollRate, pitchRate, yawRate; // angular velocities (rad/s)
  };

  enum class State {
    Operational,
    Calibrating,
  };

  void init();
  void loop();
  void calibrate();

  Status status();
  Telemetry telemetry();
  IMU::Data telemetryRaw();
  IMU::Calibration calibration();

private:
  void update();

  Protected<State> _state;
  Protected<Status> _status;
  Protected<Telemetry> _telemetry;
  Protected<IMU::Data> _telemetryRaw;
  Protected<IMU::Calibration> _calibration;

  IMU _imu;
  long _lastSyncTime = 0;
  int _readsSinceLastSync = 0;
};