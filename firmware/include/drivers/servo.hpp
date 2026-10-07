#pragma once

#include <SCServo.h>

class Servo {
public:
  struct Config {
    uint8_t id;
    float maxVelocityRps; // rad/s
    float maxAccelerationRps2; // rad/s^2
    float coordSign = 1.0f; // flip sign if servo axis is mechanically mirrored
  };

  struct Data {
    bool valid;
    float positionRad; // rad
    float velocityRps; // rad/s
    float effortPct;
  };

  struct Command {
    bool enabled;
    float positionRad; // rad
  };

  Servo(Config config);
  bool init(SMS_STS &bus);
  void update();
  const Data &data() const;
  float voltage() const;
  void command(const Command &cmd);
  bool calibrate();

private:
  int radiansToSteps(float radians) const;
  float stepsToRadians(int steps) const;

  static constexpr int STEPS_PER_REVOLUTION = 4096;
  static constexpr int MAX_SPEED_STEPS_PER_SECOND = 3400;
  static constexpr int MAX_ACCELERATION_UNITS = 254;
  static constexpr int ACCELERATION_UNIT_STEPS_PER_SECOND_SQUARED = 100;

  Config _config;
  SMS_STS *_bus;
  Data _data;
  bool _enabled;
  int _lastWrittenSteps;
  float _voltage = 0.0f;
};