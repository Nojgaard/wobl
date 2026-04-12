#pragma once

#include <SCServo.h>

class Servo {
public:
  struct Config {
    uint8_t id;
    float maxVelocityRps;
    float maxAccelerationRps2;
    float coordSign = 1.0f; // flip sign if servo axis is mechanically mirrored
  };

  struct Data {
    bool valid;
    float positionRad;
    float velocityRps;
    float effortPct;
  };

  struct Command {
    bool enabled;
    float positionRad;
  };

  Servo(Config config);
  bool init(SMS_STS &bus);
  void update();
  const Data &data() const;
  void command(const Command &cmd);

private:
  int radiansToSteps(float radians) const;
  float stepsToRadians(int steps) const;

  static constexpr int STEPS_PER_REVOLUTION = 4096;
  static constexpr int MAX_SPEED_STEPS = 6800;
  static constexpr int MAX_ACCELERATION_STEPS = 254;

  Config _config;
  SMS_STS *_bus;
  Data _data;
  bool _enabled;
  int _lastWrittenSteps;
};