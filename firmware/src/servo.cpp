#include "servo.hpp"

#include <algorithm>
#include <math.h>

Servo::Servo(Config config)
    : _config(config), _bus(nullptr), _data{}, _enabled(false),
      _lastWrittenSteps(-1) {}

bool Servo::init(SMS_STS &bus) {
  _bus = &bus;
  if (_bus->Ping(_config.id) == -1)
    return false;
  _bus->EnableTorque(_config.id, false);
  return true;
}

const Servo::Data &Servo::data() const { return _data; }

void Servo::command(const Command &cmd) {
  if (cmd.enabled != _enabled) {
    _enabled = cmd.enabled;
    _bus->EnableTorque(_config.id, cmd.enabled);
    _lastWrittenSteps = -1; // force re-write on next command
  }

  if (!cmd.enabled)
    return;

  int positionSteps = radiansToSteps(_config.coordSign * cmd.positionRad) +
                      STEPS_PER_REVOLUTION / 2;
  positionSteps = std::clamp(positionSteps, 0, STEPS_PER_REVOLUTION - 1);
  if (positionSteps == _lastWrittenSteps)
    return;

  int velocitySteps = radiansToSteps(_config.maxVelocityRps);
  velocitySteps = std::clamp(velocitySteps, 0, MAX_SPEED_STEPS);

  int accelerationSteps = radiansToSteps(_config.maxAccelerationRps2);
  accelerationSteps = std::clamp(accelerationSteps, 0, MAX_ACCELERATION_STEPS);

  _bus->WritePosEx(_config.id, positionSteps, velocitySteps, accelerationSteps);
  _lastWrittenSteps = positionSteps;
}

void Servo::update() {
  _data.valid = _bus->FeedBack(_config.id) != -1;
  if (!_data.valid)
    return;

  _data.positionRad =
      _config.coordSign *
      stepsToRadians(_bus->ReadPos(-1) - STEPS_PER_REVOLUTION / 2);
  _data.velocityRps = _config.coordSign * stepsToRadians(_bus->ReadSpeed(-1));
  _data.effortPct = _bus->ReadLoad(-1) * 0.1f;
}

int Servo::radiansToSteps(float radians) const {
  return static_cast<int>(radians * STEPS_PER_REVOLUTION / (2.0f * M_PI));
}

float Servo::stepsToRadians(int steps) const {
  return steps * 2.0f * M_PI / STEPS_PER_REVOLUTION;
}
