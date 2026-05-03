#include "wheel.hpp"

Wheel::Wheel(Config config)
    : _data{}, _command{false, 0.0}, _calibration(config.calibration), _sensor(AS5600_I2C),
      _driver(config.pinA, config.pinB, config.pinC, config.pinEnable),
      _motor(config.polePairs) {}

int Wheel::init(float voltage_supply, float voltage_limit, TwoWire &wire) {
  // Probe the AS5600 before handing off to SimpleFOC.
  wire.beginTransmission(0x36);
  if (wire.endTransmission() != 0) {
    return 0; // sensor not found on this bus
  }

  int status = 1;
  _sensor.init(&wire);
  _motor.linkSensor(&_sensor);

  _driver.voltage_power_supply = voltage_supply;
  _driver.voltage_limit = voltage_limit;
  status = _driver.init();
  if (status != 1) {
    return status;
  }

  _motor.linkDriver(&_driver);
  _motor.velocity_limit = 40; // rad/s
  _motor.voltage_limit = voltage_limit;
  _motor.controller = MotionControlType::velocity;

  // velocity low pass filtering
  // default 5ms - try different values to see what is the best.
  // the lower the less filtered
  _motor.LPF_velocity.Tf = 0.01f;

  // velocity PI controller parameters
  _motor.PID_velocity.P = 0.1f;
  _motor.PID_velocity.I = 0.5f;
  _motor.PID_velocity.D = 0.0f;

  status = _motor.init();
  if (status != 1) {
    return status;
  }

  if (_calibration.calibrated) {
    _motor.zero_electric_angle = _calibration.zero_electric_angle;
    _motor.sensor_direction = _calibration.sensor_direction;
  }

  int foc = _motor.initFOC();
  if (foc == 1 && _motor.sensor_direction == Direction::UNKNOWN) {
    // Sensor responded but direction could not be determined — likely swapped sensor/motor
    return 0;
  }
  _initOk = (foc == 1);
  return foc;
}

const Wheel::Data &Wheel::data() { return _data; }

bool Wheel::isOk() const { return _initOk; }

void Wheel::command(bool enabled, float velocity) {
  if (!_initOk) return;

  _command.enabled = enabled;
  _command.velocity = velocity;

  if (enabled == _motor.enabled) {
    return;
  } else if (enabled) {
    _motor.enable();
  } else {
    _motor.disable();
  }
}

void Wheel::update() {
  if (!_initOk) return;

  _motor.loopFOC();
  _motor.move(_command.velocity);

  _data.angle = _motor.shaftAngle();
  _data.velocity = _motor.shaftVelocity();
}