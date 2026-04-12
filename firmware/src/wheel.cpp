#include "wheel.hpp"

Wheel::Wheel(Config config)
    : _data{}, _command{false, 0.0}, _calibration(config.calibration), _sensor(AS5600_I2C),
      _driver(config.pinA, config.pinB, config.pinC, config.pinEnable),
      _motor(config.polePairs) {}

int Wheel::init(float voltage_supply, float voltage_limit, TwoWire &wire) {
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
  _motor.LPF_velocity.Tf = 0.005f;

  // velocity PI controller parameters
  _motor.PID_velocity.P = 0.2f;
  _motor.PID_velocity.I = 20;
  _motor.PID_velocity.D = 0;

  status = _motor.init();
  if (status != 1) {
    return status;
  }

  if (_calibration.calibrated) {
    _motor.zero_electric_angle = _calibration.zero_electric_angle;
    _motor.sensor_direction = _calibration.sensor_direction;
  }

  return _motor.initFOC();
}

const Wheel::Data &Wheel::data() { return _data; }

void Wheel::command(bool enabled, float velocity) {
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
  _motor.loopFOC();
  _motor.move(_command.velocity);

  _data.angle = _motor.shaftAngle();
  _data.velocity = _motor.shaftVelocity();
}