#include "wheel.hpp"
#include "debug.hpp"
#include <Preferences.h>

static constexpr const char *kNvsKeyZeroAngle = "cal_zea";
static constexpr const char *kNvsKeyDirection = "cal_dir";
static constexpr const char *kNvsKeyTuningP = "tun_p";
static constexpr const char *kNvsKeyTuningI = "tun_i";
static constexpr const char *kNvsKeyTuningD = "tun_d";
static constexpr const char *kNvsKeyTuningLpf = "tun_lpf";
static constexpr const char *kNvsKeyVelocityLimit = "tun_vel";
static constexpr const char *kNvsKeyVoltageLimit = "tun_vlt";

Wheel::Wheel(Config config)
    : _data{}, _command{false, 0.0}, _id(config.id), _sensor(AS5600_I2C),
      _driver(config.pinA, config.pinB, config.pinC, config.pinEnable),
      _motor(config.polePairs, config.phaseResistance) {}

static const char *nvsCalibNamespace(Wheel::Id id) {
  return id == Wheel::Id::Left ? "wobl_whl_cl" : "wobl_whl_cr";
}

static const char *nvsTuningNamespace(Wheel::Id id) {
  return id == Wheel::Id::Left ? "wobl_whl_tl" : "wobl_whl_tr";
}

bool Wheel::loadCalibrationFromNvs() {
  Preferences prefs;
  bool success = false;
  if (prefs.begin(nvsCalibNamespace(_id), true)) {
    _motor.zero_electric_angle = prefs.getFloat(kNvsKeyZeroAngle);
    _motor.sensor_direction =
        static_cast<Direction>(prefs.getInt(kNvsKeyDirection));
    prefs.end();
    success = true;
  }
  return success;
}

bool Wheel::saveCalibrationToNvs() {
  Preferences prefs;
  bool success = false;
  if (prefs.begin(nvsCalibNamespace(_id), false)) {
    prefs.putFloat(kNvsKeyZeroAngle, _motor.zero_electric_angle);
    prefs.putInt(kNvsKeyDirection,
                 static_cast<int32_t>(_motor.sensor_direction));
    prefs.end();
    success = true;
  }
  return success;
}

bool Wheel::loadTuningFromNvs() {
  Preferences prefs;
  bool success = false;
  if (prefs.begin(nvsTuningNamespace(_id), true)) {
    VelocityTuning tuning;
    tuning.p = prefs.getFloat(kNvsKeyTuningP, tuning.p);
    tuning.i = prefs.getFloat(kNvsKeyTuningI, tuning.i);
    tuning.d = prefs.getFloat(kNvsKeyTuningD, tuning.d);
    tuning.lpf_velocity_tf =
        prefs.getFloat(kNvsKeyTuningLpf, tuning.lpf_velocity_tf);
    tuning.velocity_limit =
        prefs.getFloat(kNvsKeyVelocityLimit, tuning.velocity_limit);
    tuning.voltage_limit =
        prefs.getFloat(kNvsKeyVoltageLimit, tuning.voltage_limit);
    prefs.end();

    tune(tuning);
    success = true;
  }
  return success;
}

bool Wheel::saveTuningToNvs() {
  Preferences prefs;
  bool success = false;
  if (prefs.begin(nvsTuningNamespace(_id), false)) {
    prefs.putFloat(kNvsKeyTuningP, _motor.PID_velocity.P);
    prefs.putFloat(kNvsKeyTuningI, _motor.PID_velocity.I);
    prefs.putFloat(kNvsKeyTuningD, _motor.PID_velocity.D);
    prefs.putFloat(kNvsKeyTuningLpf, _motor.LPF_velocity.Tf);
    prefs.putFloat(kNvsKeyVelocityLimit, _motor.velocity_limit);
    prefs.putFloat(kNvsKeyVoltageLimit, _motor.voltage_limit);
    prefs.end();
    success = true;
  }

  return success;
}

int Wheel::init(float voltage_supply, float voltage_limit, TwoWire &wire) {
  _status = Status::Uninitialized;

  // Probe the AS5600 before handing off to SimpleFOC.
  wire.beginTransmission(0x36);
  if (wire.endTransmission() != 0) {
    DPRINTF("[wheel] sensor not found on this bus\r\n");
    return static_cast<int>(_status); // sensor not found on this bus
  }

  int status = 1;
  _sensor.min_elapsed_time = 0.001f; // 1 ms = 1 kHz update rate
  _sensor.init(&wire);

  _motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  _motor.linkSensor(&_sensor);
  _motor.current_limit = 1.0f;

  _driver.voltage_power_supply = voltage_supply;
  _driver.voltage_limit = voltage_limit;
  status = _driver.init();
  if (status != 1) {
    return static_cast<int>(_status); // driver failed to initialize
  }

  _motor.linkDriver(&_driver);

  _motor.controller = MotionControlType::velocity;
  //_motor.controller = MotionControlType::torque;
  _motor.torque_controller = TorqueControlType::estimated_current;

  _motor.voltage_sensor_align = 5.0f;
  _motor.motion_downsample = 2; // Try to stabilize velocity estimation
  if (!loadTuningFromNvs()) {
    tune(VelocityTuning());
  }
  _motor.LPF_angle.Tf = 0.001f;

  status = _motor.init();
  if (status != 1) {
    return static_cast<int>(_status);
  }

  _status = Status::ReadyForCalibration;

  bool calibrated = loadCalibrationFromNvs();

  if (!calibrated)
    return static_cast<int>(_status);

  int foc = _motor.initFOC();
  if (foc == 1 && _motor.sensor_direction == Direction::UNKNOWN) {
    // Sensor responded but direction could not be determined — likely swapped
    // sensor/motor
    _status = Status::MotorError;
    return static_cast<int>(_status);
  }
  _status = (foc == 1) ? Status::Operational : Status::ReadyForCalibration;
  return foc;
}

const Wheel::Data &Wheel::data() const { return _data; }

Wheel::Calibration Wheel::calibration() const {
  return {
      .zero_electric_angle = _motor.zero_electric_angle,
      .sensor_direction = _motor.sensor_direction,
  };
}

Wheel::VelocityTuning Wheel::tuning() const {
  return {
      .p = _motor.PID_velocity.P,
      .i = _motor.PID_velocity.I,
      .d = _motor.PID_velocity.D,
      .lpf_velocity_tf = _motor.LPF_velocity.Tf,
      .velocity_limit = _motor.velocity_limit,
      .voltage_limit = _motor.voltage_limit,
  };
}

bool Wheel::isOk() const { return _status == Status::Operational; }

Wheel::Status Wheel::status() const { return _status; }

void Wheel::tune(const VelocityTuning &tuning, bool persist) {
  _motor.PID_velocity.P = tuning.p;
  _motor.PID_velocity.I = tuning.i;
  _motor.PID_velocity.D = tuning.d;
  _motor.LPF_velocity.Tf = tuning.lpf_velocity_tf;

  _motor.updateVelocityLimit(tuning.velocity_limit);
  _motor.updateVoltageLimit(tuning.voltage_limit);

  if (persist)
    saveTuningToNvs();
}

bool Wheel::calibrate() {
  if (_status == Status::Uninitialized)
    return false;

  _motor.disable();
  _command.enabled = false;
  _motor.sensor_direction = Direction::UNKNOWN;
  _motor.zero_electric_angle = NOT_SET;
  _motor.enable(); // re-enable driver hardware for alignment procedure

  int foc = _motor.initFOC();
  if (foc != 1 || _motor.sensor_direction == Direction::UNKNOWN) {
    _status = Status::MotorError;
    return false;
  }

  _status = Status::Operational;
  saveCalibrationToNvs();
  return true;
}

void Wheel::command(bool enabled, float velocity) {
  if (!isOk())
    return;

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
  if (_motor.motor_status == FOCMotorStatus::motor_error)
    _status = Status::MotorError;

  if (!isOk())
    return;

  // Back-EMF feed-forward: voltage needed to maintain commanded velocity
  // Kv = 170 RPM/V → 17.8 rad/s/V → coeff = 1 / 17.8 ≈ 0.056
  float backEmf = _command.velocity * 0.056f;

  // Coulomb friction as a saturating linear ramp — smooth through zero
  float absVel = fabsf(_command.velocity);
  float frictionMag;
  if (absVel < 1.5f) {
    frictionMag = absVel * (0.3f / 1.5f); // ramp: 0 → 0.3V over 0 → 1.5 rad/s
  } else {
    frictionMag = 0.3f; // saturate at max friction
  }
  float friction = (_command.velocity >= 0) ? frictionMag : -frictionMag;

  _motor.feed_forward_voltage.q = backEmf + friction;

  _motor.loopFOC();
  _motor.move(_command.velocity);

  _data.angle = _motor.shaft_angle;
  _data.velocity = _motor.shaft_velocity;
}