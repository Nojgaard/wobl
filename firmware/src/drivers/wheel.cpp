#include "drivers/wheel.hpp"
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
      _motor(config.polePairs, config.phaseResistance, config.kvRating) {}

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

bool pingAS5600(TwoWire &wire) {
  for (int i = 0; i < 5; ++i) {
    wire.beginTransmission(0x36);
    if (wire.endTransmission() == 0) {
      return true;
    }
    delay(100);
  }
  return false;
}

int Wheel::init(float voltage_supply, float voltage_limit, TwoWire &wire) {
  _status = Status::Uninitialized;

  if (!pingAS5600(wire)) {
    DPRINTF("[wheel] sensor not found on this bus\r\n");
    return static_cast<int>(_status); // sensor not found on this bus
  }

  int status = 1;
  _sensor.min_elapsed_time = 0.010f; // 10 ms = 100 Hz update rate
  _sensor.init(&wire);

  _motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  _motor.linkSensor(&_sensor);
  _motor.current_limit = 1.5f;

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
  //_motor.motion_downsample = 2; // Try to stabilize velocity estimation
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
  return _tuning;
}

bool Wheel::isOk() const { return _status == Status::Operational; }

Wheel::Status Wheel::status() const { return _status; }

void Wheel::tune(const VelocityTuning &tuning, bool persist) {
  _tuning = tuning;

  _motor.PID_velocity.P = tuning.p;
  _motor.PID_velocity.I = tuning.i;
  _motor.PID_velocity.D = tuning.d;
  _motor.LPF_velocity.Tf = tuning.lpf_velocity_tf;

  _motor.updateVelocityLimit(tuning.velocity_limit);
  _motor.updateVoltageLimit(tuning.voltage_limit);
  _motor.PID_velocity.output_ramp = tuning.output_ramp;
  
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

void addFeedForwardEffects(BLDCMotor &motor, const Wheel::Command &command) {
  float ffVoltage = 0.0f;

  constexpr float kCoulombV = 0.3f;
  constexpr float kCoulombRamp = 0.5f;
  float absCmd = fabsf(command.velocity);
  float mag =
      (absCmd < kCoulombRamp) ? kCoulombV * (absCmd / kCoulombRamp) : kCoulombV;
  ffVoltage += (command.velocity >= 0) ? mag : -mag;


  motor.feed_forward_voltage.q = ffVoltage;
}

void gainSchedulePID(BLDCMotor &motor, const Wheel::Command &command,
                     const Wheel::VelocityTuning &tuning) {
  constexpr float kScheduleThreshold = 0.5f; // rad/s
  constexpr float kLowSpeedP = 0.15f;         // reduced P at standstill

  float absCmd = fabsf(command.velocity);
  float blend = (absCmd < kScheduleThreshold)
                    ? absCmd / kScheduleThreshold
                    : 1.0f; // 0 = full low-speed, 1 = full normal

  motor.PID_velocity.P = kLowSpeedP + blend * (tuning.p - kLowSpeedP);
}

void Wheel::update() {
  if (_motor.motor_status == FOCMotorStatus::motor_error)
    _status = Status::MotorError;

  if (!isOk())
    return;

  gainSchedulePID(_motor, _command, _tuning);
  addFeedForwardEffects(_motor, _command);

  _motor.loopFOC();
  _motor.move(_command.velocity);

  _data.angle = _motor.shaft_angle;
  _data.velocity = _motor.shaft_velocity;
  _data.current = _motor.current_sp;
}