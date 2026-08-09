#include "comms/console.hpp"
#include <Arduino.h>

static void _skipSpace(char **p) {
  while (**p == ' ' || **p == '\n' || **p == '\r')
    (*p)++;
}

// -------------------------------------------------------------------
// Static member definitions
// -------------------------------------------------------------------

Robot *Console::_robot = nullptr;
Broadcaster *Console::_broadcaster = nullptr;
Pilot *Console::_pilot = nullptr;
Commander Console::_commander(Serial);

// -------------------------------------------------------------------
// Public API
// -------------------------------------------------------------------

Console::Console(Robot *robot, Broadcaster *broadcaster, Pilot *pilot) {
  _robot = robot;
  _broadcaster = broadcaster;
  _pilot = pilot;
}

void Console::init() {
  _commander.add('s', _cmdStatus, "status");
  _commander.add('o', _cmdObserverConfig, "config observer [r|p][=val]");
  _commander.add('g', _cmdControllerConfig, "config control [p|r|v|x|o][=val]");
  _commander.add('t', _cmdWheelConfig, "config wheel [p|i|d|f][=val]");
  _commander.add('e', _cmdEnable, "enable [0|1]");
  _commander.add('w', _cmdWheel, "drive [vel]");
  _commander.add('c', _cmdCalibrate, "calibrate [i|w]");
  _commander.add('i', _cmdImu, "imu telemetry");
  _commander.add('b', _cmdEnableTelemetry, "enable telemetry [0|1]");
  _commander.add('p', _cmdPilot, "pilot [d|s|f]");

  Serial.println("Console ready. Type '?' for commands.");
}

void Console::update() { _commander.run(); }

// ===================================================================
// Command: s — Status
// ===================================================================

void Console::_cmdStatus(char *arg) {
  _skipSpace(&arg);
  auto &r = *_robot;
  auto &p = *_pilot;

  auto is = r.imu.status();
  auto ws = r.wheels.status();
  auto ss = r.servos.status();
  auto cs = r.controller.status();
  auto ps = p.status(); 

  Serial.printf("IMU   status=%i sync=%.0fHz\n", is.status, is.syncRateHz);
  Serial.printf("WHEEL status=[L=%i R=%i] sync=%.0fHz update=%.0fHz\n", ws.left,
                ws.right, ws.syncRateHz, ws.updateRateHz);
  Serial.printf("SERVO status=[L=%i R=%i] cmdSync=%.0fHz telSync=%.0fHz\n",
                ss.left, ss.right, ss.cmdSyncRateHz, ss.telSyncRateHz);
  Serial.printf("CTRL  sync=%.0fHz\n", cs.syncRateHz);
  Serial.printf("Pilot sync=%.0fHz\n", ps.syncRate);
  Serial.printf("BATT  volt=%.2fV\n", ss.voltage);
}

// ===================================================================
// Command: g — ControllerConfig
// ===================================================================

void Console::_cmdControllerConfig(char *arg) {
  _skipSpace(&arg);
  auto cfg = _robot->controller.config();

  if (*arg == '\0') {
    Serial.printf("pitchKp=%.3f  pitchRateKp=%.3f  "
                  "velKp=%.3f  posKp=%.3f  offset=%.3f\n",
                  cfg.pitchKp, cfg.pitchRateKp, cfg.velocityKp, cfg.positionKp,
                  cfg.pitchOffset);
    return;
  }

  float v;
  bool set = false;

  if (sscanf(arg, "p=%f", &v) == 1) {
    Serial.printf("pitchKp:     %.3f -> %.3f\n", cfg.pitchKp, v);
    cfg.pitchKp = v;
  } else if (sscanf(arg, "r=%f", &v) == 1) {
    Serial.printf("pitchRateKp: %.3f -> %.3f\n", cfg.pitchRateKp, v);
    cfg.pitchRateKp = v;
  } else if (sscanf(arg, "v=%f", &v) == 1) {
    Serial.printf("velKp:       %.3f -> %.3f\n", cfg.velocityKp, v);
    cfg.velocityKp = v;
  } else if (sscanf(arg, "x=%f", &v) == 1) {
    Serial.printf("posKp:       %.3f -> %.3f\n", cfg.positionKp, v);
    cfg.positionKp = v;
  } else if (sscanf(arg, "o=%f", &v) == 1) {
    Serial.printf("offset:      %.3f -> %.3f\n", cfg.pitchOffset, v);
    cfg.pitchOffset = v;
  } else if (sscanf(arg, "c=%f", &v) == 1) {
    Serial.printf("ctrlScale:   %.3f -> %.3f\n", cfg.ctrlScale, v);
    cfg.ctrlScale = v;
  } else {
    Serial.printf("Unknown: '%s'. Try p=val r=val v=val x=val o=val\n", arg);
    return;
  }

  _robot->controller.config(cfg);
}

// ===================================================================
// Command: t — WheelConfig
// ===================================================================

void Console::_cmdWheelConfig(char *arg) {
  _skipSpace(&arg);

  // No arg → print both wheels
  if (*arg == '\0') {
    auto tL = _robot->wheels.tuning(Wheel::Id::Left);
    auto tR = _robot->wheels.tuning(Wheel::Id::Right);
    Serial.printf("Left:  p=%.4f  i=%.4f  d=%.4f  f=%.4f\n", tL.p, tL.i, tL.d,
                  tL.lpf_velocity_tf);
    Serial.printf("Right: p=%.4f  i=%.4f  d=%.4f  f=%.4f\n", tR.p, tR.i, tR.d,
                  tR.lpf_velocity_tf);
    return;
  }

  // Param
  auto t = _robot->wheels.tuning(Wheel::Id::Left);
  float v;
  bool persist = false;

  if (sscanf(arg, "p=%f", &v) == 1) {
    Serial.printf("P: %.4f -> %.4f\n", t.p, v);
    t.p = v;
  } else if (sscanf(arg, "i=%f", &v) == 1) {
    Serial.printf("I: %.4f -> %.4f\n", t.i, v);
    t.i = v;
  } else if (sscanf(arg, "d=%f", &v) == 1) {
    Serial.printf("D: %.4f -> %.4f\n", t.d, v);
    t.d = v;
  } else if (sscanf(arg, "f=%f", &v) == 1) {
    Serial.printf("LPF: %.4f -> %.4f\n", t.lpf_velocity_tf, v);
    t.lpf_velocity_tf = v;
  } else if (arg[0] == 's') {
    persist = true;
    Serial.println("Persisting tuning to NVS");
  } else {
    Serial.printf("Unknown: '%s'. Try p=val i=val d=val f=val\n", arg);
    return;
  }

  _robot->wheels.tune(t, persist);
}

// ===================================================================
// Command: e — Enable
// ===================================================================

void Console::_cmdEnable(char *arg) {
  _skipSpace(&arg);

  if (*arg == '\0') {
    auto cmd = _robot->controller.command();
    Serial.printf("Controller %s\n", cmd.enable ? "ENABLED" : "DISABLED");
    return;
  }

  int enable;
  if (sscanf(arg, "%d", &enable) == 1) {
    _robot->controller.command({static_cast<bool>(enable), 0.0f, 0.0f});
    Serial.printf("Controller %s\n", enable ? "ENABLED" : "DISABLED");
  } else {
    Serial.printf("Expected 0 or 1, got '%s'\n", arg);
  }
}

// ===================================================================
// Command: w — Wheel passthrough
// ===================================================================

void Console::_cmdWheel(char *arg) {
  _skipSpace(&arg);

  // Safety: passthrough only when controller is off
  if (_robot->controller.command().enable) {
    Serial.println(
        "Controller enabled - refusing passthrough. Disable with 'e0' first.");
    return;
  }

  if (*arg == '\0') {
    auto tel = _robot->wheels.telemetry();
    Serial.printf("Left:  %.2f rad/s\n", tel.left.velocity);
    Serial.printf("Right: %.2f rad/s\n", tel.right.velocity);
    return;
  }

  float vel;
  WheelSubsystem::Command cmd{};

  if (sscanf(arg, "%f", &vel) == 1) {
    cmd.left = {true, vel};
    cmd.right = {true, vel};
    _robot->wheels.command(cmd);
    Serial.printf("Drive: %.2f rad/s\n", vel);
  } else {
    Serial.printf("Unknown: '%s'. Try '5.0' or '3.0'\n", arg);
  }
}

// ===================================================================
// Command: c — Calibrate
// ===================================================================

void Console::_cmdCalibrate(char *arg) {
  _skipSpace(&arg);

  if (*arg == '\0') {
    auto imuCal = _robot->imu.calibration();
    auto wCalL = _robot->wheels.calibration(Wheel::Id::Left);
    auto wCalR = _robot->wheels.calibration(Wheel::Id::Right);
    Serial.printf(
        "IMU   gyro=(%ld,%ld,%ld) accel=(%ld,%ld,%ld) mag=(%ld,%ld,%ld)\n",
        (long)imuCal.gyro[0], (long)imuCal.gyro[1], (long)imuCal.gyro[2],
        (long)imuCal.accel[0], (long)imuCal.accel[1], (long)imuCal.accel[2],
        (long)imuCal.mag[0], (long)imuCal.mag[1], (long)imuCal.mag[2]);
    Serial.printf("WHEEL L angle=%.3f dir=%d  R angle=%.3f dir=%d\n",
                  wCalL.zero_electric_angle, (int)wCalL.sensor_direction,
                  wCalR.zero_electric_angle, (int)wCalR.sensor_direction);
    return;
  }

  if (strcmp(arg, "i") == 0) {
    Serial.println("Calibrating IMU...");
    _robot->imu.calibrate();
    Serial.println("IMU calibration done.");
  } else if (strcmp(arg, "w") == 0) {
    Serial.println("Calibrating wheels...");
    _robot->wheels.calibrate();
    Serial.println("Wheel calibration done.");
  } else {
    Serial.printf("Unknown: '%s'. Try: i w\n", arg);
  }
}

// ===================================================================
// Command: i — Imu
// ===================================================================

void Console::_cmdImu(char *arg) {
  auto imu = _robot->imu.telemetry();
  Serial.printf("r=%.3f p=%.3f y=%.3f\n", imu.roll, imu.pitch, imu.yaw);
}

void Console::_cmdEnableTelemetry(char *arg) {
  _skipSpace(&arg);

  int enable;
  if (sscanf(arg, "%d", &enable) == 1) {
    _broadcaster->enable(static_cast<bool>(enable));
    Serial.printf("Telemetry %s\n", enable ? "ENABLED" : "DISABLED");
  } else {
    Serial.printf("Expected 0 or 1, got '%s'\n", arg);
  }
}

void Console::_cmdObserverConfig(char *arg) {
  _skipSpace(&arg);
  auto cfg = _robot->controller.observer.config();

  if (*arg == '\0') {
    Serial.printf("tcRates=%.3f tcVelocities=%.3f\n", cfg.tcRates,
                  cfg.tcVelocities);
    return;
  }

  float v;
  if (sscanf(arg, "r=%f", &v) == 1) {
    Serial.printf("tcRates: %.3f -> %.3f\n", cfg.tcRates, v);
    cfg.tcRates = v;
  } else if (sscanf(arg, "v=%f", &v) == 1) {
    Serial.printf("tcVelocities:   %.3f -> %.3f\n", cfg.tcVelocities, v);
    cfg.tcVelocities = v;
  } else {
    Serial.printf("Unknown: '%s'. Try r=val v=val\n", arg);
    return;
  }

  _robot->controller.observer.config(cfg);
}

void Console::_cmdPilot(char *arg) {
  _skipSpace(&arg);

  if (*arg == '\0') {
    Serial.println("Usage: pilot d=disconnect, s[0|1]=scan, f=forget");
    return;
  }

  int value;
  if (strcmp(arg, "d\n") == 0) {
    Serial.println("Disconnecting gamepad...");
    _pilot->disconnect();
  } else if (sscanf(arg, "s%d", &value) == 1) {
    _pilot->scanForDevices(value != 0);
    Serial.printf("Scan %s\n", value ? "ENABLED" : "DISABLED");
  } else if (strcmp(arg, "f\n") == 0) {
    Serial.println("Forgetting paired devices...");
    _pilot->forgetDevices();
  } else {
    Serial.printf("Unknown: '%s'. Try: d s0 s1 f\n", arg);
  }
}