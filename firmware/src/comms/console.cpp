#include "comms/console.hpp"
#include <Arduino.h>

bool parse(const char *args, float &value) {
  return sscanf(args, "%f", &value) == 1;
}

bool parse(const char *args, bool &value) {
  int ival = 0;
  bool success = sscanf(args, "%d", &ival) == 1;
  value = success ? static_cast<bool>(ival) : value;
  return success;
}

void printValue(float value) { Serial.println(value, 2); }
void printValue(bool value) { Serial.println(value ? "true" : "false"); }
void printValue(Direction value) {
  switch (value) {
  case Direction::CW:
    Serial.println("CW");
    break;
  case Direction::CCW:
    Serial.println("CCW");
    break;
  default:
    Serial.println("UNKNOWN");
    break;
  }
}

template <typename Context> struct CommandEntry {
  char key;
  const char *name;
  void (*callback)(const char *, const char *, Context &);
  void (*print)(const char *, const Context &) = nullptr;

  template <auto Context::*Member>
  static constexpr CommandEntry assign(char key, const char *name) {
    return {key, name,
            [](const char *name, const char *args, Context &context) {
              if (parse(args, context.*Member)) {
                Serial.printf("%s set to ", name);
                printValue(context.*Member);
              } else {
                Serial.printf("Could not parse %s from '%s'\n", name, args);
              }
            },
            [](const char *name, const Context &context) {
              Serial.printf("%s: ", name);
              printValue(context.*Member);
            }};
  }

  // Read-only entry
  template <auto Context::*Member>
  static constexpr CommandEntry field(char key, const char *name) {
    return {key, name, nullptr,
            [](const char *name, const Context &context) {
              Serial.printf("%s: ", name);
              printValue(context.*Member);
            }};
  }

  template <void (*handler)(const char *, Context &)>
  static constexpr CommandEntry route(char key, const char *name) {
    return {key, name, [](const char *, const char *args, Context &context) {
              handler(args, context);
            }};
  }
};

struct CommandTable {

  template <typename Context, size_t N>
  static const CommandEntry<Context> *
  find(const CommandEntry<Context> (&entries)[N], char key) {
    for (size_t i = 0; i < N; ++i) {
      if (entries[i].key == key) {
        return &entries[i];
      }
    }
    return nullptr;
  }

  template <typename Context, size_t N>
  static void print(const CommandEntry<Context> (&entries)[N],
                    const Context &context) {
    for (const auto &entry : entries) {
      if (entry.print) {
        entry.print(entry.name, context);
      }
    }
  }

  template <typename Context, size_t N>
  static void dispatch(const CommandEntry<Context> (&entries)[N],
                       const char *args, Context &context) {
    const char key = args[0];

    if (key == '\0') {
      print(entries, context);
      return;
    }

    if (key == '?') {
      Serial.println("Available commands:");
      for (size_t i = 0; i < N; ++i) {
        Serial.printf("  %c: %s\n", entries[i].key, entries[i].name);
      }
      return;
    }

    const CommandEntry<Context> *entry = find(entries, key);
    if (entry) {
      if (entry->callback) {
        entry->callback(entry->name, args + 1, context);
      } else if (entry->print) {
        entry->print(entry->name, context);
      }
      return;
    }

    Serial.printf("Unknown command: %c\n", key);
  }
};

Console::Console(Robot &robot, Broadcaster &broadcaster, Pilot &pilot,
                 Monitor &monitor)
    : robot(robot), broadcaster(broadcaster), pilot(pilot), monitor(monitor) {}

void Console::init() {}

void Console::update() {
  while (Serial.available()) {
    const char c = Serial.read();
    if (c == ' ' || c == '\r' || c == '\t')
      continue;

    _received_chars[_received_count++] = c;

    if (c == '\n') {
      _received_chars[_received_count - 1] = '\0';
      dispatch(_received_chars);

      memset(_received_chars, 0, MAX_CMD_SIZE);
      _received_count = 0;
    }

    if (_received_count >= MAX_CMD_SIZE) {
      Serial.println("Error: input line too long");
      memset(_received_chars, 0, MAX_CMD_SIZE);
      _received_count = 0;
    }
  }
}

/*
 * Status Commands
 */

void cmdStatus(const char *, Console &console) {
  Robot &r = console.robot;
  Pilot &p = console.pilot;

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

/* Controller Commands */

void cmdControllerPoseGains(const char *args, MotionController &controller) {
  using Gains = MotionController::PoseGains;
  using Entry = CommandEntry<Gains>;

  static constexpr Entry commands[] = {
      Entry::assign<&Gains::roll>('p', "roll"),
      Entry::assign<&Gains::rollRate>('d', "rollRate"),
  };

  auto cfg = controller.config();
  CommandTable::dispatch(commands, args, cfg.poseGains);
  controller.config(cfg);
}

void cmdControllerBalanceGains(const char *args, MotionController &controller) {
  using Gains = MotionController::BalanceGains;
  using Entry = CommandEntry<Gains>;

  static constexpr Entry commands[] = {
      Entry::assign<&Gains::pitch>('p', "pitch"),
      Entry::assign<&Gains::pitchRate>('r', "pitchRate"),
      Entry::assign<&Gains::velocity>('v', "velocity"),
      Entry::assign<&Gains::position>('x', "position"),
  };

  auto cfg = controller.config();
  CommandTable::dispatch(commands, args, cfg.balanceGains);
  controller.config(cfg);
}

void cmdControllerCommand(const char *args, MotionController &controller) {
  using Command = MotionController::Command;
  using Entry = CommandEntry<Command>;

  static constexpr Entry commands[] = {
      Entry::assign<&Command::enable>('e', "enable"),
  };

  auto cmd = controller.command();
  CommandTable::dispatch(commands, args, cmd);
  controller.command(cmd);
}

void cmdController(const char *args, Console &console) {
  using Entry = CommandEntry<MotionController>;

  static constexpr CommandEntry<MotionController> commands[] = {
      Entry::route<cmdControllerCommand>('c', "controller command"),
      Entry::route<cmdControllerBalanceGains>('b', "balance gains"),
      Entry::route<cmdControllerPoseGains>('p', "pose gains"),
  };

  CommandTable::dispatch(commands, args, console.robot.controller);
};

/*
 * Servo Commands
 */

void cmdServo(const char *args, Console &console) {
  auto calibrate = [](const char *, ServoSubsystem &servos) {
    servos.calibrate();
  };

  using Entry = CommandEntry<ServoSubsystem>;
  static constexpr CommandEntry<ServoSubsystem> commands[] = {
      Entry::route<calibrate>('c', "calibrate")};

  CommandTable::dispatch(commands, args, console.robot.servos);
}

/*
 * Wheel Commands
 */

void cmdWheelTune(const char *args, WheelSubsystem &wheels) {
  using Tuning = Wheel::VelocityTuning;
  using Entry = CommandEntry<Tuning>;

  static constexpr CommandEntry<Tuning> commands[] = {
      Entry::assign<&Tuning::lpf_velocity_tf>('f', "lpf_vel"),
  };

  auto t = wheels.tuning(Wheel::Id::Left);
  CommandTable::dispatch(commands, args, t);
  wheels.tune(t, false);
}

void cmdWheelCalibrate(const char *args, WheelSubsystem &wheels) {
  auto calibrate = [](const char *, WheelSubsystem &wheels) {
    wheels.calibrate();
  };

  using F = CommandEntry<Wheel::Calibration>;
  static constexpr F fields[] = {
      F::field<&Wheel::Calibration::zero_electric_angle>('a', "zero angle"),
      F::field<&Wheel::Calibration::sensor_direction>('d', "direction"),
  };

  using Entry = CommandEntry<WheelSubsystem>;
  static constexpr Entry commands[] = {Entry::route<calibrate>('r', "run")};

  if (args[0] == '\0') {
    Serial.println("Left:");
    auto cl = wheels.calibration(Wheel::Id::Left);
    CommandTable::print(fields, cl);

    Serial.println("Right:");
    auto cr = wheels.calibration(Wheel::Id::Right);
    CommandTable::print(fields, cr);
  } else {
    CommandTable::dispatch(commands, args, wheels);
  }
}

void cmdWheel(const char *args, Console &console) {
  using Entry = CommandEntry<WheelSubsystem>;

  static constexpr CommandEntry<WheelSubsystem> commands[] = {
      Entry::route<cmdWheelTune>('t', "tune"),
      Entry::route<cmdWheelCalibrate>('c', "calibrate")};

  CommandTable::dispatch(commands, args, console.robot.wheels);
}

/*
 * Pilot Commands
 */

void cmdPilot(const char *args, Console &console) {
  using Entry = CommandEntry<Pilot>;

  static constexpr Entry commands[] = {
      {'d', "disconnect",
       [](auto, auto, auto &pilot) {
         Serial.println("Disconnecting gamepad...");
         pilot.disconnect();
       }},
      {'s', "scan [0|1]",
       [](auto, auto args, auto &pilot) {
         const bool enable = args[0] == '1';
         pilot.scanForDevices(enable);
         Serial.printf("Scan %s\n", enable ? "ENABLED" : "DISABLED");
       }},
      {'f', "forget",
       [](auto, auto, auto &pilot) {
         Serial.println("Forgetting paired devices...");
         pilot.forgetDevices();
       }},
  };

  CommandTable::dispatch(commands, args, console.pilot);
}

/*
 * Broadcast Commands
 */

void cmdBroadcast(const char *args, Console &console) {
  using Entry = CommandEntry<Broadcaster>;

  static constexpr Entry commands[] = {
      {'e', "enable [0|1]",
       [](auto, auto args, auto &broadcaster) {
         const bool enable = args[0] == '1';
         broadcaster.enable(enable);
         Serial.printf("Broadcast %s\n", enable ? "ENABLED" : "DISABLED");
       }},
  };

  CommandTable::dispatch(commands, args, console.broadcaster);
}

/*
 * Monitor Commands
 */
void cmdMonitor(const char *args, Console &console) {
  using Entry = CommandEntry<Monitor>;

  static constexpr Entry commands[] = {
      {'s', "servo",
       [](const char *name, const char *, Monitor &monitor) {
         monitor.mode = Monitor::Display::SERVO;
         Serial.printf("Monitoring %s\n", name);
       }},
      {'r', "robot",
       [](const char *name, const char *, Monitor &monitor) {
         monitor.mode = Monitor::Display::ROBOT;
         Serial.printf("Monitoring %s\n", name);
       }},
      {'n', "disabled",
       [](const char *name, const char *, Monitor &monitor) {
         monitor.mode = Monitor::Display::NONE;
         Serial.printf("Monitoring %s\n", name);
       }},
  };

  CommandTable::dispatch(commands, args, console.monitor);
}

/*
 * Console command dispatcher
 */
void Console::dispatch(const char *args) {
  using Entry = CommandEntry<Console>;

  static constexpr CommandEntry<Console> commands[] = {
      Entry::route<cmdStatus>('s', "status"),
      Entry::route<cmdController>('c', "controller"),
      Entry::route<cmdServo>('r', "servo"),
      Entry::route<cmdWheel>('w', "wheel"),
      Entry::route<cmdPilot>('p', "pilot"),
      Entry::route<cmdBroadcast>('b', "broadcast"),
      Entry::route<cmdMonitor>('m', "monitor"),
  };

  CommandTable::dispatch(commands, args, *this);
}