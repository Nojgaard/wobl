#pragma once
#include "drivers/servo.hpp"
#include "protected.hpp"

class ServoSubsystem {
public:
  struct Status {
    float cmdSyncRateHz;
    float telSyncRateHz;
    byte left;
    byte right;
    float voltage;
  };

  struct Command {
    Servo::Command left;
    Servo::Command right;
  };

  struct Telemetry {
    Servo::Data left;
    Servo::Data right;
  };

  ServoSubsystem();
  void init();
  void loop();

  Status status();
  void command(const Command &cmd);
  Telemetry telemetry();

private:
  void syncCommand();
  void syncTelemetry();

  Protected<Status> _status;
  Protected<Command> _command;
  Protected<Telemetry> _telemetry;

  HardwareSerial _servoSerial;
  SMS_STS _bus;

  Servo _leftHip;
  Servo _rightHip;

  long _lastCommandTime;
  long _lastFeedbackTime;
};