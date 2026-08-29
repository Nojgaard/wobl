#pragma once
#include "drivers/servo.hpp"
#include "protected.hpp"

class ServoSubsystem {
public:
  enum class Request { None, Calibrate };

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
  void calibrate();

  Status status();
  void command(const Command &cmd);
  Telemetry telemetry();

private:
  void handleRequests();
  void syncCommand();
  void syncTelemetry();

  Protected<Request> _request;
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