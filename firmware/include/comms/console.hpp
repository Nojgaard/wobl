#pragma once
#include "robot.hpp"
#include <SimpleFOC.h>

class Console {
public:
  Console(Robot *robot);
  void init();
  void update();

private:
  static void _cmdStatus(char *arg);
  static void _cmdControllerConfig(char *arg);
  static void _cmdWheelConfig(char *arg);
  static void _cmdEnable(char *arg);
  static void _cmdWheel(char *arg);
  static void _cmdCalibrate(char *arg);
  static void _cmdImu(char *arg);

  static Robot *_robot;
  static Commander _commander;
};