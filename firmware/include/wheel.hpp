#pragma once

#include <SimpleFOC.h>

class Wheel {
public:
    struct Calibration {
        bool calibrated;
        float zero_electric_angle; // rad
        Direction sensor_direction;
    };

    struct Config {
        int polePairs;
        int pinA;
        int pinB;
        int pinC;
        int pinEnable;
        Calibration calibration;
    };

    struct Data {
        float angle; // rad
        float velocity; // rad/s
    };

    struct Command {
        bool enabled;
        float velocity; // rad/s
    };

    Wheel(Config config);
    int init(float voltage_supply, float voltage_limit, TwoWire &wire);
    void update();

    const Data &data();
    void command(bool enabled, float velocity);
    bool isOk() const;

private:
    bool _initOk = false;
    Data _data;
    Command _command;
    Calibration _calibration;

    MagneticSensorI2C _sensor;
    BLDCDriver3PWM _driver;
    BLDCMotor _motor;
};