#pragma once

#include <SimpleFOC.h>
#include <cstdint>

class Wheel {
public:
    enum class Id : uint8_t {
        Left = 1,
        Right = 2,
    };

    struct Calibration {
        float zero_electric_angle; // rad
        Direction sensor_direction = Direction::UNKNOWN;
    };

    struct VelocityTuning {
        float p = 0.1f;
        float i = 0.5f;
        float d = 0.0f;
        float output_ramp = 100.0f;  // V/s max change rate of PID output
        float lpf_velocity_tf = 0.01f;
        float velocity_limit = 40.0f; // rad/s
        float voltage_limit = 5.0f;   // V
    };

    enum class Status : uint8_t {
        Uninitialized = 0,
        Operational = 1,
        ReadyForCalibration = 2,
        MotorError = 3,
    };

    struct Config {
        Id id;

        // pinout
        int pinA;
        int pinB;
        int pinC;
        int pinEnable;

        // motor parameters
        int polePairs;
        float phaseResistance;
        float kvRating;

        float ktRating() const { return 30.0f / (PI * kvRating); }

        // calibration parameters
        float zero_electric_angle; // rad
        Direction sensor_direction = Direction::UNKNOWN;
    };

    struct Data {
        float angle; // rad
        float velocity; // rad/s
        float current;
    };

    struct Command {
        bool enabled;
        float velocity; // rad/s
    };

    Wheel(Config config);
    int init(float voltage_supply, float voltage_limit, TwoWire &wire);
    void update();

    const Data &data() const;
    Calibration calibration() const;
    VelocityTuning tuning() const;
    void command(bool enabled, float velocity);
    void tune(const VelocityTuning &tuning, bool persist = false);
    bool calibrate();
    bool isOk() const;
    Status status() const;

private:
    bool loadCalibrationFromNvs();
    bool saveCalibrationToNvs();
    bool loadTuningFromNvs();
    bool saveTuningToNvs();

    Status _status = Status::Uninitialized;
    Data _data;
    Command _command;
    Id _id;
    VelocityTuning _tuning{}; // reference tuning for gain scheduling

    MagneticSensorI2C _sensor;
    BLDCDriver3PWM _driver;
    BLDCMotor _motor;
};