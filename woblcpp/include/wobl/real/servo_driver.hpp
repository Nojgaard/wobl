#pragma once

#include <cstdint>
#include <scservo/SCServo.h>
#include <string>

namespace wobl::real
{
    class ServoDriver
    {
    public:
        struct ServoState
        {
            bool success;
            double position_rad;
            double velocity_rps;
            double effort_pct;
            double voltage_volts;
            double current_amps;
            double temperature_celcius;
            int is_move;
        };
        enum ServoMode
        {
            POSITION = 0,
            VELOCITY = 1
        };

        ServoDriver(std::string port = "/dev/ttyAMA1") { port_ = port; }
        bool initialize();
        ~ServoDriver();
        bool ping(u8 servo_id);
        bool set_mode(u8 servo_id, ServoMode servo_type);
        bool write_position(u8 servo_id, double position_rad, double velocity_rps, double acceleration_rps2 = 0);
        bool write_velocity(u8 servo_id, double velocity_rps, double acceleration_rps2 = 0);
        bool set_midpoint(u8 servo_id);
        bool enable_torque(u8 servo_id, bool is_on);
        bool set_id(u8 servo_id, u8 new_servo_id);
        ServoState read_state(u8 servo);

    private:
        double steps_to_radians(int steps) const;
        double effort_to_nm(int effort) const;
        double temperature_to_celsius(int temp) const;
        double voltage_to_volts(int voltage) const;
        double current_to_amps(int current) const;

        int radians_to_steps(double radians) const;

        int baudrate_ = 1000000; // 1M baudrate
        std::string port_;
        bool is_initialized_ = false;

        const int STEPS_PER_REVOLUTION_ = 4096;
        const int MAX_SPEED_ = 6800;
        const int MAX_ACCELERATION_ = 254;
        const double KT_3215_KG_CM_PER_A = 11;

        SMS_STS servo_;
    };
} // namespace wobl::real