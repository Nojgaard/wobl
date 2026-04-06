#include <algorithm>
#include <iostream>
#include <math.h>
#include <wobl/real/servo_driver.hpp>

using namespace wobl::real;

bool ServoDriver::initialize()
{
    is_initialized_ = servo_.begin(baudrate_, port_.c_str());
    return is_initialized_;
}

ServoDriver::~ServoDriver()
{
    if (is_initialized_)
    {
        servo_.end();
        is_initialized_ = false;
    }
}

bool ServoDriver::ping(u8 servo_id) { return servo_.Ping(servo_id) != -1; }

bool ServoDriver::set_mode(u8 servo_id, ServoDriver::ServoMode mode) { return servo_.Mode(servo_id, mode); }

bool ServoDriver::write_position(u8 servo_id, double position_rad, double velocity_rps, double acceleration_rps2)
{

    u16 position_steps =
        std::clamp(radians_to_steps(position_rad) + STEPS_PER_REVOLUTION_ / 2, 0, STEPS_PER_REVOLUTION_ - 1);
    u16 velocity_steps = std::clamp(radians_to_steps(velocity_rps), -MAX_SPEED_, MAX_SPEED_);
    u8 acceleration_steps = std::clamp(radians_to_steps(acceleration_rps2), 0, MAX_ACCELERATION_);
    return servo_.WritePosEx(servo_id, position_steps, velocity_steps, acceleration_steps);
}

bool ServoDriver::write_velocity(u8 servo_id, double velocity_rps, double acceleration_rps2)
{
    u16 velocity_steps = std::clamp(radians_to_steps(velocity_rps), -MAX_SPEED_, MAX_SPEED_);
    u8 acceleration_steps = std::clamp(radians_to_steps(acceleration_rps2), 0, MAX_ACCELERATION_);
    return servo_.WriteSpe(servo_id, velocity_steps, acceleration_steps);
}

bool ServoDriver::set_midpoint(u8 servo_id) { return servo_.CalibrationOfs(servo_id); }

bool ServoDriver::enable_torque(u8 servo_id, bool is_on) { return servo_.EnableTorque(servo_id, is_on); }

bool ServoDriver::set_id(u8 servo_id, u8 new_servo_id)
{
    if (!servo_.unLockEprom(servo_id))
        return false;
    if (!servo_.writeByte(servo_id, SMSBL_ID, new_servo_id))
    {
        servo_.LockEprom(servo_id);
        return false;
    }
    return servo_.LockEprom(new_servo_id);
}

ServoDriver::ServoState ServoDriver::read_state(u8 servo_id)
{
    ServoDriver::ServoState state;
    state.success = servo_.FeedBack(servo_id) != -1;
    if (!state.success)
        return state;
    state.position_rad = steps_to_radians(servo_.ReadPos(-1) - STEPS_PER_REVOLUTION_ / 2);
    state.velocity_rps = steps_to_radians(servo_.ReadSpeed(-1));
    state.effort_pct = effort_to_nm(servo_.ReadLoad(-1));

    state.voltage_volts = voltage_to_volts(servo_.ReadVoltage(-1));
    state.current_amps = current_to_amps(servo_.ReadCurrent(-1));
    state.temperature_celcius = temperature_to_celsius(servo_.ReadTemper(-1));
    state.is_move = servo_.ReadMove(-1);
    return state;
}

// Conversions here: https://www.feetechrc.com/en/2020-05-13_56655.html
// and here:
// https://view.officeapps.live.com/op/view.aspx?src=https%3A%2F%2Ffiles.waveshare.com%2Fupload%2F2%2F27%2FST3215%2520memory%2520register%2520map-EN.xls&wdOrigin=BROWSELINK
double ServoDriver::steps_to_radians(int steps) const { return (steps * 2.0 * M_PI) / STEPS_PER_REVOLUTION_; }

double ServoDriver::effort_to_nm(int effort) const { return static_cast<double>(effort) * 0.1; }

double ServoDriver::temperature_to_celsius(int temp) const { return static_cast<double>(temp); }

double ServoDriver::voltage_to_volts(int voltage) const { return static_cast<double>(voltage) * 0.1; }

double ServoDriver::current_to_amps(int current) const { return current * 6.5 / 1000; }

int ServoDriver::radians_to_steps(double radians) const
{
    return static_cast<int>(radians * STEPS_PER_REVOLUTION_ / (2 * M_PI));
}
