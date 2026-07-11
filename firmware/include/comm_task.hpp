#pragma once

#include "robot.hpp"

#include <stdint.h>

// ---------------------------------------------------------------------------
// Wire message type IDs
// ---------------------------------------------------------------------------
enum MsgType : uint8_t {
    MSG_CMD_DRIVE       = 0x01,
    MSG_TELEM_DRIVE     = 0x02,
    MSG_CMD_POSE        = 0x03,
    MSG_TELEM_POSE      = 0x04,
    MSG_STATUS_REQ      = 0x05,
    MSG_STATUS          = 0x06,
    MSG_CALIB_WRITE     = 0x07,
    MSG_CALIB_ACK       = 0x08,
    MSG_CALIB_READ_REQ  = 0x09,
    MSG_CALIB_DATA      = 0x0A,

    MSG_WHEEL_CALIB_CMD      = 0x0B,
    MSG_WHEEL_CALIB_READ_REQ = 0x0C,
    MSG_WHEEL_CALIB_DATA     = 0x0D,
    MSG_WHEEL_TUNING_WRITE   = 0x0E,
    MSG_WHEEL_TUNING_READ_REQ= 0x0F,
    MSG_WHEEL_TUNING_DATA    = 0x10,
};

enum WheelTarget : uint8_t {
    WHEEL_TARGET_LEFT  = 1,
    WHEEL_TARGET_RIGHT = 2,
};

// ---------------------------------------------------------------------------
// Wire structs — separate from domain types, __attribute__((packed)) ensures
// identical binary layout on ESP32 and host regardless of compiler padding.
// Use fixed-width types only: uint8_t for bools, int32_t for ints, float for
// floats. All fields are in SI units (rad, rad/s, %, ms).
// ---------------------------------------------------------------------------

struct WireWheelCmd {
    uint8_t enabled;
    float   velocity;   // rad/s
} __attribute__((packed));

struct WireServoCmd {
    uint8_t enabled;
    float   positionRad; // rad
} __attribute__((packed));

// CMD_DRIVE (0x01) — host → firmware
struct WireDriveCmd {
    WireWheelCmd leftWheel;
    WireWheelCmd rightWheel;
} __attribute__((packed));

// TELEM_DRIVE (0x02) — firmware → host
struct WireDriveTelem {
    float    quatXYZW[4];   // orientation quaternion (x, y, z, w) — matches scipy.spatial.transform.Rotation.from_quat format
    float    gyr[3];        // body-frame angular velocity rad/s
    float    leftAngle;     // rad
    float    leftVel;       // rad/s
    float    leftCurrent;   // A (current setpoint)
    float    rightAngle;    // rad
    float    rightVel;      // rad/s
    float    rightCurrent;  // A (current setpoint)
    uint32_t timestampMs;
} __attribute__((packed));

// CMD_POSE (0x03) — host → firmware
struct WirePoseCmd {
    WireServoCmd leftServo;
    WireServoCmd rightServo;
} __attribute__((packed));

// TELEM_POSE (0x04) — firmware → host
struct WirePoseTelem {
    uint8_t leftValid;
    uint8_t rightValid;
    float   leftPos;    // rad
    float   leftVel;    // rad/s
    float   leftEffort; // %
    float   rightPos;   // rad
    float   rightVel;   // rad/s
    float   rightEffort;// %
} __attribute__((packed));

// STATUS (0x06) — firmware → host (response to STATUS_REQ)
struct WireStatus {
    int32_t imuStatus;
    float   imuUpdateRate;      // Hz
    float   wheelsFocRate;      // Hz
    float   wheelsUpdateRate;   // Hz
    int32_t wheelsLeftStatus;
    int32_t wheelsRightStatus;
    uint8_t servoLeftOk;
    uint8_t servoRightOk;
} __attribute__((packed));

// CALIB_DATA (0x0A) — firmware → host (response to CALIB_READ_REQ)
// CALIB_WRITE (0x07) body is a single uint8_t target — no struct needed.
struct WireCalibPayload {
    uint8_t target;
    int32_t gyroOffset[3];  // raw LSB (dps2000 scale)
    int32_t accelOffset[3]; // raw LSB (gpm4 scale)
    int32_t magOffset[3];   // raw LSB (compass)
} __attribute__((packed));

// CALIB_ACK (0x08)
struct WireCalibAck {
    uint8_t success;
} __attribute__((packed));

// WHEEL_CALIB_CMD (0x0B) — host → firmware
struct WireWheelCalibCmd {
    uint8_t target;   // WheelTarget
} __attribute__((packed));

// WHEEL_CALIB_DATA (0x0D) — firmware → host
struct WireWheelCalibData {
    uint8_t target;       // WheelTarget
    float   zeroElectricAngle;
    int32_t sensorDirection;
} __attribute__((packed));

// WHEEL_TUNING_WRITE (0x0E) — host → firmware
struct WireWheelTuning {
    uint8_t target;   // WheelTarget
    float   p;
    float   i;
    float   d;
    float   outputRamp;
    float   lpfVelocityTf;
    float   velocityLimit;
    float   voltageLimit;
    uint8_t persist;  // 1: save to NVS
} __attribute__((packed));

// WHEEL_TUNING_DATA (0x10) — firmware → host
struct WireWheelTuningData {
    uint8_t target;   // WheelTarget
    float   p;
    float   i;
    float   d;
    float   outputRamp;
    float   lpfVelocityTf;
    float   velocityLimit;
    float   voltageLimit;
} __attribute__((packed));

// ---------------------------------------------------------------------------
// Task interface
// ---------------------------------------------------------------------------

void commTaskInit(Robot &robot);
void commTask(void *params); // Robot*
