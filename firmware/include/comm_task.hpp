#pragma once

#include "shared_state.hpp"

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
    float    quatWXYZ[4];   // orientation quaternion (w, x, y, z)
    float    gyr[3];        // body-frame angular velocity rad/s
    float    leftAngle;     // rad
    float    leftVel;       // rad/s
    float    rightAngle;    // rad
    float    rightVel;      // rad/s
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

// CALIB_WRITE (0x07) / CALIB_DATA (0x0A) — shared layout
struct WireCalibPayload {
    uint8_t target;
    float   gyroOffset[3];  // rad/s
    float   accelOffset[3]; // m/s²
} __attribute__((packed));

// CALIB_ACK (0x08)
struct WireCalibAck {
    uint8_t success;
} __attribute__((packed));

// ---------------------------------------------------------------------------
// Task interface
// ---------------------------------------------------------------------------
void commTaskInit(SharedState &state);
void commTask(void *sharedState);
