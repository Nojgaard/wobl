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
    float    quatXYZW[4];   // orientation quaternion (x, y, z, w) — matches scipy.spatial.transform.Rotation.from_quat format
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

// ---------------------------------------------------------------------------
// Task interface
// ---------------------------------------------------------------------------

void commTaskInit(SharedState &state);
void commTask(void *params); // SharedState*
