#pragma once

struct BodyVelocity {
    float forwardVelocity;  // m/s
    float yawRate;          // rad/s
};

struct WheelVelocity {
    float leftRps;
    float rightRps;
};

namespace DiffDriveKinematics {
    BodyVelocity toBodyVel(float leftWheelRps, float rightWheelRps);
    WheelVelocity toWheelVel(float forwardVelocity, float yawRate);
}
