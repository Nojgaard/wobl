#pragma once

namespace LegKinematics {

float toAngle(float height);
float toHeight(float angle);

constexpr float NOMINAL_HEIGHT = 0.143;

constexpr float HEIGHT_MIN = 0.06;
constexpr float HEIGHT_MAX = 0.177;

constexpr float ANGLE_MIN = -0.2;
constexpr float ANGLE_MAX = 0.65;

}; // namespace LegKinematics