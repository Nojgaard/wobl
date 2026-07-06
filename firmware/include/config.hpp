#pragma once

#include "drivers/wheel.hpp"

static Wheel::Config leftWheelConfig{
    .id = Wheel::Id::Left,
    .pinA = 12,
    .pinB = 14,
    .pinC = 27,
    .pinEnable = 13,
    .polePairs = 11,
    .phaseResistance = 5.7f / 2.0f,
    .kvRating = 170.0f,
    .zero_electric_angle = 0.81,
    .sensor_direction = Direction::CCW,
};

static Wheel::Config rightWheelConfig{
    .id = Wheel::Id::Right,
    .pinA = 4,
    .pinB = 33,
    .pinC = 32,
    .pinEnable = 2,
    .polePairs = 11,
    .phaseResistance = 5.7f / 2.0f,
    .kvRating = 170.0f,
    .zero_electric_angle = 6.08,
    .sensor_direction = Direction::CW,
};
