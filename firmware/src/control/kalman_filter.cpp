#include "control/kalman_filter.hpp"

KalmanFilter::KalmanFilter(float processNoise, float measurementNoise)
    : _q(processNoise)
    , _r(measurementNoise)
    , _x(0.0f)
    , _p(1.0f) {
}

float KalmanFilter::update(float measurement, float dt) {
    // Predict (random-walk model)
    _p = _p + _q * dt;

    // Kalman gain
    float k = _p / (_p + _r);

    // State update
    _x = _x + k * (measurement - _x);

    // Error covariance update
    _p = (1.0f - k) * _p;

    return _x;
}

float KalmanFilter::value() const {
    return _x;
}

float KalmanFilter::covariance() const {
    return _p;
}

void KalmanFilter::reset(float initialState, float initialCovariance) {
    _x = initialState;
    _p = initialCovariance;
}