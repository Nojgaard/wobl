#pragma once

class KalmanFilter {
public:
    KalmanFilter(float processNoise, float measurementNoise);

    float update(float measurement, float dt = 1.0f);
    float value() const;
    float covariance() const;
    void reset(float initialState = 0.0f, float initialCovariance = 1.0f);

private:
    float _q;  // process noise variance
    float _r;  // measurement noise variance
    float _x;  // state estimate
    float _p;  // error covariance
};