#pragma once

class LinearFilter {
public:
    LinearFilter(float smoothingFactor, float initialValue);

    void update(float newValue);
    float value() const;

private:
    float _smoothingFactor;  // weight given to new samples (0..1)
    float _value;            // current filtered value
};
