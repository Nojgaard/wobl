#include "control/linear_filter.hpp"

LinearFilter::LinearFilter(float smoothingFactor, float initialValue)
    : _smoothingFactor(smoothingFactor)
    , _value(initialValue) {
}

void LinearFilter::update(float newValue) {
    _value = (1.0f - _smoothingFactor) * newValue + _smoothingFactor * _value;
}

float LinearFilter::value() const {
    return _value;
}
