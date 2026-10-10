#pragma once

namespace PitchOffset {

// fitted from data/pitch_offset_1.rdd
constexpr float HEIGHT_TO_PITCH_COEFFS[4] = {-193.81821387, 75.18364797,
                                             -10.34721967, 0.52621788};

float fromHeight(float height) {
  float pitch = 0.0f;
  float heightPower = 1.0f;

  for (int i = 0; i < 4; ++i) {
    pitch += HEIGHT_TO_PITCH_COEFFS[3 - i] * heightPower;
    heightPower *= height;
  }

  return pitch;
}

} // namespace PitchOffset