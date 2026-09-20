#pragma once

namespace PitchOffset {

constexpr float HEIGHT_TO_PITCH_COEFFS[4] = {-180.146716, 69.166527, -9.579059,
                                             0.535259};

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