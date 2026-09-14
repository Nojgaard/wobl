#include <cmath>
#include <control/leg_kinematics.hpp>

/*
 * Based on woblpy/control/leg_kinematics.py
 * Legs of wobl can be represented as a 4 link closed kinematic loop with an
 * extended link (de) that is parallel with (cd). The system is actuated by a
 * servo, working as a crank at vertex a.
 *
 * This class converts height (distance of hip joint a to wheel axle e)
 * to crank angle (angle at a) and vice versa.
 *
 * b    a
 * \----\
 *  \    \
 *   \    \
 *    ---------- e
 *    c    d
 */

constexpr float AB = 0.047200373356574205f;
constexpr float BC = 0.09775963712084859f;
constexpr float CD = 0.024811088992625855f;
constexpr float AD = 0.09479946921792336f;
constexpr float DE = 0.09530221411908539;

constexpr float ANGLE_OFFSET = 1.74833122;

float triEdgeLength(float a, float b, float theta) {
  return sqrtf(a * a + b * b - 2 * a * b * cosf(theta));
}

float triAngle(float a, float b, float c) {
  return acosf((a * a + b * b - c * c) / (2 * a * b));
}

float LegKinematics::toHeight(float angle) {
  angle = fmin(ANGLE_MAX, fmaxf(ANGLE_MIN, angle));
  float theta_a = ANGLE_OFFSET - angle;

  float bd = triEdgeLength(AB, AD, theta_a);
  float theta_b = triAngle(bd, AD, AB);
  float theta_c = triAngle(bd, CD, BC);
  float d = M_PI - theta_b - theta_c;
  return triEdgeLength(AD, DE, d);
}

float LegKinematics::toAngle(float height) {
  height = fmin(HEIGHT_MAX, fmaxf(HEIGHT_MIN, height));

  float theta_f = triAngle(AD, DE, height);
  float ca = triEdgeLength(CD, AD, M_PI - theta_f);
  float theta_c = triAngle(ca, AB, BC);
  float theta_d = triAngle(ca, AD, CD);

  return ANGLE_OFFSET - theta_c - theta_d;
}