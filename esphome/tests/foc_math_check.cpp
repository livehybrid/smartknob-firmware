// Compiles foc_math.h with the strict firmware warning flags (see Makefile).
#include "smartknob/foc_math.h"

namespace esphome::smartknob::foc {
// Instantiate the inline helpers so every code path is type-checked.
float foc_math_check(float a, float uq) {
  PhaseDuty d = svpwm(uq, 0.0f, normalize_angle(a), 5.0f);
  MT6701Frame f = mt6701_decode(1, 2, 3);
  uint8_t out[3];
  mt6701_encode(f.raw_angle, 0, out);
  return d.a + d.b + d.c + wrap_pi(a) + f.angle() + static_cast<float>(out[0]);
}
}  // namespace esphome::smartknob::foc
