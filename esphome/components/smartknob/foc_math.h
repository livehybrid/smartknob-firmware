#pragma once

// Hardware-independent maths for field-oriented control of the SmartKnob's
// gimbal motor, plus MT6701 frame decoding. Header-only so the firmware and
// the host unit tests (esphome/tests) share exactly the same code.

#include <cmath>
#include <cstdint>

namespace esphome::smartknob::foc {

static constexpr float PI_F = 3.14159265358979f;
static constexpr float TWO_PI_F = 6.28318530717959f;
static constexpr float SQRT3_2 = 0.86602540378444f;
static constexpr float INV_SQRT3 = 0.57735026918963f;

// Angle in [0, 2*pi).
inline float normalize_angle(float a) {
  float r = std::fmod(a, TWO_PI_F);
  return r < 0.0f ? r + TWO_PI_F : r;
}

// Angle in (-pi, pi].
inline float wrap_pi(float a) {
  float r = normalize_angle(a + PI_F) - PI_F;
  return r <= -PI_F ? r + TWO_PI_F : r;
}

struct PhaseDuty {
  float a, b, c;  // 0..1
};

// Space-vector PWM (min/max zero-sequence injection).
//   uq, ud:  voltages on the rotor q and d axes
//   theta_e: electrical angle of the rotor d axis
// Voltages beyond the linear range (v_supply / sqrt(3)) are scaled down
// keeping direction, so torque saturates smoothly rather than distorting.
inline PhaseDuty svpwm(float uq, float ud, float theta_e, float v_supply) {
  const float max_amplitude = v_supply * INV_SQRT3;
  float magnitude = std::sqrt(uq * uq + ud * ud);
  if (magnitude > max_amplitude && magnitude > 0.0f) {
    const float k = max_amplitude / magnitude;
    uq *= k;
    ud *= k;
  }
  const float s = std::sin(theta_e);
  const float c = std::cos(theta_e);
  // Inverse Park
  const float u_alpha = ud * c - uq * s;
  const float u_beta = ud * s + uq * c;
  // Inverse Clarke
  const float ua = u_alpha;
  const float ub = -0.5f * u_alpha + SQRT3_2 * u_beta;
  const float uc = -0.5f * u_alpha - SQRT3_2 * u_beta;
  // Centre the three phases in the available range.
  const float vmax = std::fmax(ua, std::fmax(ub, uc));
  const float vmin = std::fmin(ua, std::fmin(ub, uc));
  const float offset = 0.5f * v_supply - 0.5f * (vmax + vmin);
  auto duty = [&](float u) {
    float d = (u + offset) / v_supply;
    return d < 0.0f ? 0.0f : (d > 1.0f ? 1.0f : d);
  };
  return PhaseDuty{duty(ua), duty(ub), duty(uc)};
}

// ---- MT6701 SSI frame -----------------------------------------------------
// 24-bit frame: [23:10] angle (14 bit), [9:6] status, [5:0] CRC6 (x^6 + x + 1)
// over the upper 18 bits. Table and bit layout as in the original firmware.

inline uint8_t mt6701_crc6(uint32_t data18) {
  static const uint8_t TABLE[64] = {
      0x00, 0x03, 0x06, 0x05, 0x0C, 0x0F, 0x0A, 0x09, 0x18, 0x1B, 0x1E, 0x1D, 0x14, 0x17, 0x12, 0x11,
      0x30, 0x33, 0x36, 0x35, 0x3C, 0x3F, 0x3A, 0x39, 0x28, 0x2B, 0x2E, 0x2D, 0x24, 0x27, 0x22, 0x21,
      0x23, 0x20, 0x25, 0x26, 0x2F, 0x2C, 0x29, 0x2A, 0x3B, 0x38, 0x3D, 0x3E, 0x37, 0x34, 0x31, 0x32,
      0x13, 0x10, 0x15, 0x16, 0x1F, 0x1C, 0x19, 0x1A, 0x0B, 0x08, 0x0D, 0x0E, 0x07, 0x04, 0x01, 0x02};
  uint8_t index = (data18 >> 12) & 0x3F;
  uint8_t crc = (data18 >> 6) & 0x3F;
  index = crc ^ TABLE[index];
  crc = data18 & 0x3F;
  index = crc ^ TABLE[index];
  return TABLE[index];
}

struct MT6701Frame {
  bool crc_ok;
  uint16_t raw_angle;    // 0..16383
  uint8_t field_status;  // 0 normal, 1 too strong, 2 too weak
  bool push;             // push-button detection bit
  bool loss;             // loss of track
  float angle() const { return static_cast<float>(this->raw_angle) * (TWO_PI_F / 16384.0f); }
};

inline MT6701Frame mt6701_decode(uint8_t b0, uint8_t b1, uint8_t b2) {
  const uint32_t word = (static_cast<uint32_t>(b0) << 16) | (static_cast<uint32_t>(b1) << 8) | b2;
  MT6701Frame f{};
  f.raw_angle = static_cast<uint16_t>(word >> 10);
  f.field_status = (word >> 6) & 0x3;
  f.push = ((word >> 8) & 0x1) != 0;
  f.loss = ((word >> 9) & 0x1) != 0;
  f.crc_ok = mt6701_crc6(word >> 6) == (word & 0x3F);
  return f;
}

// Builds a valid frame (used by tests and the calibration simulator).
inline void mt6701_encode(uint16_t raw_angle, uint8_t status4, uint8_t out[3]) {
  const uint32_t data18 = (static_cast<uint32_t>(raw_angle & 0x3FFF) << 4) | (status4 & 0xF);
  const uint32_t word = (data18 << 6) | mt6701_crc6(data18);
  out[0] = (word >> 16) & 0xFF;
  out[1] = (word >> 8) & 0xFF;
  out[2] = word & 0xFF;
}

}  // namespace esphome::smartknob::foc
