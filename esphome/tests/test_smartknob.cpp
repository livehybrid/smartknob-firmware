// Host-side tests for the SmartKnob haptics engine and FOC maths.
//
//   make -C esphome/tests        (or: g++ -std=c++17 -O2 ... see Makefile)
//
// Covers the hardware-independent code shared with the firmware:
// detent_engine.{h,cpp} and foc_math.h. Includes a closed-loop simulation of
// the knob (rotor inertia, motor torque constant, sensor quantisation and
// noise) to check stability and feel at different control-loop rates.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>

#include "../components/smartknob/detent_engine.h"
#include "../components/smartknob/foc_math.h"

using namespace esphome::smartknob;

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond, ...)                                          \
  do {                                                            \
    g_checks++;                                                   \
    if (!(cond)) {                                                \
      g_failures++;                                               \
      std::printf("  FAIL %s:%d: %s -- ", __FILE__, __LINE__, #cond); \
      std::printf(__VA_ARGS__);                                   \
      std::printf("\n");                                          \
    }                                                             \
  } while (0)

static constexpr float DEG = foc::PI_F / 180.0f;

static HapticProfile make_profile(int32_t min_pos, int32_t max_pos, float width_deg, float detent, float endstop,
                                  float snap, float bias = 0.0f) {
  HapticProfile p;
  p.min_position = min_pos;
  p.max_position = max_pos;
  p.position_width = width_deg * DEG;
  p.detent_strength = detent;
  p.endstop_strength = endstop;
  p.snap_point = snap;
  p.snap_point_bias = bias;
  return p;
}

// ---------------------------------------------------------------------------
static void test_angles() {
  std::printf("angles\n");
  CHECK(std::fabs(foc::normalize_angle(-0.1f) - (foc::TWO_PI_F - 0.1f)) < 1e-5f, "normalize negative");
  CHECK(std::fabs(foc::normalize_angle(7.0f) - (7.0f - foc::TWO_PI_F)) < 1e-5f, "normalize > 2pi");
  CHECK(std::fabs(foc::wrap_pi(3.5f) - (3.5f - foc::TWO_PI_F)) < 1e-5f, "wrap_pi");
  CHECK(std::fabs(foc::wrap_pi(-3.5f) - (-3.5f + foc::TWO_PI_F)) < 1e-5f, "wrap_pi negative");
}

static void test_mt6701() {
  std::printf("mt6701 frames\n");
  int bad = 0;
  for (uint32_t raw = 0; raw < 16384; raw += 97) {
    for (uint8_t status = 0; status < 16; status++) {
      uint8_t b[3];
      foc::mt6701_encode(raw, status, b);
      auto f = foc::mt6701_decode(b[0], b[1], b[2]);
      if (!f.crc_ok || f.raw_angle != raw || f.field_status != (status & 3) || f.push != ((status >> 2) & 1) ||
          f.loss != ((status >> 3) & 1))
        bad++;
      // Any single-bit error in the payload must be caught by the CRC.
      for (int bit = 6; bit < 24; bit++) {
        uint32_t w = (b[0] << 16) | (b[1] << 8) | b[2];
        w ^= (1u << bit);
        auto e = foc::mt6701_decode((w >> 16) & 0xFF, (w >> 8) & 0xFF, w & 0xFF);
        if (e.crc_ok)
          bad++;
      }
    }
  }
  CHECK(bad == 0, "%d bad frames", bad);
  auto f = foc::mt6701_decode(0, 0, 0);
  CHECK(f.crc_ok && f.raw_angle == 0, "all-zero frame is valid");
}

static void test_svpwm() {
  std::printf("svpwm\n");
  const float vs = 5.0f;
  float worst = 0;
  for (int i = 0; i < 360; i += 5) {
    float th = i * DEG;
    for (float uq : {-2.5f, -1.0f, 0.0f, 0.7f, 2.8f}) {
      auto d = foc::svpwm(uq, 0.0f, th, vs);
      CHECK(d.a >= 0 && d.a <= 1 && d.b >= 0 && d.b <= 1 && d.c >= 0 && d.c <= 1, "duty in range");
      // Reconstruct alpha/beta from line voltages and compare with the request.
      float ua = d.a * vs, ub = d.b * vs, uc = d.c * vs;
      float alpha = (2 * ua - ub - uc) / 3.0f;
      float beta = (ub - uc) / std::sqrt(3.0f);
      float ex_alpha = -uq * std::sin(th);
      float ex_beta = uq * std::cos(th);
      worst = std::fmax(worst, std::fabs(alpha - ex_alpha) + std::fabs(beta - ex_beta));
    }
  }
  CHECK(worst < 1e-4f, "svpwm vector error %.6f V", worst);
  // Beyond the linear range the vector is scaled down, direction kept.
  auto d = foc::svpwm(10.0f, 0.0f, 0.3f, vs);
  float ua = d.a * vs, ub = d.b * vs, uc = d.c * vs;
  float alpha = (2 * ua - ub - uc) / 3.0f, beta = (ub - uc) / std::sqrt(3.0f);
  float mag = std::sqrt(alpha * alpha + beta * beta);
  CHECK(std::fabs(mag - vs / std::sqrt(3.0f)) < 1e-3f, "saturates at Vs/sqrt3 (%.3f)", mag);
  CHECK(std::fabs(std::atan2(beta, alpha) - (0.3f + foc::PI_F / 2)) < 1e-3f, "direction kept when saturating");
}

// ---------------------------------------------------------------------------
static void test_detent_basics() {
  std::printf("detent engine basics\n");
  DetentEngine e;
  auto p = make_profile(0, 10, 10.0f, 1.0f, 1.0f, 1.1f);
  CHECK(p.validate() == nullptr, "profile valid");
  e.set_profile(p, 0.0f, true, 5);
  CHECK(e.position() == 5, "initial position");

  // Small positive offset -> restoring (negative) torque.
  auto o = e.update(3 * DEG, 0, 0.0002f, 0);
  CHECK(o.torque < 0, "restoring torque sign (%.3f)", o.torque);
  // Inside the dead zone (1 deg) -> ~no proportional torque.
  DetentEngine e2;
  e2.set_profile(p, 0.0f, true, 5);
  o = e2.update(0.5f * DEG, 0, 0.0002f, 0);
  CHECK(std::fabs(o.torque) < 0.05f, "dead zone (%.3f)", o.torque);

  // Snap forward after snap_point (1.1 widths).
  DetentEngine e3;
  e3.set_profile(p, 0.0f, true, 5);
  float a = 0;
  uint32_t t = 0;
  for (; a < 10.5f * DEG; a += 0.05f * DEG, t++)
    e3.update(a, 0.0f, 0.0002f, t);
  CHECK(e3.position() == 5, "no snap before 1.1 widths (pos %d)", e3.position());
  for (; a < 11.5f * DEG; a += 0.05f * DEG, t++)
    e3.update(a, 0.0f, 0.0002f, t);
  CHECK(e3.position() == 6, "snapped to 6 (pos %d)", e3.position());
  // Position 6's centre is one width (10 deg) above position 5's centre (0 deg).
  CHECK(std::fabs(e3.sub_position() - (a - 10 * DEG) / (10 * DEG)) < 0.02f, "sub position after snap (%.3f)",
        static_cast<double>(e3.sub_position()));

  // With snap_point 1.1 there is no snap-back when returning past the old
  // centre: going back to 5 needs 1.1 widths below the new centre (-1 deg).
  for (; a > 0.0f; a -= 0.05f * DEG, t++)
    e3.update(a, 0.0f, 0.0002f, t);
  CHECK(e3.position() == 6, "hysteresis keeps 6 at 0 deg (pos %d)", e3.position());
  for (; a > -1.5f * DEG; a -= 0.05f * DEG, t++)
    e3.update(a, 0.0f, 0.0002f, t);
  CHECK(e3.position() == 5, "snaps back to 5 below -1 deg (pos %d)", e3.position());
}

static void test_endstops() {
  std::printf("end stops\n");
  DetentEngine e;
  auto p = make_profile(0, 3, 10.0f, 0.5f, 2.0f, 1.1f);
  e.set_profile(p, 0.0f, true, 3);
  auto o = e.update(25 * DEG, 0, 0.0002f, 0);
  CHECK(e.position() == 3, "cannot pass max (pos %d)", e.position());
  CHECK(o.out_of_bounds, "out of bounds flagged");
  CHECK(o.torque < -1.0f, "strong end stop torque (%.2f)", o.torque);
  // Set position outside the range is clamped.
  e.set_position(99, 0.0f);
  CHECK(e.position() == 3, "set_position clamps (pos %d)", e.position());
  e.set_position(-7, 0.0f);
  CHECK(e.position() == 0, "set_position clamps low (pos %d)", e.position());
  // Unbounded profile never flags out of bounds.
  DetentEngine u;
  u.set_profile(make_profile(0, -1, 10.0f, 1.0f, 1.0f, 1.1f), 0.0f, true, 0);
  for (int i = 0; i < 2000; i++)
    o = u.update(-i * 0.2f * DEG, 0, 0.0002f, i);
  CHECK(!o.out_of_bounds && u.position() < -30, "unbounded goes negative (pos %d)", u.position());
}

static void test_bias_and_magnetic() {
  std::printf("snap bias and magnetic detents\n");
  // Return-to-centre with bias: moving away from 0 needs more rotation than returning.
  auto p = make_profile(-6, 6, 60.0f, 1.0f, 1.0f, 0.55f, 0.4f);
  DetentEngine e;
  e.set_profile(p, 0.0f, true, 0);
  float a = 0;
  int t = 0;
  while (e.position() == 0 && a < 120 * DEG) {
    a += 0.1f * DEG;
    e.update(a, 0, 0.0002f, t++);
  }
  float away = a;  // expect (0.55 + 0.4) * 60 = 57 deg
  CHECK(std::fabs(away / DEG - 57.0f) < 0.5f, "away-from-centre snap at %.1f deg", away / DEG);
  // Now at position 1 centred at 60 deg; returning needs only (0.55-0.4)*60 = 9 deg.
  float centre = 60 * DEG;
  a = centre;
  while (e.position() == 1 && a > -120 * DEG) {
    a -= 0.1f * DEG;
    e.update(a, 0, 0.0002f, t++);
  }
  CHECK(std::fabs((centre - a) / DEG - 9.0f) < 0.5f, "towards-centre snap after %.1f deg", (centre - a) / DEG);

  // Magnetic detents: no torque at positions without a detent.
  auto m = make_profile(0, 31, 7.0f, 2.5f, 1.0f, 0.7f);
  m.detent_positions_count = 2;
  m.detent_positions[0] = 2;
  m.detent_positions[1] = 10;
  DetentEngine g;
  g.set_profile(m, 0.0f, true, 5);
  auto o = g.update(3 * DEG, 0, 0.0002f, 0);
  CHECK(std::fabs(o.torque) < 1e-4f, "no torque between magnetic detents (%.4f)", o.torque);
  g.set_position(10, 0.0f);
  o = g.update(3 * DEG, 0, 0.0002f, 1);
  CHECK(o.torque < -0.1f, "torque at a magnetic detent (%.3f)", o.torque);
  CHECK(g.derivative_gain() == 0.0f, "no D term with magnetic detents");
}

static void test_velocity_cutoff_and_validation() {
  std::printf("velocity cut-off and validation\n");
  DetentEngine e;
  e.set_profile(make_profile(0, 10, 10.0f, 1.0f, 1.0f, 1.1f), 0.0f, true, 5);
  auto o = e.update(4 * DEG, 80.0f, 0.0002f, 0);
  CHECK(o.torque == 0.0f, "no torque above velocity cut-off");
  HapticProfile bad = make_profile(0, 10, 10.0f, 1.0f, 1.0f, 0.3f);
  CHECK(bad.validate() != nullptr, "snap_point < 0.5 rejected");
  bad = make_profile(0, 10, 0.0f, 1.0f, 1.0f, 1.1f);
  CHECK(bad.validate() != nullptr, "zero width rejected");
}

static void test_idle_recentre() {
  std::printf("idle re-centring (same frame as detents)\n");
  // Resting 3 degrees off-centre with a user holding it there: after the idle
  // delay the centre must move TOWARDS the resting angle, reducing torque.
  DetentEngine e;
  e.set_profile(make_profile(0, 10, 10.0f, 1.0f, 1.0f, 1.1f), 0.0f, true, 5);
  const float rest = 3 * DEG;
  float first = 0, last = 0;
  const float dt = 0.0002f;
  for (int i = 0; i < 5 * 5000; i++) {  // 5 s at 5 kHz
    auto o = e.update(rest, 0.0f, dt, static_cast<uint32_t>(i * dt * 1000));
    if (i == 0)
      first = std::fabs(o.torque);
    last = std::fabs(o.torque);
  }
  CHECK(last < first * 0.2f, "torque decays when idle off-centre (%.3f -> %.3f)", first, last);
  CHECK(e.position() == 5, "position unchanged by re-centring");
  // A negative offset must behave symmetrically.
  DetentEngine n;
  n.set_profile(make_profile(0, 10, 10.0f, 1.0f, 1.0f, 1.1f), 0.0f, true, 5);
  for (int i = 0; i < 5 * 5000; i++)
    last = std::fabs(n.update(-rest, 0.0f, dt, static_cast<uint32_t>(i * dt * 1000)).torque);
  CHECK(last < first * 0.2f, "symmetric for negative offsets (%.3f)", last);
}

// ---------------------------------------------------------------------------
// Closed-loop simulation.
//
// Plant: knob + rotor inertia with viscous and Coulomb friction, driven by the
// motor (torque proportional to Uq) and a finger. The finger is a stiff
// spring-damper pulling towards a target angle that moves at a set speed, then
// lets go. Sensor: 14-bit quantisation, Gaussian noise, the same unit-vector
// low-pass filter as the firmware.

struct Plant {
  // Rough figures for a 3215 gimbal motor with the knob fitted.
  float inertia = 2.0e-5f;        // kg m^2
  float torque_per_volt = 0.01f;  // N m per volt of Uq (Kt / R)
  float damping = 2.0e-5f;        // N m s/rad (bearings, back-EMF)
  float friction = 2.0e-4f;       // N m Coulomb friction
};

struct Finger {
  float speed = 1.5f;          // rad/s target speed (~86 deg/s, a normal turn)
  float hold_s = 0.5f;         // finger moves for this long, then lets go
  float stiffness = 0.2f;      // N m / rad
  float damping = 1.0e-3f;     // N m s / rad
};

struct SimResult {
  int32_t final_position;
  float settle_amplitude;  // peak |angle - detent centre| over the last 0.3 s
  float max_torque;        // volts
  float click_impulse;     // integral of |torque| (V s) over 30 ms around the first snap
};

static SimResult simulate(const HapticProfile &p, float loop_hz, const Finger &finger, float duration_s,
                          float sensor_noise_lsb = 1.0f, unsigned seed = 1) {
  const Plant pl;
  DetentEngine e;
  e.set_profile(p, 0.0f, true, 0);
  std::mt19937 rng(seed);
  std::normal_distribution<float> noise(0.0f, sensor_noise_lsb * foc::TWO_PI_F / 16384.0f);

  const float sim_dt = 1.0e-5f;  // 100 kHz physics
  const int steps_per_control = static_cast<int>(std::lround(1.0f / (loop_hz * sim_dt)));
  const float ctrl_dt = static_cast<float>(steps_per_control) * sim_dt;
  const float filter_alpha = 1.0f - std::exp(-2.0f * foc::PI_F * 400.0f * ctrl_dt);
  float theta = 0, omega = 0, torque_v = 0;
  float fx = 1, fy = 0, prev_meas = 0, vel_est = 0;
  bool have_prev = false;
  SimResult r{};
  int32_t last_pos = 0;
  float snap_time = -1, impulse = 0;
  const int total = static_cast<int>(duration_s / sim_dt);
  for (int i = 0; i < total; i++) {
    const float t = static_cast<float>(i) * sim_dt;
    if (i % steps_per_control == 0) {
      const float lsb = foc::TWO_PI_F / 16384.0f;
      const float meas = std::round(theta / lsb) * lsb + noise(rng);
      fx += (std::cos(meas) - fx) * filter_alpha;
      fy += (std::sin(meas) - fy) * filter_alpha;
      float filtered = std::atan2(fy, fx);
      filtered += foc::TWO_PI_F * std::round((theta - filtered) / foc::TWO_PI_F);  // unwrap
      if (have_prev) {
        const float v = (filtered - prev_meas) / ctrl_dt;
        vel_est += (v - vel_est) * (1.0f - std::exp(-ctrl_dt / 0.002f));
      }
      prev_meas = filtered;
      have_prev = true;
      const auto o = e.update(filtered, vel_est, ctrl_dt, static_cast<uint32_t>(t * 1000.0f));
      torque_v = std::fmax(-2.88f, std::fmin(2.88f, o.torque));  // Vs/sqrt(3) at 5 V
      if (e.position() != last_pos && snap_time < 0)
        snap_time = t;
      last_pos = e.position();
      r.max_torque = std::fmax(r.max_torque, std::fabs(torque_v));
    }
    if (snap_time >= 0 && t >= snap_time - 0.005f && t < snap_time + 0.025f)
      impulse += std::fabs(torque_v) * sim_dt;
    float user = 0.0f;
    if (t < finger.hold_s)
      user = finger.stiffness * (finger.speed * t - theta) - finger.damping * omega;
    float tau = torque_v * pl.torque_per_volt + user - pl.damping * omega;
    if (std::fabs(omega) > 1e-3f)
      tau -= (omega > 0 ? 1.0f : -1.0f) * pl.friction;
    else if (std::fabs(tau) < pl.friction)
      tau = 0;
    omega += tau / pl.inertia * sim_dt;
    theta += omega * sim_dt;
    if (t > duration_s - 0.3f) {
      const float centre = static_cast<float>(e.position()) * p.position_width;
      r.settle_amplitude = std::fmax(r.settle_amplitude, std::fabs(theta - centre));
    }
  }
  r.final_position = e.position();
  r.click_impulse = impulse;
  return r;
}

static void test_closed_loop() {
  std::printf("closed-loop simulation\n");
  struct Case {
    const char *name;
    HapticProfile p;
    int32_t expect_min;  // finger travels 0.75 rad (43 deg)
  };
  std::vector<Case> cases = {
      {"coarse strong (32 x 8.2 deg, strength 2)", make_profile(0, 31, 8.225f, 2.0f, 1.0f, 1.1f), 3},
      {"fine with detents (1 deg, strength 1)", make_profile(0, 255, 1.0f, 1.0f, 1.0f, 1.1f), 30},
      {"on/off (60 deg, strength 1)", make_profile(0, 1, 60.0f, 1.0f, 1.0f, 0.55f), 1},
      {"brightness (51 x 3.6 deg, strength 1.2)", make_profile(0, 50, 3.6f, 1.2f, 1.5f, 1.1f), 9},
      {"end stop at 3 (10 deg steps, 43 deg push)", make_profile(0, 3, 10.0f, 1.0f, 2.0f, 1.1f), 3},
  };
  const Finger finger;
  for (auto &c : cases) {
    for (float hz : {1000.0f, 5000.0f}) {
      const auto r = simulate(c.p, hz, finger, 1.5f);
      const bool moved = r.final_position >= c.expect_min;
      const bool stable = r.settle_amplitude < 0.5f * c.p.position_width;
      std::printf("  %-44s %5.0f Hz: pos %3d, settle %5.2f deg, max %.2f V, click %5.2f mVs\n", c.name,
                  static_cast<double>(hz), r.final_position, static_cast<double>(r.settle_amplitude / DEG),
                  static_cast<double>(r.max_torque), static_cast<double>(r.click_impulse * 1000));
      CHECK(moved, "%s @%.0f Hz: followed the finger (pos %d, want >= %d)", c.name, static_cast<double>(hz),
            r.final_position, c.expect_min);
      CHECK(stable, "%s @%.0f Hz: settles within half a detent (%.2f deg)", c.name, static_cast<double>(hz),
            static_cast<double>(r.settle_amplitude / DEG));
      CHECK(!c.p.bounded() || r.final_position <= c.p.max_position, "within bounds");
    }
  }
  // Released past the end stop, the knob must be pulled back to the last position.
  Finger hard;
  hard.speed = 3.0f;
  auto r = simulate(make_profile(0, 3, 10.0f, 1.0f, 1.0f, 1.1f), 5000.0f, hard, 1.5f);
  std::printf("  pushed 86 deg against a 30 deg range: pos %d, settle %.2f deg\n", r.final_position,
              static_cast<double>(r.settle_amplitude / DEG));
  CHECK(r.final_position == 3 && r.settle_amplitude < 5 * DEG, "end stop returns the knob");

  // At rest with sensor noise the motor must not buzz.
  Finger none;
  none.hold_s = 0.0f;
  r = simulate(make_profile(0, 255, 1.0f, 1.0f, 1.0f, 1.1f), 5000.0f, none, 1.0f, 2.0f);
  std::printf("  at rest, fine detents, 2 LSB noise: max torque %.3f V\n", static_cast<double>(r.max_torque));
  CHECK(r.max_torque < 0.3f, "quiet at rest (%.3f V)", static_cast<double>(r.max_torque));
  r = simulate(make_profile(0, 31, 8.225f, 2.0f, 1.0f, 1.1f), 5000.0f, none, 1.0f, 2.0f);
  std::printf("  at rest, coarse strong detents, 2 LSB noise: max torque %.3f V\n",
              static_cast<double>(r.max_torque));
  CHECK(r.max_torque < 0.3f, "quiet at rest, coarse (%.3f V)", static_cast<double>(r.max_torque));

  // Click impulse on fine detents should not depend much on the loop rate.
  const auto c1 = simulate(make_profile(0, 255, 1.0f, 1.0f, 1.0f, 1.1f), 1000.0f, finger, 0.6f, 0.0f);
  const auto c5 = simulate(make_profile(0, 255, 1.0f, 1.0f, 1.0f, 1.1f), 5000.0f, finger, 0.6f, 0.0f);
  const float ratio = c5.click_impulse / c1.click_impulse;
  std::printf("  click impulse 5 kHz / 1 kHz = %.2f\n", static_cast<double>(ratio));
  CHECK(ratio > 0.6f && ratio < 1.6f, "click feel independent of loop rate (ratio %.2f)",
        static_cast<double>(ratio));
}

int main() {
  test_angles();
  test_mt6701();
  test_svpwm();
  test_detent_basics();
  test_endstops();
  test_bias_and_magnetic();
  test_velocity_cutoff_and_validation();
  test_idle_recentre();
  test_closed_loop();
  std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
