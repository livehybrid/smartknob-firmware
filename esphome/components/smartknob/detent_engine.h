#pragma once

// Haptic detent engine: turns a knob angle into a restoring torque.
//
// Port of the detent algorithm from Scott Bezek's SmartKnob firmware
// (https://github.com/scottbez1/smartknob, Apache-2.0), with these changes:
//  * one consistent sign convention: angle, position and torque all increase
//    in the same direction (the original's idle correction mixed frames when
//    rotation was inverted);
//  * rate-independent maths (dt passed in), so the control loop can run faster
//    than the original ~1 kHz without changing the feel;
//  * the derivative term is low-pass filtered so faster loops do not amplify
//    sensor noise into audible buzz.
//
// Deliberately free of ESP-IDF and ESPHome dependencies so it can be unit
// tested on a PC (see esphome/tests).

#include <cstdint>

namespace esphome::smartknob {

static constexpr uint8_t MAX_DETENT_POSITIONS = 8;

struct HapticProfile {
  // Positions allowed; max_position < min_position means unbounded.
  int32_t min_position{0};
  int32_t max_position{-1};
  // Angle of one position step, radians.
  float position_width{0.17453293f};  // 10 degrees
  // Detent (restoring) strength; 0 = free spinning. Roughly 0..3.
  float detent_strength{0.0f};
  // Strength of the virtual end stops at min/max.
  float endstop_strength{1.0f};
  // Fraction of a position width to turn before snapping to the next one.
  // Must be >= 0.5; > 1 gives a "sticky" feel with no snap-back.
  float snap_point{1.1f};
  // Makes it easier to move towards position 0 than away from it (return-to-centre feel).
  float snap_point_bias{0.0f};
  // If non-empty, detent torque only applies at these positions ("magnetic" detents).
  uint8_t detent_positions_count{0};
  int32_t detent_positions[MAX_DETENT_POSITIONS]{};

  bool bounded() const { return this->max_position >= this->min_position; }
  // Returns nullptr if valid, else a reason.
  const char *validate() const;
};

struct DetentOutput {
  float torque;        // requested torque (volts), positive towards increasing position
  bool out_of_bounds;  // pushing against an end stop
  bool changed;        // position changed during this update
};

class DetentEngine {
 public:
  DetentEngine();

  // Applies a new profile. The logical position is kept (clamped to the new bounds)
  // unless set_position is true, in which case it becomes `position`.
  void set_profile(const HapticProfile &profile, float angle, bool set_position, int32_t position,
                   float sub_position = 0.0f);
  // Sets the logical position without moving the knob (re-centres the detent here).
  void set_position(int32_t position, float angle, float sub_position = 0.0f);
  // Shifts the internal angle reference, e.g. after the caller re-normalises its
  // multi-turn angle, so behaviour is unchanged.
  void shift_reference(float delta) { this->detent_center_ += delta; }

  // One control step.
  //   angle:    knob angle in radians (multi-turn), increasing = towards higher positions
  //   velocity: rad/s, same sign convention
  //   dt:       seconds since the previous update
  //   now_ms:   monotonic milliseconds
  DetentOutput update(float angle, float velocity, float dt, uint32_t now_ms);

  int32_t position() const { return this->position_; }
  // Fraction of a position the knob is turned away from the detent centre,
  // positive towards higher positions. Usually within +/- snap_point.
  float sub_position() const { return this->sub_position_; }
  const HapticProfile &profile() const { return this->profile_; }
  float derivative_gain() const { return this->d_gain_; }

  // Tuning constants (public so tests and the component can adjust them).
  float strength_to_p_gain{4.0f};         // volts per radian per unit strength (as original)
  float output_limit{10.0f};              // volts, before the driver's own limit
  float velocity_cutoff{60.0f};           // rad/s: no torque above this (prevents runaway)
  float derivative_cutoff_hz{160.0f};     // low-pass on the derivative term
  float dead_zone_fraction{0.2f};         // of a position width...
  float dead_zone_max{0.01745329f};       // ...but at most 1 degree
  float idle_velocity{0.05f};             // rad/s below which the knob counts as idle
  float idle_velocity_tau{1.0f};          // seconds, idle-velocity averaging
  uint32_t idle_delay_ms{500};            // idle this long before re-centring starts
  float idle_max_angle{0.08726646f};      // only re-centre within 5 degrees
  float idle_recentre_tau{2.0f};          // seconds, re-centring time constant

 protected:
  void update_derivative_gain_();
  bool in_detent_positions_(int32_t position) const;

  HapticProfile profile_{};
  int32_t position_{0};
  float detent_center_{0.0f};
  float sub_position_{0.0f};
  float d_gain_{0.0f};
  float prev_input_{0.0f};
  float filtered_derivative_{0.0f};
  bool have_prev_{false};
  float idle_velocity_ewma_{0.0f};
  uint32_t idle_start_ms_{0};
  bool idle_{false};
};

}  // namespace esphome::smartknob
