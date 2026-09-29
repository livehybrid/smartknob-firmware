#include "detent_engine.h"

#include <cmath>

namespace esphome::smartknob {

namespace {

constexpr float PI_F = 3.14159265358979f;

template<typename T> T clampv(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }

float deg_to_rad(float deg) { return deg * PI_F / 180.0f; }

}  // namespace

const char *HapticProfile::validate() const {
  if (!(this->position_width > 0.0f))
    return "position_width must be greater than 0";
  if (this->detent_strength < 0.0f)
    return "detent_strength cannot be negative";
  if (this->endstop_strength < 0.0f)
    return "endstop_strength cannot be negative";
  if (this->snap_point < 0.5f)
    return "snap_point must be >= 0.5 for stability";
  if (this->snap_point_bias < 0.0f)
    return "snap_point_bias cannot be negative";
  if (this->detent_positions_count > MAX_DETENT_POSITIONS)
    return "too many detent_positions";
  return nullptr;
}

DetentEngine::DetentEngine() { this->update_derivative_gain_(); }

void DetentEngine::set_profile(const HapticProfile &profile, float angle, bool set_position, int32_t position,
                               float sub_position) {
  const float old_width = this->profile_.position_width;
  const float old_sub = this->sub_position_;
  this->profile_ = profile;

  int32_t new_position = set_position ? position : this->position_;
  if (profile.bounded())
    new_position = clampv(new_position, profile.min_position, profile.max_position);

  if (set_position || new_position != this->position_ || profile.position_width != old_width) {
    // Re-centre so the knob does not jump: the current angle keeps its
    // fractional offset (or the requested one) within the new detent.
    const float sub = set_position ? sub_position : old_sub;
    this->detent_center_ = angle - sub * profile.position_width;
    this->sub_position_ = sub;
  }
  this->position_ = new_position;
  this->update_derivative_gain_();
  this->have_prev_ = false;
  this->filtered_derivative_ = 0.0f;
}

void DetentEngine::set_position(int32_t position, float angle, float sub_position) {
  if (this->profile_.bounded())
    position = clampv(position, this->profile_.min_position, this->profile_.max_position);
  this->position_ = position;
  this->detent_center_ = angle - sub_position * this->profile_.position_width;
  this->sub_position_ = sub_position;
  this->have_prev_ = false;
  this->filtered_derivative_ = 0.0f;
}

void DetentEngine::update_derivative_gain_() {
  // Piecewise-linear derivative gain against detent width, as in the original
  // firmware: fine detents need more D to feel like distinct clicks, coarse
  // detents need less or the motor amplifies sensor noise.
  const float lower = this->profile_.detent_strength * 0.08f;
  const float upper = this->profile_.detent_strength * 0.02f;
  const float width_lower = deg_to_rad(3.0f);
  const float width_upper = deg_to_rad(8.0f);
  const float raw =
      lower + (upper - lower) / (width_upper - width_lower) * (this->profile_.position_width - width_lower);
  // Intermittent ("magnetic") detents feel wrong with D: it adds clicks between them.
  this->d_gain_ = this->profile_.detent_positions_count > 0
                      ? 0.0f
                      : clampv(raw, std::fmin(lower, upper), std::fmax(lower, upper));
}

bool DetentEngine::in_detent_positions_(int32_t position) const {
  for (uint8_t i = 0; i < this->profile_.detent_positions_count; i++) {
    if (this->profile_.detent_positions[i] == position)
      return true;
  }
  return false;
}

DetentOutput DetentEngine::update(float angle, float velocity, float dt, uint32_t now_ms) {
  DetentOutput out{0.0f, false, false};
  const HapticProfile &p = this->profile_;
  const float width = p.position_width;
  if (!(dt > 0.0f) || dt > 0.1f)
    dt = 0.001f;

  // Idle re-centring: if the knob rests slightly off-centre for a while, move
  // the centre towards it so the motor stops pushing against the user.
  const float idle_alpha = std::fmin(1.0f, dt / this->idle_velocity_tau);
  this->idle_velocity_ewma_ += (velocity - this->idle_velocity_ewma_) * idle_alpha;
  if (std::fabs(this->idle_velocity_ewma_) > this->idle_velocity) {
    this->idle_ = false;
  } else if (!this->idle_) {
    this->idle_ = true;
    this->idle_start_ms_ = now_ms;
  }
  float angle_to_center = angle - this->detent_center_;
  if (this->idle_ && (now_ms - this->idle_start_ms_) > this->idle_delay_ms &&
      std::fabs(angle_to_center) < this->idle_max_angle) {
    const float k = std::fmin(1.0f, dt / this->idle_recentre_tau);
    this->detent_center_ += (angle - this->detent_center_) * k;
    angle_to_center = angle - this->detent_center_;
  }

  // Snap to the neighbouring position once past the snap point. A small loop
  // catches up if a very fine profile is spun faster than one step per update.
  const bool bounded = p.bounded();
  const float snap = width * p.snap_point;
  const float bias = width * p.snap_point_bias;
  for (int i = 0; i < 4; i++) {
    const float inc_threshold = snap + (this->position_ >= 0 ? bias : -bias);
    const float dec_threshold = snap + (this->position_ <= 0 ? bias : -bias);
    if (angle_to_center > inc_threshold && (!bounded || this->position_ < p.max_position)) {
      this->position_++;
      this->detent_center_ += width;
      angle_to_center -= width;
      out.changed = true;
    } else if (angle_to_center < -dec_threshold && (!bounded || this->position_ > p.min_position)) {
      this->position_--;
      this->detent_center_ -= width;
      angle_to_center += width;
      out.changed = true;
    } else {
      break;
    }
  }
  this->sub_position_ = angle_to_center / width;

  // A small dead zone around the centre stops the motor hunting.
  const float dead_zone_limit = std::fmin(width * this->dead_zone_fraction, this->dead_zone_max);
  const float dead_zone_adjustment = clampv(angle_to_center, -dead_zone_limit, dead_zone_limit);

  const bool out_of_bounds = bounded && ((angle_to_center < 0.0f && this->position_ == p.min_position) ||
                                         (angle_to_center > 0.0f && this->position_ == p.max_position));
  out.out_of_bounds = out_of_bounds;
  const float p_gain = (out_of_bounds ? p.endstop_strength : p.detent_strength) * this->strength_to_p_gain;

  float input = -angle_to_center + dead_zone_adjustment;
  if (!out_of_bounds && p.detent_positions_count > 0 && !this->in_detent_positions_(this->position_))
    input = 0.0f;

  // Derivative of the error, low-pass filtered. The step in `input` when the
  // position snaps is deliberately kept: it produces the "click" on fine
  // detents. Filtering spreads it over ~1 ms so its impulse (and feel) does not
  // depend on the loop rate.
  float derivative = 0.0f;
  if (this->have_prev_) {
    const float raw = (input - this->prev_input_) / dt;
    const float alpha = 1.0f - std::exp(-2.0f * PI_F * this->derivative_cutoff_hz * dt);
    this->filtered_derivative_ += (raw - this->filtered_derivative_) * alpha;
    derivative = this->filtered_derivative_;
  }
  this->prev_input_ = input;
  this->have_prev_ = true;

  float torque = p_gain * input + this->d_gain_ * derivative;
  torque = clampv(torque, -this->output_limit, this->output_limit);
  // No torque when spinning fast: avoids a positive-feedback runaway.
  if (std::fabs(velocity) > this->velocity_cutoff)
    torque = 0.0f;
  out.torque = torque;
  return out;
}

}  // namespace esphome::smartknob
