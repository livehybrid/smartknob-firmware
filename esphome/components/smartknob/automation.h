#pragma once

#include <cstdint>
#include <initializer_list>

#include "esphome/core/automation.h"
#include "esphome/core/helpers.h"

#include "smartknob.h"

namespace esphome::smartknob {

// Widths are given in degrees in YAML and lambdas; the engine works in radians.
static constexpr float DEGREES_TO_RADIANS = 0.017453292519943295f;

template<typename... Ts> class SetProfileAction : public Action<Ts...>, public Parented<SmartKnob> {
 public:
  TEMPLATABLE_VALUE(int32_t, min_position)
  TEMPLATABLE_VALUE(int32_t, max_position)
  TEMPLATABLE_VALUE(float, position_width)
  TEMPLATABLE_VALUE(float, detent_strength)
  TEMPLATABLE_VALUE(float, endstop_strength)
  TEMPLATABLE_VALUE(float, snap_point)
  TEMPLATABLE_VALUE(float, snap_point_bias)
  TEMPLATABLE_VALUE(int32_t, position)

  void set_detent_positions(std::initializer_list<int32_t> positions) {
    this->detent_positions_count_ = 0;
    for (int32_t p : positions) {
      if (this->detent_positions_count_ < MAX_DETENT_POSITIONS)
        this->detent_positions_[this->detent_positions_count_++] = p;
    }
  }

  void play(const Ts &...x) override {
    HapticProfile profile;
    profile.min_position = this->min_position_.value(x...);
    profile.max_position = this->max_position_.value(x...);
    profile.position_width = this->position_width_.value(x...) * DEGREES_TO_RADIANS;
    profile.detent_strength = this->detent_strength_.value(x...);
    profile.endstop_strength = this->endstop_strength_.value(x...);
    profile.snap_point = this->snap_point_.value(x...);
    profile.snap_point_bias = this->snap_point_bias_.value(x...);
    profile.detent_positions_count = this->detent_positions_count_;
    for (uint8_t i = 0; i < this->detent_positions_count_; i++)
      profile.detent_positions[i] = this->detent_positions_[i];
    const bool set_position = this->position_.has_value();
    this->parent_->set_profile(profile, set_position, set_position ? this->position_.value(x...) : 0);
  }

 protected:
  uint8_t detent_positions_count_{0};
  int32_t detent_positions_[MAX_DETENT_POSITIONS]{};
};

template<typename... Ts> class SetPositionAction : public Action<Ts...>, public Parented<SmartKnob> {
 public:
  TEMPLATABLE_VALUE(int32_t, position)

  void play(const Ts &...x) override { this->parent_->set_position(this->position_.value(x...)); }
};

template<typename... Ts> class ClickAction : public Action<Ts...>, public Parented<SmartKnob> {
 public:
  TEMPLATABLE_VALUE(float, strength)
  TEMPLATABLE_VALUE(uint32_t, duration)

  void play(const Ts &...x) override {
    this->parent_->click(this->strength_.value(x...), this->duration_.value(x...));
  }
};

template<typename... Ts> class CalibrateAction : public Action<Ts...>, public Parented<SmartKnob> {
 public:
  void play(const Ts &...x) override { this->parent_->calibrate(); }
};

template<typename... Ts> class SetHapticsAction : public Action<Ts...>, public Parented<SmartKnob> {
 public:
  TEMPLATABLE_VALUE(bool, enabled)

  void play(const Ts &...x) override { this->parent_->set_haptics_enabled(this->enabled_.value(x...)); }
};

template<typename... Ts> class SetPressThresholdsAction : public Action<Ts...>, public Parented<SmartKnob> {
 public:
  TEMPLATABLE_VALUE(float, press)
  TEMPLATABLE_VALUE(float, release)

  void play(const Ts &...x) override {
    if (this->press_.has_value())
      this->parent_->set_press_threshold(this->press_.value(x...));
    if (this->release_.has_value())
      this->parent_->set_release_threshold(this->release_.value(x...));
  }
};

}  // namespace esphome::smartknob
