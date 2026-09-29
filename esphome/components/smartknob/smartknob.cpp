#include "smartknob.h"

#include <cinttypes>
#include <cmath>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome::smartknob {

static const char *const TAG = "smartknob";

void SmartKnob::set_initial_profile(int32_t min_position, int32_t max_position, float width, float detent,
                                    float endstop, float snap, float bias, int32_t position) {
  this->profile_.min_position = min_position;
  this->profile_.max_position = max_position;
  this->profile_.position_width = width;
  this->profile_.detent_strength = detent;
  this->profile_.endstop_strength = endstop;
  this->profile_.snap_point = snap;
  this->profile_.snap_point_bias = bias;
  this->profile_.detent_positions_count = 0;
  this->initial_position_ = position;
}

void SmartKnob::setup() {
  this->calibration_pref_ =
      global_preferences->make_preference<StoredCalibration>(fnv1_hash("smartknob_motor_calibration"), true);
  MotorCalibration calibration = this->yaml_calibration_;
  if (calibration.valid) {
    this->calibration_source_ = "YAML";
  } else {
    StoredCalibration stored{};
    if (this->calibration_pref_.load(&stored) && stored.magic == CALIBRATION_MAGIC && stored.pole_pairs >= 3 &&
        stored.pole_pairs <= 12 && (stored.direction == 1 || stored.direction == -1)) {
      calibration = MotorCalibration{true, stored.pole_pairs, stored.direction, stored.zero_offset};
      this->calibration_source_ = "flash";
    }
  }

  if (!this->controller_.start(this->config_, calibration)) {
    ESP_LOGE(TAG, "Motor start-up failed: %s", this->controller_.start_error());
    this->mark_failed();
    return;
  }
  this->started_ = true;
  this->boot_ms_ = millis();
#ifdef USE_OTA_STATE_LISTENER
  ota::get_global_ota_callback()->add_global_state_listener(this);
#endif
  this->send_profile_(true, this->initial_position_);
  this->state_ = this->controller_.state();
}

void SmartKnob::send_profile_(bool set_position, int32_t position) {
  if (set_position) {
    this->pending_position_ = position;
    this->pending_position_ms_ = millis();
    this->has_pending_position_ = true;
  }
  if (!this->controller_.set_profile(this->profile_, set_position, position))
    ESP_LOGW(TAG, "Motor task busy: haptic profile change dropped");
}

void SmartKnob::set_profile(const HapticProfile &profile, bool set_position, int32_t position) {
  const char *error = profile.validate();
  if (error != nullptr) {
    ESP_LOGW(TAG, "Ignoring haptic profile: %s", error);
    return;
  }
  this->profile_ = profile;
  if (!this->started_) {
    if (set_position)
      this->initial_position_ = position;
    return;
  }
  this->send_profile_(set_position, position);
}

void SmartKnob::set_position(int32_t position) {
  if (!this->started_) {
    this->initial_position_ = position;
    return;
  }
  this->pending_position_ = position;
  this->pending_position_ms_ = millis();
  this->has_pending_position_ = true;
  if (!this->controller_.set_position(position))
    ESP_LOGW(TAG, "Motor task busy: position change dropped");
}

void SmartKnob::click(float strength, uint32_t duration_us) {
  if (this->started_ && !this->controller_.click(strength, duration_us / 2))
    ESP_LOGW(TAG, "Motor task busy: click dropped");
}

void SmartKnob::calibrate() {
  if (!this->started_)
    return;
  ESP_LOGI(TAG, "Motor calibration requested: keep hands off the knob for about 20 s");
  if (!this->controller_.calibrate())
    ESP_LOGW(TAG, "Motor task busy: calibration request dropped");
}

void SmartKnob::set_haptics_enabled(bool enabled) {
  this->haptics_enabled_ = enabled;
  this->apply_enabled_();
}

void SmartKnob::apply_enabled_() {
  if (this->started_ && !this->controller_.set_enabled(this->haptics_enabled_ && !this->ota_active_))
    ESP_LOGW(TAG, "Motor task busy: enable change dropped");
}

void SmartKnob::on_shutdown() {
  if (this->started_)
    this->controller_.set_enabled(false);
}

#ifdef USE_OTA_STATE_LISTENER
void SmartKnob::on_ota_global_state(ota::OTAState state, float progress, uint8_t error, ota::OTAComponent *comp) {
  if (state == ota::OTA_STARTED) {
    this->ota_active_ = true;
  } else if (state == ota::OTA_ERROR || state == ota::OTA_ABORT) {
    this->ota_active_ = false;
  } else {
    return;
  }
  this->apply_enabled_();
}
#endif

void SmartKnob::set_press_threshold(float press) {
  this->config_.press_threshold = press;
  if (this->started_)
    this->controller_.set_press_thresholds(this->config_.press_threshold, this->config_.release_threshold);
}

void SmartKnob::set_release_threshold(float release) {
  this->config_.release_threshold = release;
  if (this->started_)
    this->controller_.set_press_thresholds(this->config_.press_threshold, this->config_.release_threshold);
}

void SmartKnob::loop() {
  if (!this->started_)
    return;
  this->state_ = this->controller_.state();
  const ControllerState &s = this->state_;
  const uint32_t now = millis();

  // Position changes. Changes we asked for ourselves (set_position, or a
  // profile with a position) are published but do not fire the trigger, so
  // an update coming from Home Assistant is not echoed back to it.
  if (this->has_pending_position_ && now - this->pending_position_ms_ > 250)
    this->has_pending_position_ = false;
  if (!this->position_published_ || s.position != this->last_position_) {
    const bool first = !this->position_published_;
    const bool programmatic = this->has_pending_position_ && s.position == this->pending_position_;
    if (programmatic)
      this->has_pending_position_ = false;
    this->last_position_ = s.position;
    this->position_published_ = true;
#ifdef USE_SENSOR
    if (this->position_sensor_ != nullptr)
      this->position_sensor_->publish_state(static_cast<float>(s.position));
#endif
    if (!first && !programmatic)
      this->position_callback_.call(s.position);
  }

  // Presses and releases. Counters catch a press and release that both
  // happen between two passes of the main loop.
  for (;;) {
    if (this->seen_press_count_ == this->seen_release_count_ && this->seen_press_count_ != s.press_count) {
      this->seen_press_count_++;
      this->press_start_ms_ = now;
      this->long_press_fired_ = false;
#ifdef USE_BINARY_SENSOR
      if (this->pressed_binary_sensor_ != nullptr)
        this->pressed_binary_sensor_->publish_state(true);
#endif
      this->press_callback_.call();
    } else if (this->seen_release_count_ != this->seen_press_count_ &&
               this->seen_release_count_ != s.release_count) {
      this->seen_release_count_++;
#ifdef USE_BINARY_SENSOR
      if (this->pressed_binary_sensor_ != nullptr)
        this->pressed_binary_sensor_->publish_state(false);
#endif
      this->release_callback_.call();
      if (!this->long_press_fired_)
        this->short_press_callback_.call();
    } else {
      break;
    }
  }
  if (this->seen_press_count_ != this->seen_release_count_ && !this->long_press_fired_ &&
      now - this->press_start_ms_ >= this->long_press_ms_) {
    this->long_press_fired_ = true;
    this->long_press_callback_.call();
  }

  // Calibration results.
  if (s.calibration_sequence != this->seen_calibration_sequence_) {
    this->seen_calibration_sequence_ = s.calibration_sequence;
    const bool ok = s.calibration_state == CalibrationState::SUCCEEDED;
    if (ok) {
      StoredCalibration stored{CALIBRATION_MAGIC, s.calibration.pole_pairs, s.calibration.direction,
                               s.calibration.zero_offset};
      if (this->calibration_pref_.save(&stored) && global_preferences->sync()) {
        this->calibration_source_ = "flash";
        ESP_LOGI(TAG, "Motor calibration saved: %u pole pairs, direction %d, zero offset %.3f",
                 s.calibration.pole_pairs, s.calibration.direction, s.calibration.zero_offset);
      } else {
        ESP_LOGW(TAG, "Motor calibration succeeded but could not be saved");
      }
      // Commands sent during calibration may have been dropped: re-apply.
      this->send_profile_(false, 0);
    } else {
      ESP_LOGE(TAG, "Motor calibration failed: %s", s.calibration_message);
    }
    this->calibration_callback_.call(ok);
  }

  // First boot: calibrate automatically once the system has settled.
  if (this->auto_calibrate_ && !this->auto_calibration_done_ && !s.calibrated &&
      s.calibration_state == CalibrationState::IDLE && s.sensor_ok && now - this->boot_ms_ > 3000) {
    this->auto_calibration_done_ = true;
    ESP_LOGW(TAG, "Motor not calibrated yet: calibrating now");
    this->calibrate();
  }

#ifdef USE_SENSOR
  // Strain signal for tuning the press thresholds: at most 4 Hz, only on change.
  if (this->strain_sensor_ != nullptr && s.strain_present && now - this->last_strain_publish_ms_ >= 250) {
    this->last_strain_publish_ms_ = now;
    const float value = std::round(s.strain);
    const float step = std::fmax(1.0f, 0.02f * this->config_.release_threshold);
    if (std::fabs(value - this->last_strain_published_) >= step) {
      this->last_strain_published_ = value;
      this->strain_sensor_->publish_state(value);
    }
  }
  if (this->loop_time_sensor_ != nullptr && now - this->last_diagnostics_ms_ >= 10000) {
    this->last_diagnostics_ms_ = now;
    this->loop_time_sensor_->publish_state(static_cast<float>(s.max_loop_us));
    this->controller_.reset_max_loop_time();
  }
#endif
}

void SmartKnob::dump_config() {
  const ControllerConfig &c = this->config_;
  ESP_LOGCONFIG(TAG, "SmartKnob:");
  ESP_LOGCONFIG(TAG, "  Motor: UH=%d UL=%d VH=%d VL=%d WH=%d WL=%d", c.pins.uh, c.pins.ul, c.pins.vh, c.pins.vl,
                c.pins.wh, c.pins.wl);
  ESP_LOGCONFIG(TAG, "  PWM %" PRIu32 " Hz, dead time %" PRIu32 " ns, supply %.1f V, voltage limit %.2f V",
                c.pwm_frequency, c.dead_time_ns, c.supply_voltage, c.voltage_limit);
  ESP_LOGCONFIG(TAG, "  Encoder: MT6701 on SPI%d (CLK=%d DATA=%d CS=%d) at %" PRIu32 " Hz, filter %.0f Hz",
                c.spi_host == SPI3_HOST ? 3 : 2, c.encoder_clk, c.encoder_data, c.encoder_cs, c.encoder_clock_hz,
                c.sensor_filter_hz);
  ESP_LOGCONFIG(TAG, "  Control loop: %" PRIu32 " Hz on core %d, priority %d%s", c.control_hz, c.core, c.priority,
                c.invert ? ", direction inverted" : "");
  if (c.strain_dout >= 0) {
    ESP_LOGCONFIG(TAG, "  Strain gauge: DOUT=%d SCK=%d, press %.0f, release %.0f", c.strain_dout, c.strain_sck,
                  c.press_threshold, c.release_threshold);
  }
  if (this->is_failed()) {
    ESP_LOGE(TAG, "  Start-up failed: %s", this->controller_.start_error());
    return;
  }
  const ControllerState s = this->controller_.state();
  if (s.calibrated) {
    ESP_LOGCONFIG(TAG, "  Calibration (%s): %u pole pairs, direction %d, zero offset %.3f rad",
                  this->calibration_source_, s.calibration.pole_pairs, s.calibration.direction,
                  s.calibration.zero_offset);
  } else {
    ESP_LOGCONFIG(TAG, "  Calibration: none%s", this->auto_calibrate_ ? " (runs automatically after boot)" : "");
  }
  ESP_LOGCONFIG(TAG, "  Encoder %s, field status %u, %" PRIu32 " read errors, loop max %" PRIu32 " us",
                s.sensor_ok ? "OK" : "NOT RESPONDING", s.field_status, s.sensor_errors, s.max_loop_us);
  if (c.strain_dout >= 0)
    ESP_LOGCONFIG(TAG, "  Strain gauge %s, %.1f samples/s", s.strain_present ? "OK" : "not ready",
                  s.strain_rate_hz);
}

}  // namespace esphome::smartknob
