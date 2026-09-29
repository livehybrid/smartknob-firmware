#include "motor_controller.h"

#ifdef USE_ESP32

#include <cmath>

#include <esp_attr.h>
#include <esp_log.h>
#include <esp_timer.h>

#include "foc_math.h"

namespace esphome::smartknob {

static const char *const TAG = "smartknob.motor";

static constexpr float RAD_TO_DEG = 57.2957795f;
static constexpr int64_t STUCK_PRESS_US = 30000000;  // re-baseline after a 30 s "press"

static bool IRAM_ATTR on_control_timer(gptimer_handle_t /*timer*/, const gptimer_alarm_event_data_t * /*edata*/,
                                       void *ctx) {
  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(static_cast<TaskHandle_t>(ctx), &woken);
  return woken == pdTRUE;
}

// ------------------------------------------------------------- public API --

bool MotorController::start(const ControllerConfig &config, const MotorCalibration &calibration) {
  this->config_ = config;
  this->calibration_ = calibration;
  this->commands_ = xQueueCreate(8, sizeof(Command));
  if (this->commands_ == nullptr) {
    this->start_error_ = "no memory for the command queue";
    return false;
  }
  if (xTaskCreatePinnedToCore(task_entry_, "smartknob", 6144, this, config.priority, &this->task_, config.core) !=
      pdPASS) {
    this->start_error_ = "could not create the motor task";
    return false;
  }
  for (int i = 0; i < 100 && !this->start_done_; i++)
    vTaskDelay(pdMS_TO_TICKS(10));
  if (!this->start_done_) {
    this->start_error_ = "motor task did not start";
    return false;
  }
  return this->start_error_ == nullptr;
}

bool MotorController::send_(const Command &command) {
  return this->commands_ != nullptr && xQueueSend(this->commands_, &command, 0) == pdTRUE;
}

bool MotorController::set_profile(const HapticProfile &profile, bool set_position, int32_t position) {
  Command c{};
  c.type = CommandType::PROFILE;
  c.profile = profile;
  c.flag = set_position;
  c.position = position;
  return this->send_(c);
}

bool MotorController::set_position(int32_t position) {
  Command c{};
  c.type = CommandType::SET_POSITION;
  c.position = position;
  return this->send_(c);
}

bool MotorController::click(float strength, uint32_t half_period_us) {
  Command c{};
  c.type = CommandType::CLICK;
  c.value_a = strength;
  c.half_period_us = half_period_us;
  return this->send_(c);
}

bool MotorController::calibrate() {
  Command c{};
  c.type = CommandType::CALIBRATE;
  return this->send_(c);
}

bool MotorController::set_enabled(bool enabled) {
  Command c{};
  c.type = CommandType::ENABLE;
  c.flag = enabled;
  return this->send_(c);
}

bool MotorController::set_press_thresholds(float press, float release) {
  Command c{};
  c.type = CommandType::PRESS_THRESHOLDS;
  c.value_a = press;
  c.value_b = release;
  return this->send_(c);
}

ControllerState MotorController::state() {
  portENTER_CRITICAL(&this->state_lock_);
  ControllerState copy = this->shared_;
  portEXIT_CRITICAL(&this->state_lock_);
  return copy;
}

void MotorController::reset_max_loop_time() { this->reset_max_loop_ = true; }

// ------------------------------------------------------------- motor task --

void MotorController::task_entry_(void *arg) {
  static_cast<MotorController *>(arg)->run_();
  vTaskDelete(nullptr);
}

bool MotorController::init_hardware_() {
  const float fs = static_cast<float>(this->config_.control_hz);
  this->filter_alpha_ = this->config_.sensor_filter_hz > 0.0f
                            ? 1.0f - std::exp(-2.0f * foc::PI_F * this->config_.sensor_filter_hz / fs)
                            : 1.0f;

  esp_err_t err = this->encoder_.begin(this->config_.spi_host, this->config_.encoder_clk, this->config_.encoder_data,
                                       this->config_.encoder_cs, this->config_.encoder_clock_hz);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Encoder SPI set-up failed: %s", esp_err_to_name(err));
    this->start_error_ = "encoder SPI set-up failed (is another component using this SPI host?)";
    return false;
  }
  err = this->inverter_.begin(this->config_.pins, this->config_.pwm_frequency, this->config_.dead_time_ns);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "MCPWM set-up failed: %s", esp_err_to_name(err));
    this->start_error_ = "motor PWM set-up failed";
    return false;
  }
  if (this->config_.strain_dout >= 0 && this->config_.strain_sck >= 0) {
    this->strain_.begin(this->config_.strain_dout, this->config_.strain_sck);
    this->strain_enabled_ = true;
  }

  // Hardware timer paces the control loop; its interrupt lands on this core.
  gptimer_config_t timer_config{};
  timer_config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
  timer_config.direction = GPTIMER_COUNT_UP;
  timer_config.resolution_hz = 1000000;
  if ((err = gptimer_new_timer(&timer_config, &this->timer_)) != ESP_OK) {
    ESP_LOGE(TAG, "GPTimer set-up failed: %s", esp_err_to_name(err));
    this->start_error_ = "control timer set-up failed";
    return false;
  }
  gptimer_event_callbacks_t callbacks{};
  callbacks.on_alarm = on_control_timer;
  gptimer_register_event_callbacks(this->timer_, &callbacks, xTaskGetCurrentTaskHandle());
  gptimer_alarm_config_t alarm{};
  alarm.alarm_count = 1000000ULL / this->config_.control_hz;
  alarm.reload_count = 0;
  alarm.flags.auto_reload_on_alarm = true;
  gptimer_set_alarm_action(this->timer_, &alarm);
  gptimer_enable(this->timer_);
  gptimer_start(this->timer_);
  return true;
}

void MotorController::run_() {
  if (!this->init_hardware_()) {
    this->start_done_ = true;
    return;
  }
  for (int i = 0; i < 50 && !this->read_sensor_(0.001f); i++)
    vTaskDelay(1);
  this->engine_.set_position(0, this->knob_angle_);
  this->start_done_ = true;

  const float nominal_dt = 1.0f / static_cast<float>(this->config_.control_hz);
  int64_t last_us = esp_timer_get_time();
  for (;;) {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
    const int64_t start_us = esp_timer_get_time();
    float dt = static_cast<float>(start_us - last_us) * 1e-6f;
    last_us = start_us;
    if (!(dt > 0.0f) || dt > 0.05f)
      dt = nominal_dt;

    Command command;
    for (int i = 0; i < 4 && xQueueReceive(this->commands_, &command, 0) == pdTRUE; i++)
      this->handle_command_(command, start_us);
    if (this->calibration_requested_) {
      this->calibration_requested_ = false;
      this->run_calibration_();
      last_us = esp_timer_get_time();
      continue;
    }

    this->read_sensor_(dt);
    if (this->strain_enabled_)
      this->update_strain_(start_us);

    float torque = 0.0f;
    if (this->calibration_.valid && this->sensor_ok_) {
      torque = this->engine_
                   .update(this->knob_angle_, this->velocity_, dt, static_cast<uint32_t>(start_us / 1000))
                   .torque;
      if (this->clicking_) {
        const int64_t elapsed = start_us - this->click_start_us_;
        if (elapsed < static_cast<int64_t>(this->click_half_us_)) {
          torque = this->click_strength_;
        } else if (elapsed < 2 * static_cast<int64_t>(this->click_half_us_)) {
          torque = -this->click_strength_;
        } else {
          this->clicking_ = false;
        }
      }
    }
    this->torque_ = torque;
    const bool drive = this->calibration_.valid && this->enabled_ && this->sensor_ok_;
    this->inverter_.set_enabled(drive);
    if (drive)
      this->apply_knob_torque_(torque);
    this->publish_(start_us);
  }
}

void MotorController::handle_command_(const Command &command, int64_t now_us) {
  switch (command.type) {
    case CommandType::PROFILE:
      this->engine_.set_profile(command.profile, this->knob_angle_, command.flag, command.position);
      break;
    case CommandType::SET_POSITION:
      this->engine_.set_position(command.position, this->knob_angle_);
      break;
    case CommandType::CLICK:
      this->start_click_(command.value_a, command.half_period_us, now_us);
      break;
    case CommandType::CALIBRATE:
      this->calibration_requested_ = true;
      break;
    case CommandType::ENABLE:
      this->enabled_ = command.flag;
      break;
    case CommandType::PRESS_THRESHOLDS:
      this->config_.press_threshold = command.value_a;
      this->config_.release_threshold = command.value_b;
      break;
  }
}

bool MotorController::read_sensor_(float dt) {
  foc::MT6701Frame frame{};
  if (!this->encoder_.read(frame) || !frame.crc_ok) {
    this->sensor_errors_++;
    if (++this->consecutive_errors_ > 100)
      this->sensor_ok_ = false;
    return false;
  }
  this->consecutive_errors_ = 0;
  this->sensor_ok_ = true;
  this->field_status_ = frame.field_status;
  const float a = frame.angle();
  this->raw_angle_ = a;
  const float frame_sign = (this->config_.invert ? -1.0f : 1.0f) *
                           static_cast<float>(this->calibration_.valid ? this->calibration_.direction : 1);
  if (!this->have_angle_) {
    this->filt_x_ = std::cos(a);
    this->filt_y_ = std::sin(a);
    this->prev_filtered_ = a;
    this->shaft_ = a;
    this->knob_angle_ = frame_sign * a;
    this->velocity_ = 0.0f;
    this->have_angle_ = true;
    return true;
  }
  // Low-pass on the unit vector (no wrap-around artefacts), then unwrap.
  this->filt_x_ += (std::cos(a) - this->filt_x_) * this->filter_alpha_;
  this->filt_y_ += (std::sin(a) - this->filt_y_) * this->filter_alpha_;
  const float filtered = std::atan2(this->filt_y_, this->filt_x_);
  this->shaft_ += foc::wrap_pi(filtered - this->prev_filtered_);
  this->prev_filtered_ = filtered;

  float knob = frame_sign * this->shaft_;
  // Keep float precision over long free spins: shift everything by whole turns.
  if (std::fabs(this->shaft_) > 2000.0f) {
    const float turns = std::round(this->shaft_ / foc::TWO_PI_F) * foc::TWO_PI_F;
    this->shaft_ -= turns;
    const float shift = frame_sign * turns;
    knob -= shift;
    this->knob_angle_ -= shift;
    this->engine_.shift_reference(-shift);
  }
  const float v = (knob - this->knob_angle_) / dt;
  this->velocity_ += (v - this->velocity_) * (dt / (this->config_.velocity_filter_s + dt));
  this->knob_angle_ = knob;
  return true;
}

void MotorController::update_strain_(int64_t now_us) {
  if (!this->strain_.ready())
    return;
  const int32_t raw = this->strain_.read();
  const float raw_f = static_cast<float>(raw);

  this->strain_samples_++;
  if (this->strain_rate_window_us_ == 0) {
    this->strain_rate_window_us_ = now_us;
  } else if (now_us - this->strain_rate_window_us_ >= 1000000) {
    this->strain_rate_hz_ = static_cast<float>(this->strain_samples_) * 1e6f /
                            static_cast<float>(now_us - this->strain_rate_window_us_);
    this->strain_samples_ = 0;
    this->strain_rate_window_us_ = now_us;
  }
  const float sample_dt =
      this->last_strain_us_ != 0 ? static_cast<float>(now_us - this->last_strain_us_) * 1e-6f : 0.0125f;
  this->last_strain_us_ = now_us;

  if (!this->baseline_ready_) {
    this->baseline_acc_ += raw;
    if (++this->baseline_count_ >= 16) {
      this->baseline_ = static_cast<float>(this->baseline_acc_) / 16.0f;
      this->baseline_ready_ = true;
    }
    return;
  }

  const float delta = raw_f - this->baseline_;
  const float signal =
      this->config_.strain_sign == 0 ? std::fabs(delta) : static_cast<float>(this->config_.strain_sign) * delta;
  this->strain_value_ = signal;

  if (!this->pressed_) {
    if (signal > this->config_.press_threshold) {
      // Two consecutive samples over the threshold: rejects single-sample
      // spikes (the ghost presses reported upstream). Ignored while spinning fast.
      if (std::fabs(this->velocity_) < this->config_.press_max_velocity && ++this->over_count_ >= 2) {
        this->over_count_ = 0;
        this->pressed_ = true;
        this->pressed_since_us_ = now_us;
        this->press_count_++;
        if (this->config_.press_click_strength > 0.0f)
          this->start_click_(this->config_.press_click_strength, this->config_.click_half_period_us, now_us);
      }
    } else {
      this->over_count_ = 0;
      // Track slow drift (temperature, creep) only while clearly released.
      if (signal < this->config_.release_threshold)
        this->baseline_ += (raw_f - this->baseline_) * std::fmin(1.0f, sample_dt / 3.0f);
    }
  } else {
    const bool stuck = now_us - this->pressed_since_us_ > STUCK_PRESS_US;
    if (signal < this->config_.release_threshold || stuck) {
      if (stuck)
        this->baseline_ = raw_f;
      this->pressed_ = false;
      this->release_count_++;
      if (this->config_.release_click_strength > 0.0f)
        this->start_click_(this->config_.release_click_strength, this->config_.click_half_period_us, now_us);
    }
  }
}

void MotorController::start_click_(float strength, uint32_t half_period_us, int64_t now_us) {
  this->click_strength_ = strength;
  this->click_half_us_ = half_period_us;
  this->click_start_us_ = now_us;
  this->clicking_ = strength > 0.0f && half_period_us > 0;
}

void MotorController::apply_knob_torque_(float torque) {
  // Knob frame -> motor frame (positive Uq increases the calibrated sensor angle).
  float uq = (this->config_.invert ? -1.0f : 1.0f) * torque;
  const float limit = this->config_.voltage_limit;
  uq = uq > limit ? limit : (uq < -limit ? -limit : uq);
  // Commutate on the unfiltered angle so the field never lags the rotor.
  const float theta_e = foc::normalize_angle(
      static_cast<float>(this->calibration_.pole_pairs * this->calibration_.direction) * this->raw_angle_ -
      this->calibration_.zero_offset);
  this->apply_voltage_(uq, 0.0f, theta_e);
}

void MotorController::apply_voltage_(float uq, float ud, float theta_e) {
  const foc::PhaseDuty duty = foc::svpwm(uq, ud, theta_e, this->config_.supply_voltage);
  this->inverter_.set_duty(duty.a, duty.b, duty.c);
}

void MotorController::publish_(int64_t loop_start_us) {
  const auto elapsed = static_cast<uint32_t>(esp_timer_get_time() - loop_start_us);
  this->loop_count_++;
  if (this->reset_max_loop_) {
    this->max_loop_us_ = 0;
    this->reset_max_loop_ = false;
  }
  if (elapsed > this->max_loop_us_)
    this->max_loop_us_ = elapsed;

  portENTER_CRITICAL(&this->state_lock_);
  ControllerState &s = this->shared_;
  s.position = this->engine_.position();
  s.sub_position = this->engine_.sub_position();
  s.angle = this->knob_angle_;
  s.velocity = this->velocity_;
  s.torque = this->torque_;
  s.calibrated = this->calibration_.valid;
  s.enabled = this->enabled_;
  s.sensor_ok = this->sensor_ok_;
  s.field_status = this->field_status_;
  s.sensor_errors = this->sensor_errors_;
  s.calibration_state = this->calibration_state_;
  s.calibration_sequence = this->calibration_sequence_;
  s.calibration = this->calibration_;
  s.calibration_message = this->calibration_message_;
  s.strain_present = this->strain_enabled_ && this->baseline_ready_;
  s.pressed = this->pressed_;
  s.press_count = this->press_count_;
  s.release_count = this->release_count_;
  s.strain = this->strain_value_;
  s.strain_rate_hz = this->strain_rate_hz_;
  s.loop_count = this->loop_count_;
  s.max_loop_us = this->max_loop_us_;
  portEXIT_CRITICAL(&this->state_lock_);
}

// ------------------------------------------------------------ calibration --
//
// Same procedure as the original firmware, in one consistent convention:
//  1. Sweep 3 electrical turns open-loop: which way does the sensor move?
//  2. Sweep 20 electrical turns: mechanical travel gives the pole pair count.
//  3. Step across half a mechanical turn and back with voltage on the d axis
//     (the rotor aligns to the commanded angle): the circular mean of
//     (sensor electrical angle - commanded angle) is the zero offset.
// Running: theta_e = pole_pairs * direction * sensor_angle - zero_offset.

bool MotorController::calibration_track_(float &unwrapped) {
  foc::MT6701Frame frame{};
  if (!this->encoder_.read(frame) || !frame.crc_ok)
    return false;
  const float a = frame.angle();
  unwrapped += foc::wrap_pi(a - this->raw_angle_);
  this->raw_angle_ = a;
  return true;
}

float MotorController::calibration_average_(float &unwrapped, int samples) {
  float sum = 0.0f;
  for (int i = 0; i < samples; i++) {
    vTaskDelay(1);
    this->calibration_track_(unwrapped);
    sum += unwrapped;
  }
  return sum / static_cast<float>(samples);
}

void MotorController::calibration_hold_(float theta, int ms, float &unwrapped) {
  this->apply_voltage_(0.0f, this->config_.calibration_voltage, theta);
  for (int i = 0; i < ms; i++) {
    vTaskDelay(1);
    this->calibration_track_(unwrapped);
  }
}

bool MotorController::calibration_sweep_(float &theta, float target, float step, float &unwrapped) {
  while (step > 0.0f ? theta < target : theta > target) {
    theta += step;
    this->apply_voltage_(0.0f, this->config_.calibration_voltage, theta);
    vTaskDelay(1);
    this->calibration_track_(unwrapped);
  }
  return true;
}

void MotorController::run_calibration_() {
  ESP_LOGI(TAG, "Motor calibration started: keep hands off the knob for about 20 s");
  this->calibration_state_ = CalibrationState::RUNNING;
  this->calibration_message_ = "running";
  this->clicking_ = false;
  this->publish_(esp_timer_get_time());

  const MotorCalibration previous = this->calibration_;
  auto fail = [&](const char *reason) {
    ESP_LOGE(TAG, "Motor calibration failed: %s", reason);
    this->calibration_ = previous;
    this->calibration_state_ = CalibrationState::FAILED;
    this->calibration_message_ = reason;
    this->calibration_sequence_++;
    this->inverter_.set_enabled(false);
    this->reseed_after_calibration_();
  };

  foc::MT6701Frame frame{};
  if (!this->encoder_.read(frame) || !frame.crc_ok) {
    fail("encoder not responding");
    return;
  }
  this->raw_angle_ = frame.angle();
  float unwrapped = this->raw_angle_;
  float theta = 0.0f;
  this->inverter_.set_enabled(true);

  // 1. Direction.
  this->calibration_hold_(theta, 500, unwrapped);
  const float start = this->calibration_average_(unwrapped, 20);
  this->calibration_sweep_(theta, 3.0f * foc::TWO_PI_F, 0.01f, unwrapped);
  this->calibration_hold_(theta, 200, unwrapped);
  const float moved = this->calibration_average_(unwrapped, 20) - start;
  ESP_LOGD(TAG, "3 electrical turns moved the knob %.1f deg", moved * RAD_TO_DEG);
  if (std::fabs(moved) < 30.0f / RAD_TO_DEG || std::fabs(moved) > 180.0f / RAD_TO_DEG) {
    fail("the knob did not turn as expected: keep hands off it and check the motor cable");
    return;
  }
  const int8_t direction = moved > 0.0f ? 1 : -1;

  // 2. Pole pairs.
  this->calibration_hold_(theta, 300, unwrapped);
  const float p0 = this->calibration_average_(unwrapped, 50);
  this->calibration_sweep_(theta, theta + 20.0f * foc::TWO_PI_F, 0.03f, unwrapped);
  this->calibration_hold_(theta, 500, unwrapped);
  const float mechanical = static_cast<float>(direction) * (this->calibration_average_(unwrapped, 50) - p0);
  const float ratio = mechanical > 0.01f ? 20.0f * foc::TWO_PI_F / mechanical : 0.0f;
  ESP_LOGD(TAG, "Electrical/mechanical ratio %.2f", ratio);
  if (ratio < 3.0f || ratio > 12.0f) {
    fail("unexpected pole pair count");
    return;
  }
  const long pole_pairs = std::lround(ratio);
  if (std::fabs(ratio - static_cast<float>(pole_pairs)) > 0.3f) {
    fail("pole pair measurement inconsistent (knob slipped?)");
    return;
  }

  // 3. Zero offset.
  float sx = 0.0f, sy = 0.0f;
  int n = 0;
  auto sample = [&]() {
    this->calibration_hold_(theta, 60, unwrapped);
    const float mech = this->calibration_average_(unwrapped, 20);
    const float measured = static_cast<float>(pole_pairs * direction) * mech;
    const float diff = foc::wrap_pi(measured - theta);
    sx += std::cos(diff);
    sy += std::sin(diff);
    n++;
  };
  const float begin = theta;
  const float span = static_cast<float>(pole_pairs) * foc::PI_F;  // half a mechanical turn
  for (; theta < begin + span; theta += 0.4f)
    sample();
  for (; theta > begin; theta -= 0.4f)
    sample();
  const float resultant = std::sqrt(sx * sx + sy * sy) / static_cast<float>(n);
  if (resultant < 0.9f) {
    fail("encoder readings inconsistent during alignment");
    return;
  }

  this->calibration_.valid = true;
  this->calibration_.pole_pairs = static_cast<uint8_t>(pole_pairs);
  this->calibration_.direction = direction;
  this->calibration_.zero_offset = std::atan2(sy, sx);
  this->calibration_state_ = CalibrationState::SUCCEEDED;
  this->calibration_message_ = "ok";
  this->calibration_sequence_++;
  ESP_LOGI(TAG, "Motor calibration done: %u pole pairs, direction %d, zero offset %.3f rad (consistency %.3f)",
           this->calibration_.pole_pairs, this->calibration_.direction, this->calibration_.zero_offset, resultant);
  this->inverter_.set_enabled(false);
  this->reseed_after_calibration_();
}

void MotorController::reseed_after_calibration_() {
  // The knob moved and the frame may have flipped: restart filters and keep
  // the logical position at wherever the knob is now.
  const int32_t position = this->engine_.position();
  this->have_angle_ = false;
  for (int i = 0; i < 20 && !this->read_sensor_(0.001f); i++)
    vTaskDelay(1);
  this->engine_.set_position(position, this->knob_angle_);
  this->publish_(esp_timer_get_time());
}

}  // namespace esphome::smartknob

#endif  // USE_ESP32
