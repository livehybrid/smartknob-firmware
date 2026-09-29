#pragma once

#include <cstdint>
#include <utility>

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"

#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif
#ifdef USE_OTA_STATE_LISTENER
#include "esphome/components/ota/ota_backend.h"
#endif

#include "detent_engine.h"
#include "motor_controller.h"

namespace esphome::smartknob {

// ESPHome side of the SmartKnob: owns the real-time MotorController, turns
// its state into ESPHome triggers and entities, and persists calibration.
class SmartKnob : public Component
#ifdef USE_OTA_STATE_LISTENER
    , public ota::OTAGlobalStateListener
#endif
{
 public:
  // ---- configuration, called from generated code ----
  void set_motor_pins(int uh, int ul, int vh, int vl, int wh, int wl) {
    this->config_.pins = InverterPins{uh, ul, vh, vl, wh, wl};
  }
  void set_pwm_frequency(uint32_t hz) { this->config_.pwm_frequency = hz; }
  void set_dead_time_ns(uint32_t ns) { this->config_.dead_time_ns = ns; }
  void set_supply_voltage(float v) { this->config_.supply_voltage = v; }
  void set_voltage_limit(float v) { this->config_.voltage_limit = v; }
  void set_calibration_voltage(float v) { this->config_.calibration_voltage = v; }
  void set_encoder_pins(int clk, int data, int cs) {
    this->config_.encoder_clk = clk;
    this->config_.encoder_data = data;
    this->config_.encoder_cs = cs;
  }
  void set_encoder_spi_host(int host) { this->config_.spi_host = host == 3 ? SPI3_HOST : SPI2_HOST; }
  void set_encoder_clock(uint32_t hz) { this->config_.encoder_clock_hz = hz; }
  void set_sensor_filter(float hz) { this->config_.sensor_filter_hz = hz; }
  void set_control_frequency(uint32_t hz) { this->config_.control_hz = hz; }
  void set_invert(bool invert) { this->config_.invert = invert; }
  void set_task(int core, int priority) {
    this->config_.core = core;
    this->config_.priority = priority;
  }
  void set_strain_pins(int dout, int sck) {
    this->config_.strain_dout = dout;
    this->config_.strain_sck = sck;
  }
  void set_press_thresholds(float press, float release) {
    this->config_.press_threshold = press;
    this->config_.release_threshold = release;
  }
  void set_strain_sign(int sign) { this->config_.strain_sign = static_cast<int8_t>(sign); }
  // Haptic clicks played on press and release (0 = none); duration is the whole click.
  void set_press_clicks(float press, float release, uint32_t duration_us) {
    this->config_.press_click_strength = press;
    this->config_.release_click_strength = release;
    this->config_.click_half_period_us = duration_us / 2;
  }
  void set_press_max_velocity(float v) { this->config_.press_max_velocity = v; }
  void set_static_calibration(int pole_pairs, int direction, float zero_offset) {
    this->yaml_calibration_ = MotorCalibration{true, static_cast<uint8_t>(pole_pairs),
                                               static_cast<int8_t>(direction), zero_offset};
  }
  void set_auto_calibrate(bool auto_calibrate) { this->auto_calibrate_ = auto_calibrate; }
  void set_long_press_time(uint32_t ms) { this->long_press_ms_ = ms; }
  void set_menu_press_time(uint32_t ms) { this->menu_press_ms_ = ms; }
  void set_initial_profile(int32_t min_position, int32_t max_position, float width, float detent, float endstop,
                           float snap, float bias, int32_t position);
  void add_initial_detent_position(int32_t position) {
    if (this->profile_.detent_positions_count < MAX_DETENT_POSITIONS)
      this->profile_.detent_positions[this->profile_.detent_positions_count++] = position;
  }

#ifdef USE_SENSOR
  void set_position_sensor(sensor::Sensor *s) { this->position_sensor_ = s; }
  void set_strain_sensor(sensor::Sensor *s) { this->strain_sensor_ = s; }
  void set_loop_time_sensor(sensor::Sensor *s) { this->loop_time_sensor_ = s; }
#endif
#ifdef USE_BINARY_SENSOR
  void set_pressed_binary_sensor(binary_sensor::BinarySensor *s) { this->pressed_binary_sensor_ = s; }
#endif

  // ---- runtime API (actions and lambdas) ----
  void set_profile(const HapticProfile &profile, bool set_position = false, int32_t position = 0);
  void set_position(int32_t position);
  // Plays a click: `strength` volts one way, then the other, over `duration_us` in total.
  void click(float strength, uint32_t duration_us);
  void calibrate();
  void set_haptics_enabled(bool enabled);
  void set_press_threshold(float press);
  void set_release_threshold(float release);

  int32_t get_position() const { return this->state_.position; }
  float get_sub_position() const { return this->state_.sub_position; }
  float get_angle() const { return this->state_.angle; }
  bool is_pressed() const { return this->state_.pressed; }
  bool is_calibrated() const { return this->state_.calibrated; }
  bool is_calibrating() const { return this->state_.calibration_state == CalibrationState::RUNNING; }
  const HapticProfile &get_profile() const { return this->profile_; }

  template<typename F> void add_on_position_callback(F &&callback) {
    this->position_callback_.add(std::forward<F>(callback));
  }
  template<typename F> void add_on_press_callback(F &&callback) {
    this->press_callback_.add(std::forward<F>(callback));
  }
  template<typename F> void add_on_release_callback(F &&callback) {
    this->release_callback_.add(std::forward<F>(callback));
  }
  // Short press: released before long_press_time. Long press: fires once
  // while still held, after long_press_time; the release then fires no short press.
  template<typename F> void add_on_short_press_callback(F &&callback) {
    this->short_press_callback_.add(std::forward<F>(callback));
  }
  template<typename F> void add_on_long_press_callback(F &&callback) {
    this->long_press_callback_.add(std::forward<F>(callback));
  }
  // Fires once, while still held, after menu_press_time (held longer than a
  // plain long press). Independent of on_long_press: both fire in sequence
  // on a long enough hold.
  template<typename F> void add_on_menu_press_callback(F &&callback) {
    this->menu_press_callback_.add(std::forward<F>(callback));
  }
  template<typename F> void add_on_calibration_callback(F &&callback) {
    this->calibration_callback_.add(std::forward<F>(callback));
  }

  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void setup() override;
  void loop() override;
  void dump_config() override;
  void on_shutdown() override;
#ifdef USE_OTA_STATE_LISTENER
  // The motor task stalls while flash is written, so let the knob coast during updates.
  void on_ota_global_state(ota::OTAState state, float progress, uint8_t error, ota::OTAComponent *comp) override;
#endif

 protected:
  struct StoredCalibration {
    uint32_t magic;
    uint8_t pole_pairs;
    int8_t direction;
    float zero_offset;
  };
  static constexpr uint32_t CALIBRATION_MAGIC = 0x534B4331;  // "SKC1"

  void send_profile_(bool set_position, int32_t position);
  void apply_enabled_();

  MotorController controller_{};
  ControllerConfig config_{};
  ControllerState state_{};
  MotorCalibration yaml_calibration_{};
  const char *calibration_source_{"none"};
  ESPPreferenceObject calibration_pref_;
  bool auto_calibrate_{true};
  bool haptics_enabled_{true};
  bool ota_active_{false};
  bool started_{false};
  bool auto_calibration_done_{false};
  uint32_t boot_ms_{0};

  HapticProfile profile_{};
  int32_t initial_position_{0};
  int32_t last_position_{0};
  bool position_published_{false};
  bool has_pending_position_{false};
  int32_t pending_position_{0};
  uint32_t pending_position_ms_{0};
  uint32_t seen_press_count_{0};
  uint32_t seen_release_count_{0};
  uint32_t long_press_ms_{600};
  uint32_t menu_press_ms_{1200};
  uint32_t press_start_ms_{0};
  bool long_press_fired_{false};
  bool menu_press_fired_{false};
  uint32_t seen_calibration_sequence_{0};
  uint32_t last_strain_publish_ms_{0};
  float last_strain_published_{-1.0f};
  uint32_t last_diagnostics_ms_{0};
  uint32_t last_overrun_check_ms_{0};
  uint32_t last_overruns_{0};
  bool strain_fault_logged_{false};

#ifdef USE_SENSOR
  sensor::Sensor *position_sensor_{nullptr};
  sensor::Sensor *strain_sensor_{nullptr};
  sensor::Sensor *loop_time_sensor_{nullptr};
#endif
#ifdef USE_BINARY_SENSOR
  binary_sensor::BinarySensor *pressed_binary_sensor_{nullptr};
#endif

  CallbackManager<void(int32_t)> position_callback_{};
  CallbackManager<void()> press_callback_{};
  CallbackManager<void()> release_callback_{};
  CallbackManager<void()> short_press_callback_{};
  CallbackManager<void()> long_press_callback_{};
  CallbackManager<void()> menu_press_callback_{};
  CallbackManager<void(bool)> calibration_callback_{};
};

}  // namespace esphome::smartknob
