#pragma once

// Real-time motor task for the SmartKnob: reads the MT6701 at a fixed rate,
// runs the detent engine, drives the motor with voltage-mode FOC and watches
// the strain gauge for presses. Runs pinned to one core at high priority and
// talks to ESPHome's main loop only through a command queue and a
// lock-protected state snapshot, so no ESPHome API is called from this task.

#ifdef USE_ESP32

#include <cstdint>

#include <driver/gptimer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "detent_engine.h"
#include "hardware.h"

namespace esphome::smartknob {

struct MotorCalibration {
  bool valid{false};
  uint8_t pole_pairs{7};
  int8_t direction{1};      // sensor direction relative to the commanded electrical angle
  float zero_offset{0.0f};  // electrical radians
};

enum class CalibrationState : uint8_t { IDLE, RUNNING, SUCCEEDED, FAILED };

struct ControllerConfig {
  InverterPins pins{};
  uint32_t pwm_frequency{25000};
  uint32_t dead_time_ns{500};
  float supply_voltage{5.0f};
  float voltage_limit{2.9f};
  float calibration_voltage{2.5f};
  spi_host_device_t spi_host{SPI2_HOST};
  int encoder_clk{-1};
  int encoder_data{-1};
  int encoder_cs{-1};
  uint32_t encoder_clock_hz{4000000};
  uint32_t control_hz{5000};
  float sensor_filter_hz{300.0f};
  float velocity_filter_s{0.002f};
  bool invert{false};
  int core{1};
  int priority{20};
  // Strain gauge (press detection); disabled if pins are negative.
  int strain_dout{-1};
  int strain_sck{-1};
  float press_threshold{30000.0f};
  float release_threshold{15000.0f};
  int8_t strain_sign{0};  // 0: either direction, 1: positive only, -1: negative only
  float press_click_strength{0.0f};
  float release_click_strength{0.0f};
  uint32_t click_half_period_us{3000};  // push, then pull back for the same time
  float press_max_velocity{8.0f};  // rad/s: ignore "presses" while spinning fast
};

struct ControllerState {
  int32_t position{0};
  float sub_position{0.0f};
  float angle{0.0f};
  float velocity{0.0f};
  float torque{0.0f};
  bool calibrated{false};
  bool enabled{true};
  bool sensor_ok{false};
  uint8_t field_status{0};
  uint32_t sensor_errors{0};
  CalibrationState calibration_state{CalibrationState::IDLE};
  uint32_t calibration_sequence{0};  // increments when a calibration finishes
  MotorCalibration calibration{};
  const char *calibration_message{""};
  bool strain_present{false};
  bool pressed{false};
  uint32_t press_count{0};
  uint32_t release_count{0};
  float strain{0.0f};
  float strain_rate_hz{0.0f};
  uint32_t loop_count{0};
  uint32_t max_loop_us{0};
};

class MotorController {
 public:
  // Starts the task; returns false (see start_error()) if hardware set-up failed.
  bool start(const ControllerConfig &config, const MotorCalibration &calibration);
  const char *start_error() const { return this->start_error_; }

  // Thread-safe, non-blocking. Return false if the command queue is full.
  bool set_profile(const HapticProfile &profile, bool set_position, int32_t position);
  bool set_position(int32_t position);
  bool click(float strength, uint32_t half_period_us);
  bool calibrate();
  bool set_enabled(bool enabled);
  bool set_press_thresholds(float press, float release);

  // Thread-safe copy of the latest state.
  ControllerState state();
  void reset_max_loop_time();

 protected:
  enum class CommandType : uint8_t { PROFILE, SET_POSITION, CLICK, CALIBRATE, ENABLE, PRESS_THRESHOLDS };
  struct Command {
    CommandType type;
    bool flag;
    int32_t position;
    float value_a;
    float value_b;
    uint32_t half_period_us;
    HapticProfile profile;
  };

  static void task_entry_(void *arg);
  bool send_(const Command &command);
  void run_();
  bool init_hardware_();
  void handle_command_(const Command &command, int64_t now_us);
  bool read_sensor_(float dt);
  void update_strain_(int64_t now_us);
  void start_click_(float strength, uint32_t half_period_us, int64_t now_us);
  void apply_knob_torque_(float torque);
  void apply_voltage_(float uq, float ud, float theta_e);
  void run_calibration_();
  bool calibration_track_(float &unwrapped);
  float calibration_average_(float &unwrapped, int samples);
  bool calibration_sweep_(float &theta, float target, float step, float &unwrapped);
  void calibration_hold_(float theta, int ms, float &unwrapped);
  void reseed_after_calibration_();
  void publish_(int64_t loop_start_us);

  ControllerConfig config_{};
  MotorCalibration calibration_{};
  DetentEngine engine_{};
  MT6701Reader encoder_{};
  Inverter inverter_{};
  HX711Reader strain_{};
  bool strain_enabled_{false};

  TaskHandle_t task_{nullptr};
  QueueHandle_t commands_{nullptr};
  gptimer_handle_t timer_{nullptr};
  volatile bool start_done_{false};
  const char *start_error_{nullptr};

  portMUX_TYPE state_lock_ = portMUX_INITIALIZER_UNLOCKED;
  ControllerState shared_{};
  volatile bool reset_max_loop_{false};

  // --- Owned by the motor task -------------------------------------------
  bool enabled_{true};
  bool calibration_requested_{false};
  CalibrationState calibration_state_{CalibrationState::IDLE};
  uint32_t calibration_sequence_{0};
  const char *calibration_message_{""};
  float filter_alpha_{0.3f};
  float raw_angle_{0.0f};  // latest unfiltered mechanical angle, 0..2pi
  float filt_x_{1.0f};
  float filt_y_{0.0f};
  float prev_filtered_{0.0f};
  float shaft_{0.0f};  // filtered multi-turn angle in the sensor frame
  bool have_angle_{false};
  float knob_angle_{0.0f};
  float velocity_{0.0f};
  float torque_{0.0f};
  bool sensor_ok_{false};
  uint32_t consecutive_errors_{0};
  uint32_t sensor_errors_{0};
  uint8_t field_status_{0};
  // click
  bool clicking_{false};
  float click_strength_{0.0f};
  uint32_t click_half_us_{0};
  int64_t click_start_us_{0};
  // strain
  bool baseline_ready_{false};
  int64_t baseline_acc_{0};
  uint8_t baseline_count_{0};
  float baseline_{0.0f};
  float strain_value_{0.0f};
  uint8_t over_count_{0};
  bool pressed_{false};
  int64_t pressed_since_us_{0};
  uint32_t press_count_{0};
  uint32_t release_count_{0};
  uint32_t strain_samples_{0};
  int64_t strain_rate_window_us_{0};
  float strain_rate_hz_{0.0f};
  int64_t last_strain_us_{0};
  // stats
  uint32_t loop_count_{0};
  uint32_t max_loop_us_{0};
};

}  // namespace esphome::smartknob

#endif  // USE_ESP32
