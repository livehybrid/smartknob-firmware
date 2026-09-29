#pragma once

// Low-level drivers for the SmartKnob Dev Kit, written against ESP-IDF 5.x:
//  * MT6701 magnetic angle sensor over SSI (SPI mode 1, 24-bit frames)
//  * three-phase inverter on MCPWM with complementary outputs and dead time
//    (TMC6300 gate inputs: UH/UL, VH/VL, WH/WL)
//  * HX711 strain-gauge ADC (bit-banged), used for press detection
// All of these are used only from the real-time motor task.

#ifdef USE_ESP32

#include <cstdint>

#include <driver/mcpwm_prelude.h>
#include <driver/spi_master.h>

#include "foc_math.h"

namespace esphome::smartknob {

class MT6701Reader {
 public:
  esp_err_t begin(spi_host_device_t host, int clk_pin, int data_pin, int cs_pin, uint32_t clock_hz);
  // Reads one frame. Returns false if the SPI transfer itself failed.
  bool read(foc::MT6701Frame &frame);

 protected:
  spi_device_handle_t device_{nullptr};
  spi_transaction_t transaction_{};
};

struct InverterPins {
  int uh, ul, vh, vl, wh, wl;
};

class Inverter {
 public:
  esp_err_t begin(const InverterPins &pins, uint32_t pwm_frequency, uint32_t dead_time_ns);
  // Duty cycles 0..1 per phase (centre-aligned PWM).
  void set_duty(float a, float b, float c);
  // Enabled: complementary PWM. Disabled: all six switches off (motor coasts).
  void set_enabled(bool enabled);
  bool enabled() const { return this->enabled_; }

 protected:
  mcpwm_timer_handle_t timer_{nullptr};
  mcpwm_oper_handle_t operators_[3]{};
  mcpwm_cmpr_handle_t comparators_[3]{};
  mcpwm_gen_handle_t generators_[3][2]{};
  uint32_t peak_ticks_{0};
  bool enabled_{false};
};

class HX711Reader {
 public:
  void begin(int dout_pin, int sck_pin);
  bool ready() const;
  // Reads a conversion (channel A, gain 128 for the next one). Only call when ready().
  int32_t read();

 protected:
  int dout_pin_{-1};
  int sck_pin_{-1};
};

}  // namespace esphome::smartknob

#endif  // USE_ESP32
