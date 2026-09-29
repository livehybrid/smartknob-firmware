#include "hardware.h"

#ifdef USE_ESP32

#include <driver/gpio.h>
#include <esp_rom_sys.h>
#include <freertos/FreeRTOS.h>

namespace esphome::smartknob {

// ---------------------------------------------------------------- MT6701 ----

esp_err_t MT6701Reader::begin(spi_host_device_t host, int clk_pin, int data_pin, int cs_pin, uint32_t clock_hz) {
  spi_bus_config_t bus{};
  bus.mosi_io_num = -1;
  bus.miso_io_num = data_pin;
  bus.sclk_io_num = clk_pin;
  bus.quadwp_io_num = -1;
  bus.quadhd_io_num = -1;
  bus.max_transfer_sz = 0;
  esp_err_t err = spi_bus_initialize(host, &bus, SPI_DMA_DISABLED);
  if (err != ESP_OK)
    return err;

  spi_device_interface_config_t dev{};
  dev.mode = 1;  // SSI: data valid on the falling clock edge
  dev.clock_speed_hz = static_cast<int>(clock_hz);
  dev.spics_io_num = cs_pin;
  dev.cs_ena_pretrans = 4;  // CSN low well before the first clock (as the original firmware)
  dev.queue_size = 1;
  err = spi_bus_add_device(host, &dev, &this->device_);
  if (err != ESP_OK)
    return err;
  // The bus is dedicated to the sensor: hold it so polling transfers are as fast as possible.
  err = spi_device_acquire_bus(this->device_, portMAX_DELAY);
  if (err != ESP_OK)
    return err;

  this->transaction_.flags = SPI_TRANS_USE_RXDATA;
  this->transaction_.length = 24;
  this->transaction_.rxlength = 24;
  return ESP_OK;
}

bool MT6701Reader::read(foc::MT6701Frame &frame) {
  if (this->device_ == nullptr || spi_device_polling_transmit(this->device_, &this->transaction_) != ESP_OK)
    return false;
  frame = foc::mt6701_decode(this->transaction_.rx_data[0], this->transaction_.rx_data[1],
                             this->transaction_.rx_data[2]);
  return true;
}

// -------------------------------------------------------------- Inverter ----

static constexpr uint32_t INVERTER_RESOLUTION_HZ = 40000000;  // 25 ns ticks

esp_err_t Inverter::begin(const InverterPins &pins, uint32_t pwm_frequency, uint32_t dead_time_ns) {
  mcpwm_timer_config_t timer_config{};
  timer_config.group_id = 0;
  timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
  timer_config.resolution_hz = INVERTER_RESOLUTION_HZ;
  timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN;  // centre-aligned PWM
  timer_config.period_ticks = INVERTER_RESOLUTION_HZ / pwm_frequency;
  esp_err_t err = mcpwm_new_timer(&timer_config, &this->timer_);
  if (err != ESP_OK)
    return err;
  this->peak_ticks_ = timer_config.period_ticks / 2;

  mcpwm_operator_config_t operator_config{};
  operator_config.group_id = 0;
  mcpwm_comparator_config_t comparator_config{};
  comparator_config.flags.update_cmp_on_tez = true;  // glitch-free duty updates
  const int gpios[3][2] = {{pins.uh, pins.ul}, {pins.vh, pins.vl}, {pins.wh, pins.wl}};
  const uint32_t dead_ticks =
      static_cast<uint32_t>((static_cast<uint64_t>(dead_time_ns) * INVERTER_RESOLUTION_HZ) / 1000000000ULL);

  for (int i = 0; i < 3; i++) {
    if ((err = mcpwm_new_operator(&operator_config, &this->operators_[i])) != ESP_OK)
      return err;
    if ((err = mcpwm_operator_connect_timer(this->operators_[i], this->timer_)) != ESP_OK)
      return err;
    if ((err = mcpwm_new_comparator(this->operators_[i], &comparator_config, &this->comparators_[i])) != ESP_OK)
      return err;
    mcpwm_comparator_set_compare_value(this->comparators_[i], this->peak_ticks_ / 2);

    for (int j = 0; j < 2; j++) {
      mcpwm_generator_config_t gen_config{};
      gen_config.gen_gpio_num = gpios[i][j];
      if ((err = mcpwm_new_generator(this->operators_[i], &gen_config, &this->generators_[i][j])) != ESP_OK)
        return err;
      // Both generators produce the same waveform (high while the counter is
      // below the compare value); the dead-time stage makes them complementary.
      mcpwm_gen_compare_event_action_t up{};
      up.direction = MCPWM_TIMER_DIRECTION_UP;
      up.comparator = this->comparators_[i];
      up.action = MCPWM_GEN_ACTION_LOW;
      mcpwm_gen_compare_event_action_t down{};
      down.direction = MCPWM_TIMER_DIRECTION_DOWN;
      down.comparator = this->comparators_[i];
      down.action = MCPWM_GEN_ACTION_HIGH;
      if ((err = mcpwm_generator_set_action_on_compare_event(this->generators_[i][j], up)) != ESP_OK)
        return err;
      if ((err = mcpwm_generator_set_action_on_compare_event(this->generators_[i][j], down)) != ESP_OK)
        return err;
    }
    // High side: delay the rising edge. Low side: delay the falling edge, then
    // invert. Each half-bridge therefore never has both switches on.
    mcpwm_dead_time_config_t high_side{};
    high_side.posedge_delay_ticks = dead_ticks;
    mcpwm_dead_time_config_t low_side{};
    low_side.negedge_delay_ticks = dead_ticks;
    low_side.flags.invert_output = true;
    if ((err = mcpwm_generator_set_dead_time(this->generators_[i][0], this->generators_[i][0], &high_side)) != ESP_OK)
      return err;
    if ((err = mcpwm_generator_set_dead_time(this->generators_[i][1], this->generators_[i][1], &low_side)) != ESP_OK)
      return err;
  }

  this->enabled_ = true;  // force set_enabled(false) to apply
  this->set_enabled(false);
  if ((err = mcpwm_timer_enable(this->timer_)) != ESP_OK)
    return err;
  return mcpwm_timer_start_stop(this->timer_, MCPWM_TIMER_START_NO_STOP);
}

void Inverter::set_duty(float a, float b, float c) {
  const float duty[3] = {a, b, c};
  for (int i = 0; i < 3; i++) {
    float d = duty[i] < 0.0f ? 0.0f : (duty[i] > 1.0f ? 1.0f : duty[i]);
    mcpwm_comparator_set_compare_value(this->comparators_[i],
                                       static_cast<uint32_t>(d * static_cast<float>(this->peak_ticks_)));
  }
}

void Inverter::set_enabled(bool enabled) {
  if (enabled == this->enabled_)
    return;
  for (int i = 0; i < 3; i++) {
    if (enabled) {
      // Release the forced levels: complementary PWM resumes.
      mcpwm_generator_set_force_level(this->generators_[i][0], -1, true);
      mcpwm_generator_set_force_level(this->generators_[i][1], -1, true);
    } else {
      // High side 0; low side forced 1 is inverted by its dead-time stage to 0.
      mcpwm_generator_set_force_level(this->generators_[i][0], 0, true);
      mcpwm_generator_set_force_level(this->generators_[i][1], 1, true);
    }
  }
  this->enabled_ = enabled;
}

// ----------------------------------------------------------------- HX711 ----

void HX711Reader::begin(int dout_pin, int sck_pin) {
  this->dout_pin_ = dout_pin;
  this->sck_pin_ = sck_pin;
  gpio_config_t io{};
  io.pin_bit_mask = 1ULL << dout_pin;
  io.mode = GPIO_MODE_INPUT;
  gpio_config(&io);
  io.pin_bit_mask = 1ULL << sck_pin;
  io.mode = GPIO_MODE_OUTPUT;
  gpio_config(&io);
  gpio_set_level(static_cast<gpio_num_t>(sck_pin), 0);  // SCK low = powered up
}

bool HX711Reader::ready() const {
  return this->dout_pin_ >= 0 && gpio_get_level(static_cast<gpio_num_t>(this->dout_pin_)) == 0;
}

int32_t HX711Reader::read() {
  static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  const auto sck = static_cast<gpio_num_t>(this->sck_pin_);
  const auto dout = static_cast<gpio_num_t>(this->dout_pin_);
  uint32_t value = 0;
  // SCK must not stay high for more than 60 us or the HX711 powers down, so
  // keep interrupts off for the ~50 us transfer.
  portENTER_CRITICAL(&mux);
  for (int i = 0; i < 24; i++) {
    gpio_set_level(sck, 1);
    esp_rom_delay_us(1);
    value = (value << 1) | static_cast<uint32_t>(gpio_get_level(dout));
    gpio_set_level(sck, 0);
    esp_rom_delay_us(1);
  }
  // 25th pulse selects channel A, gain 128 for the next conversion.
  gpio_set_level(sck, 1);
  esp_rom_delay_us(1);
  gpio_set_level(sck, 0);
  esp_rom_delay_us(1);
  portEXIT_CRITICAL(&mux);
  if (value & 0x800000)
    value |= 0xFF000000;  // sign-extend 24-bit two's complement
  return static_cast<int32_t>(value);
}

}  // namespace esphome::smartknob

#endif  // USE_ESP32
