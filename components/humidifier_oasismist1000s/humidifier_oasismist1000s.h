#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/select/select.h"
#include "esphome/components/number/number.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "types.h"

namespace esphome {
namespace humidifier_oasismist1000s {

class Humidifier;

// =============================================================================
// Entity Classes
// =============================================================================

class PowerSwitch : public switch_::Switch, public Parented<Humidifier> {
  void write_state(bool state) override;
};

class DisplaySwitch : public switch_::Switch, public Parented<Humidifier> {
  void write_state(bool state) override;
};

class ModeSelect : public select::Select, public Parented<Humidifier> {
  void control(const std::string &value) override;
};

class TargetHumidityNumber : public number::Number, public Parented<Humidifier> {
  void control(float value) override;
};

class MistLevelNumber : public number::Number, public Parented<Humidifier> {
  void control(float value) override;
};

// =============================================================================
// Main Component
// =============================================================================

class Humidifier : public PollingComponent, public uart::UARTDevice {
 public:
  // Component overrides
  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // Entity setters
  void set_humidity_sensor(sensor::Sensor *s) { humidity_sensor_ = s; }
  void set_reservoir_sensor(binary_sensor::BinarySensor *s) { reservoir_sensor_ = s; }
  void set_water_sensor(binary_sensor::BinarySensor *s) { water_sensor_ = s; }
  void set_misting_sensor(binary_sensor::BinarySensor *s) { misting_sensor_ = s; }

  void set_power_switch(PowerSwitch *s) {
    s->set_parent(this);
    power_switch_ = s;
  }
  void set_display_switch(DisplaySwitch *s) {
    s->set_parent(this);
    display_switch_ = s;
  }
  void set_mode_select(ModeSelect *s) {
    s->set_parent(this);
    mode_select_ = s;
  }
  void set_target_humidity_number(TargetHumidityNumber *n) {
    n->set_parent(this);
    target_humidity_number_ = n;
  }
  void set_mist_level_number(MistLevelNumber *n) {
    n->set_parent(this);
    mist_level_number_ = n;
  }

  // Commands
  void send_power(bool on);
  void send_display(bool on);
  void send_mode(Mode mode);
  void send_mist_level(uint8_t level, bool auto_switch_mode = false);
  void send_target_humidity(uint8_t humidity, bool auto_switch_mode = false); 
  void send_wifi_status(bool ha_connected, bool wifi_connected);

  // Helpers
  static const char *mode_to_string(Mode mode);
  static Mode string_to_mode(const std::string &str);

 private:
  // UART
  void read_uart_();
  void send_ping_();
  void send_command_(const Address &addr, uint8_t value);
  uint8_t calculate_checksum_(const uint8_t *data, size_t len);

  // Address matching
  bool match_address_(const uint8_t *data, const Address &addr);

  // Parsing
  void parse_packet_(const uint8_t *data, size_t len);

  template<typename Handler>
  void parse_tlvs_(const uint8_t *data, size_t len, Handler handler);

  void handle_status_tlv_(uint8_t type, uint8_t len, const uint8_t *value);
  void handle_wifi_tlv_(uint8_t type, uint8_t len, const uint8_t *value);

  // Helpers
  void invalidate_diagnostic_sensors_();
  
  // Entities
  sensor::Sensor *humidity_sensor_{nullptr};
  binary_sensor::BinarySensor *reservoir_sensor_{nullptr};
  binary_sensor::BinarySensor *water_sensor_{nullptr};
  binary_sensor::BinarySensor *misting_sensor_{nullptr};
  PowerSwitch *power_switch_{nullptr};
  DisplaySwitch *display_switch_{nullptr};
  ModeSelect *mode_select_{nullptr};
  TargetHumidityNumber *target_humidity_number_{nullptr};
  MistLevelNumber *mist_level_number_{nullptr};
  
  // State
  std::vector<uint8_t> rx_buffer_;
  uint32_t last_rx_time_{0};
  uint8_t seq_{0};
  Mode last_mode_{Mode::AUTO};
  bool power_on_{false};
};

}  // namespace levoit_humidifier
}  // namespace esphome