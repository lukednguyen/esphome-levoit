#pragma once

#include <vector>

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/fan/fan.h"
#include "esphome/components/number/number.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "types.h"

namespace esphome {
namespace humidifier_oasismist1000s {

class Humidifier;

// =============================================================================
// Entity Classes
// =============================================================================

class HumidifierFan : public fan::Fan, public Parented<Humidifier> {
 public:
  fan::FanTraits get_traits() override;
  void control(const fan::FanCall &call) override;

  // called manually from Humidifier::setup(), not a Component override.
  // registers preset modes only; device is authoritative, so nothing is
  // restored or sent at boot (MCU status packets populate this within one poll)
  void setup();

  // called from RX path; MCU is source of truth for mode
  void publish_mode(Mode mode);
};

class DisplaySwitch : public switch_::Switch, public Parented<Humidifier> {
  void write_state(bool state) override;
};

class TargetHumidityNumber : public number::Number, public Parented<Humidifier> {
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

  void set_fan(HumidifierFan *f) {
    f->set_parent(this);
    fan_ = f;
  }
  void set_display_switch(DisplaySwitch *s) {
    s->set_parent(this);
    display_switch_ = s;
  }
  void set_target_humidity_number(TargetHumidityNumber *n) {
    n->set_parent(this);
    target_humidity_number_ = n;
  }

  // Options
  void set_wifi_status_led(bool enable) { wifi_status_led_ = enable; }

  // Commands
  void send_power(bool on);
  void send_display(bool on);
  void send_mode(Mode mode);
  void send_mist_level(uint8_t level, bool auto_switch_mode = false);
  void send_target_humidity(uint8_t humidity, bool auto_switch_mode = false);
  void send_wifi_status(bool ha_connected, bool wifi_connected);

 private:
  // UART
  void read_uart_();
  void send_ping_();
  void send_command_(const Address &addr, uint8_t value);
  bool match_address_(const uint8_t *data, const Address &addr);
  uint8_t calc_checksum_(const uint8_t *data, size_t len);

  // Parsing
  void parse_packet_(const uint8_t *data, size_t len);

  template <typename Handler>
  void parse_tlvs_(const uint8_t *data, size_t len, Handler handler);

  void handle_status_tlv_(uint8_t type, uint8_t len, const uint8_t *value);

  // Helpers
  void invalidate_diagnostic_sensors_();

  // Entities
  sensor::Sensor *humidity_sensor_{nullptr};
  binary_sensor::BinarySensor *reservoir_sensor_{nullptr};
  binary_sensor::BinarySensor *water_sensor_{nullptr};
  binary_sensor::BinarySensor *misting_sensor_{nullptr};
  HumidifierFan *fan_{nullptr};
  DisplaySwitch *display_switch_{nullptr};
  TargetHumidityNumber *target_humidity_number_{nullptr};

  // State
  std::vector<uint8_t> rx_buffer_;
  uint32_t last_rx_time_{0};
  uint8_t seq_{0};
  Mode last_mode_{Mode::AUTO};
  bool power_on_{false};
  bool wifi_status_led_{false};
};

}  // namespace humidifier_oasismist1000s
}  // namespace esphome