#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/fan/fan.h"
#include "esphome/components/switch/switch.h"
// #include "esphome/components/button/button.h"  // filter reset not used
#include "types.h"

namespace esphome {
namespace air_purifier_vital200s {

class AirPurifier;

// =============================================================================
// Entity Classes
// =============================================================================

class DisplaySwitch : public switch_::Switch, public Parented<AirPurifier> {
  void write_state(bool state) override;
};

class DisplayLockSwitch : public switch_::Switch, public Parented<AirPurifier> {
  void write_state(bool state) override;
};

class LightDetectionSwitch : public switch_::Switch, public Parented<AirPurifier> {
  void write_state(bool state) override;
};

// Filter reset (filter life not readable from MCU)
// class FilterResetButton : public button::Button, public Parented<AirPurifierComponent> {
//   void press_action() override;
// };

class PurifierFan : public fan::Fan, public Parented<AirPurifier> {
 public:
  fan::FanTraits get_traits() override;
  void control(const fan::FanCall &call) override;
  void setup();
};

// =============================================================================
// Main Component
// =============================================================================

class AirPurifier : public PollingComponent, public uart::UARTDevice {
 public:
  // Component overrides
  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // Entity setters
  void set_pm25_sensor(sensor::Sensor *s) { pm25_sensor_ = s; }
  void set_air_quality_sensor(text_sensor::TextSensor *s) { air_quality_sensor_ = s; }

  void set_fan(PurifierFan *f) {
    f->set_parent(this);
    fan_ = f;
  }
  void set_display_switch(DisplaySwitch *s) {
    s->set_parent(this);
    display_switch_ = s;
  }
  void set_display_lock_switch(DisplayLockSwitch *s) {
    s->set_parent(this);
    display_lock_switch_ = s;
  }
  void set_light_detection_switch(LightDetectionSwitch *s) {
    s->set_parent(this);
    light_detection_switch_ = s;
  }

  // Filter reset (filter life not readable from MCU)
  // void set_filter_reset_button(FilterResetButton *b) {
  //   b->set_parent(this);
  //   filter_reset_button_ = b;
  // }

  // Commands
  void send_power(bool on);
  void send_mode(Mode mode);
  void send_fan_speed(uint8_t speed);
  void send_display(bool on);
  void send_display_lock(bool on);
  void send_light_detection(bool on);
  void send_wifi_status(bool ha_connected, bool wifi_connected);

  // Filter reset (filter life not readable from MCU)
  // void send_filter_reset();

 private:
  // UART
  void read_uart_();
  void send_ping_();
  void send_command_(const Address &addr, uint8_t value);
  bool match_address_(const uint8_t *data, const Address &addr);
  uint8_t calc_checksum_(const uint8_t *data, size_t len);

  // Timer
  void send_timer_cancel_();
  void handle_timer_tlv_(uint8_t type, uint8_t len, const uint8_t *value);

  // Parsing
  void parse_packet_(const uint8_t *data, size_t len);

  template<typename Handler>
  void parse_tlvs_(const uint8_t *data, size_t len, size_t start, Handler handler);

  void handle_status_tlv_(uint8_t type, uint8_t len, const uint8_t *value);

  // Entities
  sensor::Sensor *pm25_sensor_{nullptr};
  text_sensor::TextSensor *air_quality_sensor_{nullptr};
  PurifierFan *fan_{nullptr};
  DisplaySwitch *display_switch_{nullptr};
  DisplayLockSwitch *display_lock_switch_{nullptr};
  LightDetectionSwitch *light_detection_switch_{nullptr};

  // Filter reset (filter life not readable from MCU)
  // FilterResetButton *filter_reset_button_{nullptr};

  // State
  std::vector<uint8_t> rx_buffer_;
  uint32_t last_rx_time_{0};
  uint8_t seq_{0};
  Mode last_mode_{Mode::AUTO};
};

}  // namespace air_purifier_vital200s
}  // namespace esphome