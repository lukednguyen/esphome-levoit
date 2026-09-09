#include "air_purifier_vital200s.h"

#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <algorithm>

namespace esphome {
namespace air_purifier_vital200s {

static const char *const TAG = "air_purifier_vital200s";

// =============================================================================
// Entity Implementations
// =============================================================================

void DisplaySwitch::write_state(bool state) { parent_->send_display(state); }

void DisplayLockSwitch::write_state(bool state) { parent_->send_display_lock(state); }

void LightDetectionSwitch::write_state(bool state) { parent_->send_light_detection(state); }

fan::FanTraits PurifierFan::get_traits() {
  fan::FanTraits traits;
  traits.set_speed(true);
  traits.set_supported_speed_count(FAN_SPEED_COUNT);
  traits.set_direction(false);
  traits.set_oscillation(false);
  // preset modes moved to the entity in 2026.4.0
  this->wire_preset_modes_(traits);
  return traits;
}

void PurifierFan::setup() {
  // must precede restore_state_(): FanRestoreState resolves the stored preset
  // index against this list
  this->set_supported_preset_modes({MODE_AUTO, MODE_SLEEP, MODE_MANUAL});

  // restore from flash and apply to MCU
  this->restore_state_();

  if (this->state) {
    ESP_LOGI(TAG, "Restoring fan: ON, mode: %s, speed: %d", this->get_preset_mode().c_str(), this->speed);
    parent_->send_power(true);
    if (this->has_preset_mode()) {
      parent_->send_mode(string_to_mode(this->get_preset_mode().str()));
    }
    if (this->speed > 0) {
      parent_->send_fan_speed(this->speed);
    }
  } else {
    ESP_LOGI(TAG, "Restoring fan: OFF");
    parent_->send_power(false);
  }
}

void PurifierFan::control(const fan::FanCall &call) {
  if (call.get_state().has_value()) {
    parent_->send_power(*call.get_state());
  }

  if (call.has_preset_mode()) {
    parent_->send_mode(string_to_mode(call.get_preset_mode()));
  }

  if (call.get_speed().has_value()) {
    const int speed = *call.get_speed();
    if (speed >= FAN_SPEED_MIN && speed <= FAN_SPEED_MAX) {
      parent_->send_mode(Mode::MANUAL);
      parent_->send_fan_speed(static_cast<uint8_t>(speed));
    }
  }
}

void PurifierFan::publish_mode(Mode mode) {
  this->set_preset_mode_(mode_to_string(mode));
  this->publish_state();
}

// =============================================================================
// Component Lifecycle
// =============================================================================

void AirPurifier::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Air Purifier...");
  rx_buffer_.reserve(RX_BUFFER_MAX);

  if (fan_ != nullptr) {
    fan_->setup();
  }
}

void AirPurifier::loop() { read_uart_(); }

void AirPurifier::update() { send_ping_(); }

void AirPurifier::dump_config() {
  ESP_LOGCONFIG(TAG, "Air Purifier:");
  ESP_LOGCONFIG(TAG, "  Update interval: %.1fs", get_update_interval() / 1000.0f);
  LOG_SENSOR("  ", "PM2.5", pm25_sensor_);
  LOG_TEXT_SENSOR("  ", "Air Quality", air_quality_sensor_);
  if (fan_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Fan: %s", fan_->get_name().c_str());
  }
  LOG_SWITCH("  ", "Display", display_switch_);
  LOG_SWITCH("  ", "Display Lock", display_lock_switch_);
  LOG_SWITCH("  ", "Light Detection", light_detection_switch_);
}

// =============================================================================
// UART Communication
// =============================================================================

void AirPurifier::read_uart_() {
  const uint32_t now = millis();

  // drop stale data
  if (!rx_buffer_.empty() && (now - last_rx_time_) > RX_TIMEOUT_MS) {
    rx_buffer_.clear();
  }

  while (available()) {
    uint8_t byte;
    read_byte(&byte);
    last_rx_time_ = now;

    // sync to header
    if (rx_buffer_.empty() && byte != PACKET_HEADER) {
      continue;
    }

    rx_buffer_.push_back(byte);

    // wait for minimum header
    if (rx_buffer_.size() < RX_MIN_HEADER_LEN) {
      continue;
    }

    const auto type = static_cast<PacketType>(rx_buffer_[static_cast<size_t>(Offset::TYPE)]);
    const uint8_t payload_len = rx_buffer_[static_cast<size_t>(Offset::PAYLOAD_LEN)];

    size_t expected_size;
    switch (type) {
      case PacketType::STATUS:
        expected_size = RX_MIN_HEADER_LEN + payload_len;
        break;
      case PacketType::PING:
        expected_size = 10;  // PING frames are fixed-length
        break;
      default:
        rx_buffer_.clear();
        continue;
    }

    // wait for complete packet
    if (rx_buffer_.size() < expected_size) {
      continue;
    }

    if (type == PacketType::STATUS) {
      const uint8_t received_checksum = rx_buffer_[static_cast<size_t>(Offset::CHECKSUM)];
      const uint8_t calculated_checksum = calc_checksum_(rx_buffer_.data(), expected_size);

      // MCU outbound checksum formula unverified on hardware: warn but still parse
      if (received_checksum != calculated_checksum) {
        ESP_LOGW(TAG, "Checksum mismatch: 0x%02X != 0x%02X", received_checksum, calculated_checksum);
      }
      parse_packet_(rx_buffer_.data(), expected_size);
    }

    rx_buffer_.clear();
  }

  // prevent overflow
  if (rx_buffer_.size() > RX_BUFFER_MAX) {
    rx_buffer_.clear();
  }
}

void AirPurifier::send_ping_() {
  // clang-format off
  uint8_t packet[] = {
      PACKET_HEADER, static_cast<uint8_t>(PacketType::PING),  // header, packet type
      seq_++, static_cast<uint8_t>(PayloadLen::PING),         // sequence, payload length
      0x00, 0x00,                                             // reserved, checksum placeholder
      ADDR_STATUS[0], ADDR_STATUS[1], ADDR_STATUS[2], ADDR_STATUS[3],
  };
  // clang-format on

  packet[static_cast<size_t>(Offset::CHECKSUM)] = calc_checksum_(packet, sizeof(packet));
  write_array(packet, sizeof(packet));
}

void AirPurifier::send_command_(const Address &addr, uint8_t value) {
  // clang-format off
  uint8_t packet[] = {
      PACKET_HEADER, static_cast<uint8_t>(PacketType::STATUS),  // header, packet type
      seq_++, static_cast<uint8_t>(PayloadLen::COMMAND),        // sequence, payload length
      0x00, 0x00,                                               // reserved, checksum placeholder
      addr[0], addr[1], addr[2], addr[3],                       // target address
      0x01, 0x01, value,                                        // TLV: type, length, value
  };
  // clang-format on

  packet[static_cast<size_t>(Offset::CHECKSUM)] = calc_checksum_(packet, sizeof(packet));
  write_array(packet, sizeof(packet));
  ESP_LOGV(TAG, "TX: %s", format_hex_pretty(packet, sizeof(packet)).c_str());
}

bool AirPurifier::match_address_(const uint8_t *data, const Address &addr) {
  const size_t addr_offset = static_cast<size_t>(Offset::ADDR_START);
  return std::equal(data + addr_offset, data + addr_offset + addr.size(), addr.begin());
}

uint8_t AirPurifier::calc_checksum_(const uint8_t *data, size_t len) {
  uint8_t sum = 0;
  for (size_t i = 0; i < len; i++) {
    if (i != static_cast<size_t>(Offset::SEQ) && i != static_cast<size_t>(Offset::CHECKSUM)) {
      sum += data[i];
    }
  }
  return 0xFF - sum - data[static_cast<size_t>(Offset::SEQ)];
}

// =============================================================================
// Commands
// =============================================================================

void AirPurifier::send_power(bool on) {
  ESP_LOGI(TAG, "Power: %s", on ? "ON" : "OFF");
  send_command_(ADDR_POWER, on ? VALUE_ON : VALUE_OFF);
}

void AirPurifier::send_mode(Mode mode) {
  ESP_LOGI(TAG, "Mode: %s", mode_to_string(mode));
  send_command_(ADDR_MODE, static_cast<uint8_t>(mode));
}

void AirPurifier::send_fan_speed(uint8_t speed) {
  speed = std::clamp(speed, FAN_SPEED_MIN, FAN_SPEED_MAX);
  ESP_LOGI(TAG, "Fan speed: %d", speed);
  send_command_(ADDR_MANUAL_SPEED, speed);
}

void AirPurifier::send_display(bool on) {
  ESP_LOGI(TAG, "Display: %s", on ? "ON" : "OFF");
  const auto brightness = on ? DisplayBrightness::ON : DisplayBrightness::OFF;
  send_command_(ADDR_DISPLAY, static_cast<uint8_t>(brightness));
}

void AirPurifier::send_display_lock(bool on) {
  ESP_LOGI(TAG, "Display lock: %s", on ? "ON" : "OFF");
  send_command_(ADDR_DISPLAY_LOCK, on ? VALUE_ON : VALUE_OFF);
}

void AirPurifier::send_light_detection(bool on) {
  ESP_LOGI(TAG, "Light detection: %s", on ? "ON" : "OFF");
  send_command_(ADDR_LIGHT_DETECTION, on ? VALUE_ON : VALUE_OFF);
}

void AirPurifier::send_wifi_status(bool ha_connected, bool wifi_connected) {
  WifiLedStatus status;
  const char *status_str;

  if (ha_connected) {
    status = WifiLedStatus::SOLID;
    status_str = "solid (HA connected)";
  } else if (wifi_connected) {
    status = WifiLedStatus::BLINKING;
    status_str = "blinking (WiFi only)";
  } else {
    status = WifiLedStatus::OFF;
    status_str = "off (disconnected)";
  }

  ESP_LOGI(TAG, "WiFi LED: %s", status_str);

  constexpr uint8_t blink_lo = WIFI_BLINK_MS & 0xFF;
  constexpr uint8_t blink_hi = (WIFI_BLINK_MS >> 8) & 0xFF;

  // clang-format off
  uint8_t packet[] = {
      PACKET_HEADER, static_cast<uint8_t>(PacketType::STATUS),  // header, packet type
      seq_++, static_cast<uint8_t>(PayloadLen::WIFI_LED),       // sequence, payload length
      0x00, 0x00,                                               // reserved, checksum placeholder
      ADDR_WIFI_LED[0], ADDR_WIFI_LED[1], ADDR_WIFI_LED[2], ADDR_WIFI_LED[3],
      static_cast<uint8_t>(WifiLedTLV::STATUS), 0x01, static_cast<uint8_t>(status),
      static_cast<uint8_t>(WifiLedTLV::BLINK_ON), 0x02, blink_lo, blink_hi,
      static_cast<uint8_t>(WifiLedTLV::BLINK_OFF), 0x02, blink_lo, blink_hi,
      static_cast<uint8_t>(WifiLedTLV::RESET_FLAG), 0x01, 0x00,
  };
  // clang-format on

  packet[static_cast<size_t>(Offset::CHECKSUM)] = calc_checksum_(packet, sizeof(packet));
  write_array(packet, sizeof(packet));
  ESP_LOGD(TAG, "TX WiFi LED: %s", format_hex_pretty(packet, sizeof(packet)).c_str());
}

// =============================================================================
// Timer
// =============================================================================

void AirPurifier::send_timer_cancel_() {
  ESP_LOGI(TAG, "Cancelling timer");

  // clang-format off
  uint8_t packet[] = {
      PACKET_HEADER, static_cast<uint8_t>(PacketType::STATUS),   // header, packet type
      seq_++, static_cast<uint8_t>(PayloadLen::TIMER_CANCEL),    // sequence, payload length
      0x00, 0x00,                                                // reserved, checksum placeholder
      ADDR_TIMER_SET[0], ADDR_TIMER_SET[1], ADDR_TIMER_SET[2], ADDR_TIMER_SET[3],
      0x01, 0x04, 0x00, 0x00, 0x00, 0x00,                        // TLV: type, length, 4-byte value
  };
  // clang-format on

  packet[static_cast<size_t>(Offset::CHECKSUM)] = calc_checksum_(packet, sizeof(packet));
  write_array(packet, sizeof(packet));
  ESP_LOGD(TAG, "TX Timer cancel: %s", format_hex_pretty(packet, sizeof(packet)).c_str());
}

void AirPurifier::handle_timer_tlv_(uint8_t type, uint8_t len, const uint8_t *value) {
  if (len < 4) {
    ESP_LOGW(TAG, "Timer TLV 0x%02X too short: %d bytes", type, len);
    return;
  }

  const uint32_t seconds = static_cast<uint32_t>(value[0]) | (static_cast<uint32_t>(value[1]) << 8) |
                           (static_cast<uint32_t>(value[2]) << 16) | (static_cast<uint32_t>(value[3]) << 24);

  switch (static_cast<TimerTLV>(type)) {
    case TimerTLV::REMAINING:
      ESP_LOGV(TAG, "Timer remaining: %lu seconds", static_cast<unsigned long>(seconds));
      break;

    case TimerTLV::TOTAL:
      if (seconds > 0) {
        ESP_LOGW(TAG, "Timer %lu seconds detected, cancelling...", static_cast<unsigned long>(seconds));
        send_timer_cancel_();
      }
      break;

    default:
      ESP_LOGV(TAG, "Unknown Timer TLV 0x%02X: %s", type, format_hex_pretty(value, len).c_str());
      break;
  }
}

// =============================================================================
// Packet Parsing
// =============================================================================

void AirPurifier::parse_packet_(const uint8_t *data, size_t len) {
  if (len < RX_MIN_PACKET_LEN) {
    return;
  }

  if (match_address_(data, ADDR_STATUS) && len >= static_cast<size_t>(Offset::TLV_START_STATUS)) {
    parse_tlvs_(data, len, static_cast<size_t>(Offset::TLV_START_STATUS),
                [this](uint8_t t, uint8_t l, const uint8_t *v) { handle_status_tlv_(t, l, v); });
  } else if (match_address_(data, ADDR_TIMER) && len >= static_cast<size_t>(Offset::TLV_START_TIMER)) {
    parse_tlvs_(data, len, static_cast<size_t>(Offset::TLV_START_TIMER),
                [this](uint8_t t, uint8_t l, const uint8_t *v) { handle_timer_tlv_(t, l, v); });
  }
}

template <typename Handler>
void AirPurifier::parse_tlvs_(const uint8_t *data, size_t len, size_t start, Handler handler) {
  size_t pos = start;
  while (pos + 2 <= len) {
    const uint8_t type = data[pos];
    const uint8_t tlv_len = data[pos + 1];
    if (pos + 2 + tlv_len > len) break;
    if (tlv_len > 0) {
      handler(type, tlv_len, &data[pos + 2]);
    }
    pos += 2 + tlv_len;
  }
}

void AirPurifier::handle_status_tlv_(uint8_t type, uint8_t len, const uint8_t *value) {
  if (len == 0) {
    return;
  }

  const uint8_t v = value[0];

  switch (static_cast<TLV>(type)) {
    case TLV::POWER:
      if (fan_ != nullptr) {
        fan_->state = (v != VALUE_OFF);
        fan_->publish_state();
      }
      ESP_LOGD(TAG, "Power: %s", v != VALUE_OFF ? "ON" : "OFF");
      break;

    case TLV::MODE:
      last_mode_ = static_cast<Mode>(v);
      if (fan_ != nullptr) fan_->publish_mode(last_mode_);
      ESP_LOGD(TAG, "Mode: %s", mode_to_string(last_mode_));
      break;

    case TLV::SPEED:
      if (v >= FAN_SPEED_MIN && v <= FAN_SPEED_MAX) {
        if (fan_ != nullptr) {
          fan_->speed = v;
          fan_->publish_state();
        }
        ESP_LOGD(TAG, "Speed: %d", v);
      }
      break;

    case TLV::DISPLAY_CURRENT:
      ESP_LOGV(TAG, "Display current: %s", v != VALUE_OFF ? "ON" : "OFF");
      break;

    case TLV::DISPLAY_SAVED:
      if (display_switch_ != nullptr) {
        display_switch_->publish_state(v != VALUE_OFF);
      }
      ESP_LOGD(TAG, "Display: %s", v != VALUE_OFF ? "ON" : "OFF");
      break;

    case TLV::AIR_QUALITY:
      if (air_quality_sensor_ != nullptr) {
        const char *quality = air_quality_to_string(v);
        air_quality_sensor_->publish_state(quality);
        ESP_LOGD(TAG, "Air quality: %s", quality);
      }
      break;

    case TLV::PM25:
      if (pm25_sensor_ != nullptr && len >= 2) {
        const uint16_t pm25 = value[0] | (value[1] << 8);
        pm25_sensor_->publish_state(pm25);
        ESP_LOGD(TAG, "PM2.5: %d µg/m³", pm25);
      }
      break;

    case TLV::DISPLAY_LOCK:
      if (display_lock_switch_ != nullptr) {
        display_lock_switch_->publish_state(v != VALUE_OFF);
      }
      ESP_LOGD(TAG, "Display lock: %s", v != VALUE_OFF ? "ON" : "OFF");
      break;

    case TLV::LIGHT_DETECTION:
      if (light_detection_switch_ != nullptr) {
        light_detection_switch_->publish_state(v != VALUE_OFF);
      }
      ESP_LOGD(TAG, "Light detection: %s", v != VALUE_OFF ? "ON" : "OFF");
      break;

    default:
      ESP_LOGV(TAG, "Unknown TLV 0x%02X: %s", type, format_hex_pretty(value, len).c_str());
      break;
  }
}

}  // namespace air_purifier_vital200s
}  // namespace esphome