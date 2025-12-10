#include "humidifier_oasismist1000s.h"

#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <algorithm>
#include <cstring>

namespace esphome {
namespace humidifier_oasismist1000s {

static const char *TAG = "humidifier_oasismist1000s";

// =============================================================================
// Entity Implementations
// =============================================================================

void PowerSwitch::write_state(bool state) {
  parent_->send_power(state);
}

void DisplaySwitch::write_state(bool state) {
  parent_->send_display(state);
}

void ModeSelect::control(const std::string &value) {
  parent_->send_mode(Humidifier::string_to_mode(value));
}

void TargetHumidityNumber::control(float value) {
  parent_->send_target_humidity(static_cast<uint8_t>(value), true);
}

void MistLevelNumber::control(float value) {
  parent_->send_mist_level(static_cast<uint8_t>(value), true);
}

// =============================================================================
// Component Lifecycle
// =============================================================================

void Humidifier::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Levoit Humidifier...");
  rx_buffer_.reserve(RX_BUFFER_MAX);
  invalidate_diagnostic_sensors_();
}

void Humidifier::loop() {
  read_uart_();
}

void Humidifier::update() {
  send_ping_();
}

void Humidifier::dump_config() {
  ESP_LOGCONFIG(TAG, "Humidifier OasisMist 1000S:");
  ESP_LOGCONFIG(TAG, "  Update interval: %.1fs", get_update_interval() / 1000.0f);
  LOG_SENSOR("  ", "Humidity", humidity_sensor_);
  LOG_BINARY_SENSOR("  ", "Reservoir", reservoir_sensor_);
  LOG_BINARY_SENSOR("  ", "Water", water_sensor_);
  LOG_BINARY_SENSOR("  ", "Misting", misting_sensor_);
  LOG_SWITCH("  ", "Power", power_switch_);
  LOG_SWITCH("  ", "Display", display_switch_);
  LOG_SELECT("  ", "Mode", mode_select_);
  LOG_NUMBER("  ", "Target Humidity", target_humidity_number_);
  LOG_NUMBER("  ", "Mist Level", mist_level_number_);
}

// =============================================================================
// UART Communication
// =============================================================================

void Humidifier::read_uart_() {
  const uint32_t now = millis();

  // Clear stale data
  if (!rx_buffer_.empty() && (now - last_rx_time_) > RX_TIMEOUT_MS) {
    rx_buffer_.clear();
  }

  while (available()) {
    uint8_t byte;
    read_byte(&byte);
    last_rx_time_ = now;

    // Sync to header
    if (rx_buffer_.empty() && byte != PACKET_HEADER) {
      continue;
    }

    rx_buffer_.push_back(byte);

    // Wait for minimum header
    if (rx_buffer_.size() < 6) {
      continue;
    }

    // Calculate expected size
    const uint8_t payload_len = rx_buffer_[static_cast<uint8_t>(Offset::PAYLOAD_LEN)];
    const size_t expected_size = 6 + payload_len;

    // Wait for complete packet
    if (rx_buffer_.size() < expected_size) {
      continue;
    }

    // Validate and process
    const uint8_t type = rx_buffer_[static_cast<uint8_t>(Offset::TYPE)];
    if (type == static_cast<uint8_t>(PacketType::STATUS)) {
      const uint8_t received_checksum = rx_buffer_[static_cast<uint8_t>(Offset::CHECKSUM)];
      const uint8_t calculated_checksum = calculate_checksum_(rx_buffer_.data(), expected_size);

      if (received_checksum == calculated_checksum) {
        parse_packet_(rx_buffer_.data(), expected_size);
      } else {
        ESP_LOGW(TAG, "Checksum mismatch: 0x%02X != 0x%02X", received_checksum, calculated_checksum);
      }
    }

    rx_buffer_.clear();
  }

  // Prevent overflow
  if (rx_buffer_.size() > RX_BUFFER_MAX) {
    rx_buffer_.clear();
  }
}

void Humidifier::send_ping_() {
    uint8_t packet[] = {
      PACKET_HEADER,
      static_cast<uint8_t>(PacketType::PING),
      seq_++,
      static_cast<uint8_t>(PayloadLen::PING),
      0x00,
      0x00,
      ADDR_STATUS[0], ADDR_STATUS[1], ADDR_STATUS[2], ADDR_STATUS[3]
    };
  packet[static_cast<uint8_t>(Offset::CHECKSUM)] = calculate_checksum_(packet, sizeof(packet));
  write_array(packet, sizeof(packet));
}

void Humidifier::send_command_(const Address &addr, uint8_t value) {
    uint8_t packet[] = {
      PACKET_HEADER,
      static_cast<uint8_t>(PacketType::STATUS),
      seq_++,
      static_cast<uint8_t>(PayloadLen::COMMAND),
      0x00,
      0x00,
      addr[0], addr[1], addr[2], addr[3],
      0x01, 0x01, value
    };
  packet[static_cast<uint8_t>(Offset::CHECKSUM)] = calculate_checksum_(packet, sizeof(packet));
  write_array(packet, sizeof(packet));
  ESP_LOGV(TAG, "TX: %s", format_hex_pretty(packet, sizeof(packet)).c_str());
}

uint8_t Humidifier::calculate_checksum_(const uint8_t *data, size_t len) {
  uint8_t sum = 0;
  for (size_t i = 0; i < len; i++) {
    if (i != static_cast<uint8_t>(Offset::SEQ) && i != static_cast<uint8_t>(Offset::CHECKSUM)) {
      sum += data[i];
    }
  }
  return 0xFF - sum - data[static_cast<uint8_t>(Offset::SEQ)];
}

bool Humidifier::match_address_(const uint8_t *data, const Address &addr) {
  const size_t addr_offset = static_cast<size_t>(Offset::ADDR_START);
  return std::equal(data + addr_offset, data + addr_offset + addr.size(), addr.begin());
}

// =============================================================================
// Commands
// =============================================================================

void Humidifier::send_power(bool on) {
  ESP_LOGI(TAG, "Power: %s", on ? "ON" : "OFF");
  send_command_(ADDR_POWER, on ? VALUE_ON : VALUE_OFF);
}

void Humidifier::send_display(bool on) {
  ESP_LOGI(TAG, "Display: %s", on ? "ON" : "OFF");
  send_command_(ADDR_DISPLAY, on ? VALUE_ON : VALUE_OFF);
}

void Humidifier::send_mode(Mode mode) {
  ESP_LOGI(TAG, "Mode: %s%s", mode_to_string(mode), power_on_ ? "" : " (powering on)");
  if (!power_on_) send_command_(ADDR_POWER, VALUE_ON);
  send_command_(ADDR_MODE, static_cast<uint8_t>(mode));
}

void Humidifier::send_mist_level(uint8_t level, bool auto_switch_mode) {
  level = std::clamp(level, MIST_LEVEL_MIN, MIST_LEVEL_MAX);
  
  bool need_power = !power_on_;
  bool need_mode = auto_switch_mode && last_mode_ != Mode::MANUAL;
  
  ESP_LOGI(TAG, "Mist level: %d%s%s", level,
      need_power ? " (powering on)" : "",
      need_mode ? " (→ Manual)" : "");
  
  if (need_power) send_command_(ADDR_POWER, VALUE_ON);
  if (need_power || need_mode) send_command_(ADDR_MODE, static_cast<uint8_t>(Mode::MANUAL));
  send_command_(ADDR_MANUAL, level);
}

void Humidifier::send_target_humidity(uint8_t humidity, bool auto_switch_mode) {
  humidity = std::clamp(humidity, HUMIDITY_MIN, HUMIDITY_MAX);
  
  bool need_power = !power_on_;
  bool need_mode = auto_switch_mode && last_mode_ != Mode::AUTO && last_mode_ != Mode::SLEEP;
  
  ESP_LOGI(TAG, "Target humidity: %d%%%s%s", humidity,
      need_power ? " (powering on)" : "",
      need_mode ? " (→ Auto)" : "");
  
  if (need_power) send_command_(ADDR_POWER, VALUE_ON);
  if (need_power || need_mode) send_command_(ADDR_MODE, static_cast<uint8_t>(Mode::AUTO));
  send_command_(ADDR_TARGET_HUMIDITY, humidity);
}

void Humidifier::send_wifi_status(bool ha_connected, bool wifi_connected) {
  uint8_t status;
  const char *status_str;
  
  if (ha_connected) {
    status = static_cast<uint8_t>(WifiStatus::CONNECTED);
    status_str = "solid (HA connected)";
  } else if (wifi_connected) {
    status = static_cast<uint8_t>(WifiStatus::CONNECTING);
    status_str = "blinking (WiFi only)";
  } else {
    status = static_cast<uint8_t>(WifiStatus::DISCONNECTED);
    status_str = "off (no WiFi)";
  }
  
  constexpr uint8_t blink_lo = WIFI_BLINK_MS & 0xFF;
  constexpr uint8_t blink_hi = (WIFI_BLINK_MS >> 8) & 0xFF;
  
    uint8_t packet[] = {
      PACKET_HEADER,
      static_cast<uint8_t>(PacketType::STATUS),
      seq_++,
      static_cast<uint8_t>(PayloadLen::WIFI_STATUS),
      0x00,
      0x00,
      ADDR_WIFI_STATUS[0], ADDR_WIFI_STATUS[1], ADDR_WIFI_STATUS[2], ADDR_WIFI_STATUS[3],
      static_cast<uint8_t>(WifiTLV::STATUS), 0x01, status,
      static_cast<uint8_t>(WifiTLV::BLINK_ON), 0x02, blink_lo, blink_hi,
      static_cast<uint8_t>(WifiTLV::BLINK_OFF), 0x02, blink_lo, blink_hi,
      static_cast<uint8_t>(WifiTLV::RESET_FLAG), 0x01, 0x00
    };
  
  packet[static_cast<uint8_t>(Offset::CHECKSUM)] = calculate_checksum_(packet, sizeof(packet));
  write_array(packet, sizeof(packet));
  
  ESP_LOGI(TAG, "WiFi LED: %s", status_str);
}

// =============================================================================
// Mode Helpers
// =============================================================================

const char *Humidifier::mode_to_string(Mode mode) {
  switch (mode) {
    case Mode::MANUAL: return MODE_MANUAL;
    case Mode::SLEEP:  return MODE_SLEEP;
    default:           return MODE_AUTO;
  }
}

Mode Humidifier::string_to_mode(const std::string &str) {
  if (str == MODE_MANUAL) return Mode::MANUAL;
  if (str == MODE_SLEEP)  return Mode::SLEEP;
  return Mode::AUTO;
}

// =============================================================================
// Packet Parsing
// =============================================================================

void Humidifier::parse_packet_(const uint8_t *data, size_t len) {
  if (len < static_cast<size_t>(Offset::TLV_START)) return;

  if (match_address_(data, ADDR_STATUS)) {
    parse_tlvs_(data, len, [this](uint8_t t, uint8_t l, const uint8_t *v) {
      handle_status_tlv_(t, l, v);
    });
  } else if (match_address_(data, ADDR_WIFI_STATUS)) {
    // Currently not used
    // parse_tlvs_(data, len, [this](uint8_t t, uint8_t l, const uint8_t *v) {
    //   handle_wifi_tlv_(t, l, v);
    // });
  }
}

template<typename Handler>
void Humidifier::parse_tlvs_(const uint8_t *data, size_t len, Handler handler) {
  constexpr size_t tlv_start = static_cast<size_t>(Offset::TLV_START);
  size_t pos = tlv_start;
  
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

void Humidifier::handle_status_tlv_(uint8_t type, uint8_t len, const uint8_t *value) {
  if (len == 0) return;

  const uint8_t v = value[0];

  switch (static_cast<TLV>(type)) {
    case TLV::POWER:
      power_on_ = (v == VALUE_ON);
      if (power_switch_) power_switch_->publish_state(power_on_);
      if (!power_on_) invalidate_diagnostic_sensors_();
      break;

    case TLV::RESERVOIR:
      if (power_on_ && reservoir_sensor_) {
        reservoir_sensor_->publish_state(v == VALUE_OFF);
      }
      break;

    case TLV::WATER:
      // Moisture sensor: ON = wet (has water), OFF = dry (empty)
      // Only update when power ON and reservoir attached
      if (power_on_ && water_sensor_ && reservoir_sensor_ && reservoir_sensor_->state) {
        water_sensor_->publish_state(v == VALUE_OFF);
      }
      break;

    case TLV::DISPLAY:
      if (display_switch_) display_switch_->publish_state(v == VALUE_ACTIVE);
      break;

    case TLV::MISTING:
      if (power_on_ && misting_sensor_) {
        misting_sensor_->publish_state(v == VALUE_ON);
      }
      break;

    case TLV::TARGET_HUMIDITY:
      if (target_humidity_number_) target_humidity_number_->publish_state(v);
      break;

    case TLV::CURRENT_HUMIDITY:
      if (humidity_sensor_) humidity_sensor_->publish_state(v);
      break;

    case TLV::MODE:
      last_mode_ = static_cast<Mode>(v);
      if (mode_select_) mode_select_->publish_state(mode_to_string(last_mode_));
      break;

    case TLV::MIST_LEVEL:
      if (mist_level_number_) mist_level_number_->publish_state(v);
      break;

    default:
      ESP_LOGV(TAG, "Unknown Status TLV 0x%02X: %s", type, format_hex_pretty(value, len).c_str());
      break;
  }
}

void Humidifier::handle_wifi_tlv_(uint8_t type, uint8_t len, const uint8_t *value) {
  if (len == 0) return;

  switch (static_cast<WifiTLV>(type)) {
    case WifiTLV::STATUS:
      if (len >= 1) {
        const uint8_t status = value[0];        
        const char *status_str;
        switch (status) {
          case static_cast<uint8_t>(WifiStatus::DISCONNECTED): status_str = "disconnected"; break;
          case static_cast<uint8_t>(WifiStatus::CONNECTED): status_str = "connected"; break;
          case static_cast<uint8_t>(WifiStatus::CONNECTING): status_str = "connecting"; break;
          default: status_str = "unknown"; break;
        }
        ESP_LOGD(TAG, "WiFi: %s (0x%02X)", status_str, status);
      }
      break;
      
    case WifiTLV::BLINK_ON:
    case WifiTLV::BLINK_OFF:
      if (len >= 2) {
        uint16_t ms = value[0] | (value[1] << 8);
        ESP_LOGV(TAG, "WiFi blink %s: %dms", 
            type == static_cast<uint8_t>(WifiTLV::BLINK_ON) ? "on" : "off", ms);
      }
      break;
      
    case WifiTLV::RESET_FLAG:
      if (len >= 1) {
        ESP_LOGD(TAG, "WiFi reset flag: 0x%02X", value[0]);
      }
      break;
      
    default:
      ESP_LOGV(TAG, "Unknown WiFi TLV 0x%02X: %s", type, format_hex_pretty(value, len).c_str());
      break;
  }
}

void Humidifier::invalidate_diagnostic_sensors_() {
  if (reservoir_sensor_) reservoir_sensor_->invalidate_state();
  // water_sensor_ excluded - sticks after power off
  if (misting_sensor_) misting_sensor_->invalidate_state();
}

}  // namespace levoit_humidifier
}  // namespace esphome