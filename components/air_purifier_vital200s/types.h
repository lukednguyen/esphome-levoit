#pragma once

#include <cstdint>
#include <cstddef>
#include <array>

namespace esphome {
namespace air_purifier_vital200s {

// =============================================================================
// RX Constants
// =============================================================================

inline constexpr size_t RX_BUFFER_MAX = 128;
inline constexpr size_t RX_MIN_PACKET_LEN = 10;
inline constexpr uint32_t RX_TIMEOUT_MS = 100;

// =============================================================================
// Packet Structure
// =============================================================================

inline constexpr uint8_t PACKET_HEADER = 0xA5;

enum class PacketType : uint8_t {
  PING = 0x12,
  STATUS = 0x22,
};

enum class PayloadLen : uint8_t {
  PING = 4,
  // FILTER_RESET = 6,  // Commented out - filter life not readable from MCU
  COMMAND = 7,
  TIMER_CANCEL = 10,
  WIFI_LED = 18,
};

enum class Offset : size_t {
  HEADER = 0,
  TYPE = 1,
  SEQ = 2,
  PAYLOAD_LEN = 3,
  CHECKSUM = 5,
  ADDR_START = 6,
  TLV_START_TIMER = 10,
  TLV_START_STATUS = 18,
};

// =============================================================================
// Command Addresses
// =============================================================================

using Address = std::array<uint8_t, 4>;

inline constexpr Address ADDR_POWER           = {0x02, 0x00, 0x50, 0x00};
inline constexpr Address ADDR_STATUS          = {0x02, 0x00, 0x55, 0x00};
inline constexpr Address ADDR_MODE            = {0x02, 0x02, 0x55, 0x00};
inline constexpr Address ADDR_MANUAL_SPEED    = {0x02, 0x03, 0x55, 0x00};
inline constexpr Address ADDR_DISPLAY         = {0x02, 0x04, 0x55, 0x00};
// inline constexpr Address ADDR_FILTER_RESET = {0x02, 0x05, 0x55, 0x00};  // Commented out
inline constexpr Address ADDR_LIGHT_DETECTION = {0x02, 0x11, 0x55, 0x00};
inline constexpr Address ADDR_WIFI_LED        = {0x02, 0x18, 0x50, 0x00};
inline constexpr Address ADDR_TIMER_SET       = {0x02, 0x19, 0x50, 0x00};
inline constexpr Address ADDR_TIMER           = {0x02, 0x1B, 0x50, 0x00};
inline constexpr Address ADDR_DISPLAY_LOCK    = {0x02, 0x40, 0x51, 0x00};

// =============================================================================
// TLV Types - Status Response
// =============================================================================

enum class TLV : uint8_t {
  POWER = 0x02,
  MODE = 0x03,
  SPEED = 0x04,
  DISPLAY_CURRENT = 0x06,
  DISPLAY_SAVED = 0x07,
  AIR_QUALITY = 0x09,
  PM25 = 0x0B,
  DISPLAY_LOCK = 0x0E,
  LIGHT_DETECTION = 0x13,
};

// =============================================================================
// TLV Types - Timer Response (ADDR_TIMER)
// =============================================================================

enum class TimerTLV : uint8_t {
  REMAINING = 0x01,
  TOTAL = 0x02,
};

// =============================================================================
// TLV Types - WiFi LED (ADDR_WIFI_LED)
// =============================================================================

enum class WifiLedTLV : uint8_t {
  STATUS = 0x01,
  BLINK_ON = 0x02,
  BLINK_OFF = 0x03,
  RESET_FLAG = 0x04,
};

// =============================================================================
// Mode
// =============================================================================

enum class Mode : uint8_t {
  MANUAL = 0x00,
  SLEEP = 0x01,
  AUTO = 0x02,
};

inline constexpr const char *MODE_AUTO = "Auto";
inline constexpr const char *MODE_MANUAL = "Manual";
inline constexpr const char *MODE_SLEEP = "Sleep";

inline constexpr const char *mode_to_string(Mode mode) {
  switch (mode) {
    case Mode::MANUAL: return MODE_MANUAL;
    case Mode::SLEEP:  return MODE_SLEEP;
    default:           return MODE_AUTO;
  }
}

inline constexpr Mode string_to_mode(const std::string &str) {
  if (str == MODE_MANUAL) return Mode::MANUAL;
  if (str == MODE_SLEEP)  return Mode::SLEEP;
  return Mode::AUTO;
}

// =============================================================================
// Air Quality
// =============================================================================

enum class AirQuality : uint8_t {
  UNKNOWN = 0,
  VERY_GOOD = 1,
  GOOD = 2,
  MODERATE = 3,
  BAD = 4,
};

inline constexpr const char *AIR_QUALITY_UNKNOWN = "Unknown";
inline constexpr const char *AIR_QUALITY_VERY_GOOD = "Very Good";
inline constexpr const char *AIR_QUALITY_GOOD = "Good";
inline constexpr const char *AIR_QUALITY_MODERATE = "Moderate";
inline constexpr const char *AIR_QUALITY_BAD = "Bad";

inline constexpr const char *air_quality_to_string(AirQuality quality) {
  switch (quality) {
    case AirQuality::VERY_GOOD: return AIR_QUALITY_VERY_GOOD;
    case AirQuality::GOOD:      return AIR_QUALITY_GOOD;
    case AirQuality::MODERATE:  return AIR_QUALITY_MODERATE;
    case AirQuality::BAD:       return AIR_QUALITY_BAD;
    default:                    return AIR_QUALITY_UNKNOWN;
  }
}

inline constexpr AirQuality uint8_to_air_quality(uint8_t value) {
  if (value >= static_cast<uint8_t>(AirQuality::VERY_GOOD) &&
      value <= static_cast<uint8_t>(AirQuality::BAD)) {
    return static_cast<AirQuality>(value);
  }
  return AirQuality::UNKNOWN;
}

// =============================================================================
// Fan Speed
// =============================================================================

inline constexpr uint8_t FAN_SPEED_MIN = 1;
inline constexpr uint8_t FAN_SPEED_MAX = 4;
inline constexpr uint8_t FAN_SPEED_COUNT = 4;

// =============================================================================
// Values
// =============================================================================

inline constexpr uint8_t VALUE_OFF = 0x00;
inline constexpr uint8_t VALUE_ON = 0x01;

// Filter (commented out - filter life not readable from MCU)
// inline constexpr uint8_t FILTER_RESET_ACTION = 0x03;

enum class DisplayBrightness : uint8_t {
  OFF = 0x00,
  ON = 0x64,
};

// =============================================================================
// WiFi LED
// =============================================================================

enum class WifiLedStatus : uint8_t {
  OFF = 0x00,       // LED off (disconnected)
  SOLID = 0x01,     // LED solid on (HA connected)
  BLINKING = 0x02,  // LED blinking (WiFi only, connecting to HA)
};

inline constexpr uint16_t WIFI_BLINK_MS = 500;

}  // namespace air_purifier_vital200s
}  // namespace esphome