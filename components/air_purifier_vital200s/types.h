#pragma once

#include <cstdint>
#include <cstddef>
#include <array>
#include <string>

namespace esphome {
namespace air_purifier_vital200s {

// =============================================================================
// RX Constants
// =============================================================================

inline constexpr size_t RX_BUFFER_MAX = 128;
inline constexpr size_t RX_MIN_HEADER_LEN = 6;
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

inline constexpr Address ADDR_POWER = {0x02, 0x00, 0x50, 0x00};
inline constexpr Address ADDR_STATUS = {0x02, 0x00, 0x55, 0x00};
inline constexpr Address ADDR_MODE = {0x02, 0x02, 0x55, 0x00};
inline constexpr Address ADDR_MANUAL_SPEED = {0x02, 0x03, 0x55, 0x00};
inline constexpr Address ADDR_DISPLAY = {0x02, 0x04, 0x55, 0x00};
inline constexpr Address ADDR_LIGHT_DETECTION = {0x02, 0x11, 0x55, 0x00};
inline constexpr Address ADDR_WIFI_LED = {0x02, 0x18, 0x50, 0x00};
inline constexpr Address ADDR_TIMER_SET = {0x02, 0x19, 0x50, 0x00};
inline constexpr Address ADDR_TIMER = {0x02, 0x1B, 0x50, 0x00};
inline constexpr Address ADDR_DISPLAY_LOCK = {0x02, 0x40, 0x51, 0x00};

// Observed, not implemented:
//   ADDR_FILTER_RESET = {0x02, 0x05, 0x55, 0x00}, PayloadLen 6, action byte 0x03.
//   Write-only: MCU never reports filter life, so nothing to expose in HA.

// =============================================================================
// TLV Types - Status Response
// =============================================================================

enum class TLV : uint8_t {
  POWER = 0x02,
  MODE = 0x03,
  SPEED = 0x04,
  // 0x06 = live display brightness; ignored, DISPLAY_SAVED is the switch state
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

// MCU echoes these back on ADDR_WIFI_LED; RX side ignored
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

// index = Mode value: MANUAL=0, SLEEP=1, AUTO=2
inline constexpr std::array<const char *, 3> MODE_NAMES = {MODE_MANUAL, MODE_SLEEP, MODE_AUTO};

inline constexpr const char *mode_to_string(Mode mode) {
  const size_t i = static_cast<size_t>(mode);
  return i < MODE_NAMES.size() ? MODE_NAMES[i] : MODE_AUTO;
}

inline Mode string_to_mode(const std::string &str) {
  if (str == MODE_MANUAL) return Mode::MANUAL;
  if (str == MODE_SLEEP) return Mode::SLEEP;
  return Mode::AUTO;
}

// =============================================================================
// Air Quality
// =============================================================================

// index = MCU air-quality value; 0 doubles as the out-of-range fallback
inline constexpr std::array<const char *, 5> AIR_QUALITY_NAMES = {"Unknown", "Very Good", "Good", "Moderate", "Bad"};

inline constexpr const char *air_quality_to_string(uint8_t value) {
  return value < AIR_QUALITY_NAMES.size() ? AIR_QUALITY_NAMES[value] : AIR_QUALITY_NAMES[0];
}

// =============================================================================
// Fan Speed
// =============================================================================

inline constexpr uint8_t FAN_SPEED_COUNT = 4;  // speeds are 1..4

// =============================================================================
// Values
// =============================================================================

inline constexpr uint8_t VALUE_OFF = 0x00;
inline constexpr uint8_t VALUE_ON = 0x01;
inline constexpr uint8_t DISPLAY_ON_BRIGHTNESS = 0x64;  // display takes a brightness byte, not VALUE_ON

// =============================================================================
// WiFi LED
// =============================================================================

enum class WifiLedStatus : uint8_t {
  OFF = 0x00,       // disconnected
  SOLID = 0x01,     // HA connected
  BLINKING = 0x02,  // WiFi only, connecting to HA
};

inline constexpr uint16_t WIFI_BLINK_MS = 500;

}  // namespace air_purifier_vital200s
}  // namespace esphome