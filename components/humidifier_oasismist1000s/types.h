#pragma once

#include <cstdint>
#include <cstddef>
#include <array>
#include <string>

namespace esphome {
namespace humidifier_oasismist1000s {

// =============================================================================
// RX Constants
// =============================================================================

inline constexpr size_t RX_BUFFER_MAX = 128;
inline constexpr size_t RX_MIN_HEADER_LEN = 6;
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
  COMMAND = 7,
  WIFI_LED = 18,
};

enum class Offset : size_t {
  HEADER = 0,
  TYPE = 1,
  SEQ = 2,
  PAYLOAD_LEN = 3,
  CHECKSUM = 5,
  ADDR_START = 6,
  TLV_START = 10,
};

// =============================================================================
// Command Addresses (4 bytes each)
// =============================================================================

using Address = std::array<uint8_t, 4>;

inline constexpr Address ADDR_POWER           = {0x02, 0x00, 0x50, 0x00};
inline constexpr Address ADDR_DISPLAY         = {0x02, 0x0F, 0x50, 0x00};
inline constexpr Address ADDR_WIFI_LED        = {0x02, 0x18, 0x50, 0x00};
inline constexpr Address ADDR_STATUS          = {0x02, 0x30, 0x55, 0x00};
inline constexpr Address ADDR_MODE            = {0x02, 0x32, 0x55, 0x00};
inline constexpr Address ADDR_MANUAL          = {0x02, 0x33, 0x55, 0x00};
inline constexpr Address ADDR_TARGET_HUMIDITY = {0x02, 0x36, 0x55, 0x00};

// =============================================================================
// TLV Types - Status (ADDR_STATUS)
// =============================================================================

enum class TLV : uint8_t {
  POWER = 0x02,
  RESERVOIR = 0x03,
  WATER = 0x04,
  DISPLAY = 0x05,
  MISTING = 0x07,
  TARGET_HUMIDITY = 0x08,
  CURRENT_HUMIDITY = 0x09,
  MODE = 0x0B,
  MIST_LEVEL = 0x0C,
};

// =============================================================================
// TLV Types - WiFi LED (ADDR_WIFI_LED)
// =============================================================================

// The MCU echoes these TLVs back on ADDR_WIFI_LED; we ignore the RX side.
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
  AUTO = 0x00,
  MANUAL = 0x01,
  SLEEP = 0x02,
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

inline Mode string_to_mode(const std::string &str) {
  if (str == MODE_MANUAL) return Mode::MANUAL;
  if (str == MODE_SLEEP)  return Mode::SLEEP;
  return Mode::AUTO;
}

// =============================================================================
// Values
// =============================================================================

inline constexpr uint8_t VALUE_OFF = 0x00;
inline constexpr uint8_t VALUE_ON = 0x01;
inline constexpr uint8_t VALUE_ACTIVE = 0x64;

inline constexpr uint8_t HUMIDITY_MIN = 40;
inline constexpr uint8_t HUMIDITY_MAX = 80;
inline constexpr uint8_t MIST_LEVEL_MIN = 1;
inline constexpr uint8_t MIST_LEVEL_MAX = 9;

// =============================================================================
// WiFi LED
// =============================================================================

enum class WifiLedStatus : uint8_t {
  OFF = 0x00,       // LED off (disconnected)
  SOLID = 0x01,     // LED solid on (HA connected)
  BLINKING = 0x02,  // LED blinking (WiFi only, connecting to HA)
};

inline constexpr uint16_t WIFI_BLINK_MS = 500;

}  // namespace humidifier_oasismist1000s
}  // namespace esphome
