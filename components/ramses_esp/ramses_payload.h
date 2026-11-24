#pragma once

#include <stddef.h>
#include <stdint.h>

namespace esphome {
namespace ramses_esp {

struct OrconBoostTimer {
  uint16_t minutes;
  uint8_t speed;
};

// 1298: domain/index byte, then big-endian CO2 ppm. Values with bit 15 set
// are protocol fault sentinels, not measurements.
inline bool decode_co2_payload(const uint8_t *payload, size_t length, uint16_t *ppm) {
  if (payload == nullptr || ppm == nullptr || length != 3)
    return false;
  const uint16_t value = (static_cast<uint16_t>(payload[1]) << 8) | payload[2];
  *ppm = value;
  if ((value & 0x8000) != 0)
    return false;
  return true;
}

// Orcon's short 31D9 payload carries its mode index in byte 2.
inline bool decode_orcon_fan_mode(const uint8_t *payload, size_t length, uint8_t *mode) {
  if (payload == nullptr || mode == nullptr || length != 3)
    return false;
  *mode = payload[2];
  return true;
}

// Orcon 22F3 dialects use flags 0x02 (newer remotes) or 0x12 (classic
// remotes), with bit 0x40 selecting hours. Byte 3 is boost speed: 1..3.
inline bool decode_orcon_boost_timer(const uint8_t *payload, size_t length, OrconBoostTimer *timer) {
  if (payload == nullptr || timer == nullptr || length != 7 || payload[0] != 0)
    return false;
  const uint8_t flags = payload[1];
  const uint8_t dialect = flags & 0x12;
  if (dialect != 0x02 && dialect != 0x12)
    return false;
  const uint8_t encoded_duration = payload[2];
  if (encoded_duration == 0 || payload[3] < 1 || payload[3] > 3)
    return false;
  timer->minutes = (flags & 0x40) ? static_cast<uint16_t>(encoded_duration) * 60U : encoded_duration;
  timer->speed = payload[3];
  return true;
}

}  // namespace ramses_esp
}  // namespace esphome
