#include "../components/ramses_esp/ramses_payload.h"

#include <cassert>

using namespace esphome::ramses_esp;

int main() {
  uint16_t ppm = 0;
  const uint8_t co2_1475[] = {0x00, 0x05, 0xC3};
  const uint8_t co2_1457[] = {0x00, 0x05, 0xB1};
  const uint8_t co2_fault[] = {0x00, 0x80, 0x01};
  assert(decode_co2_payload(co2_1475, sizeof(co2_1475), &ppm) && ppm == 1475);
  assert(decode_co2_payload(co2_1457, sizeof(co2_1457), &ppm) && ppm == 1457);
  assert(!decode_co2_payload(co2_fault, sizeof(co2_fault), &ppm));
  assert(!decode_co2_payload(co2_1475, 2, &ppm));

  uint8_t mode = 0xFF;
  const uint8_t fan_auto[] = {0x00, 0x80, 0x04};
  const uint8_t fan_low[] = {0x00, 0x80, 0x01};
  assert(decode_orcon_fan_mode(fan_auto, sizeof(fan_auto), &mode) && mode == 4);
  assert(decode_orcon_fan_mode(fan_low, sizeof(fan_low), &mode) && mode == 1);
  assert(!decode_orcon_fan_mode(fan_low, 2, &mode));

  OrconBoostTimer timer{};
  const uint8_t newer_remote_high_15[] = {0x00, 0x02, 0x0F, 0x03, 0x04, 0x00, 0x00};
  const uint8_t classic_remote_low_60[] = {0x00, 0x12, 0x3C, 0x01, 0x04, 0x04, 0x04};
  const uint8_t invalid_speed[] = {0x00, 0x02, 0x0F, 0x04, 0x04, 0x00, 0x00};
  assert(decode_orcon_boost_timer(newer_remote_high_15, sizeof(newer_remote_high_15), &timer));
  assert(timer.minutes == 15 && timer.speed == 3);
  assert(decode_orcon_boost_timer(classic_remote_low_60, sizeof(classic_remote_low_60), &timer));
  assert(timer.minutes == 60 && timer.speed == 1);
  assert(!decode_orcon_boost_timer(invalid_speed, sizeof(invalid_speed), &timer));
  assert(!decode_orcon_boost_timer(newer_remote_high_15, 6, &timer));
  return 0;
}
