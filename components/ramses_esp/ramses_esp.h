#pragma once

#include "esphome/core/component.h"
#include "esphome/components/spi/spi.h"

namespace esphome {
namespace ramses_esp {

class RamsesEsp : public Component,
                  public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW,
                                        spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_1MHZ> {
 public:
  void setup() override;
  void dump_config() override;

 protected:
  bool initialize_cc1101_();
  uint8_t read_register_(uint8_t address);
  uint8_t write_register_(uint8_t address, uint8_t value);
  uint8_t strobe_(uint8_t command);
};

}  // namespace ramses_esp
}  // namespace esphome
