#pragma once

#include "esphome/core/component.h"
#include "esphome/components/spi/spi.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace ramses_esp {

class RamsesEsp : public Component,
                  public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW,
                                        spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_1MHZ>,
                  public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

 protected:
  bool initialize_cc1101_();
  uint8_t read_register_(uint8_t address);
  uint8_t write_register_(uint8_t address, uint8_t value);
  uint8_t strobe_(uint8_t command);
  bool start_rx_();
  void process_rx_byte_(uint8_t byte);

  uint32_t rx_sync_buffer_{0};
  uint32_t rx_sync_count_{0};
  uint32_t rx_byte_count_{0};
  uint32_t rx_report_ms_{0};
};

}  // namespace ramses_esp
}  // namespace esphome
