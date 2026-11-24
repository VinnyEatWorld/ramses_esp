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
  void process_rx_frame_byte_(uint8_t byte);
  bool process_message_byte_(uint8_t byte);
  void reset_rx_frame_();
  void reject_rx_frame_(const char *reason);
  void log_valid_frame_();

  uint32_t rx_sync_buffer_{0};
  uint32_t rx_sync_count_{0};
  uint32_t rx_valid_frame_count_{0};
  uint32_t rx_rejected_frame_count_{0};
  uint32_t rx_byte_count_{0};
  uint32_t rx_report_ms_{0};

  bool rx_frame_active_{false};
  bool rx_have_half_nibble_{false};
  uint8_t rx_half_nibble_{0};
  uint8_t rx_message_state_{0};
  uint8_t rx_message_checksum_{0};
  uint8_t rx_message_type_{0};
  uint8_t rx_address_mask_{0};
  uint8_t rx_param_mask_{0};
  uint8_t rx_encoded_byte_count_{0};
  uint8_t rx_address_slot_{0};
  uint8_t rx_address_byte_{0};
  uint8_t rx_param_slot_{0};
  uint8_t rx_addresses_[3][3]{};
  uint8_t rx_params_[2]{};
  uint8_t rx_opcode_[2]{};
  uint8_t rx_opcode_bytes_{0};
  uint8_t rx_payload_length_{0};
  uint8_t rx_payload_count_{0};
  uint8_t rx_payload_[64]{};
};

}  // namespace ramses_esp
}  // namespace esphome
