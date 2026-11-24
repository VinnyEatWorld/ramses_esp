#pragma once

#include "esphome/core/component.h"
#include "esphome/components/spi/spi.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

#include <string>

namespace esphome {
namespace ramses_esp {

class RamsesEsp : public Component,
                  public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW,
                                        spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_1MHZ>,
                  public uart::UARTDevice {
 public:
  void set_fan_id(const std::string &id);
  void set_sensor_timeout_seconds(uint32_t seconds) { this->sensor_timeout_ms_ = seconds * 1000UL; }
  void set_co2_sensor(sensor::Sensor *entity) { this->co2_sensor_ = entity; }
  void set_boost_duration_sensor(sensor::Sensor *entity) { this->boost_duration_sensor_ = entity; }
  void set_fan_mode_sensor(text_sensor::TextSensor *entity) { this->fan_mode_sensor_ = entity; }
  void set_co2_presence_sensor(binary_sensor::BinarySensor *entity) { this->co2_presence_sensor_ = entity; }

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
  void process_application_frame_();
  bool address_matches_(uint8_t slot, const std::string &expected) const;
  std::string address_string_(uint8_t slot) const;
  void mark_co2_sensor_seen_();
  void publish_fan_mode_(uint8_t mode);

  uint32_t rx_sync_buffer_{0};
  uint32_t rx_sync_count_{0};
  uint32_t rx_valid_frame_count_{0};
  uint32_t rx_rejected_frame_count_{0};
  uint32_t rx_byte_count_{0};
  uint32_t rx_report_ms_{0};
  uint32_t sensor_timeout_ms_{1800000};
  uint32_t last_co2_sensor_seen_ms_{0};
  std::string fan_id_{};
  std::string observed_remote_id_{};
  std::string co2_sensor_id_{};
  std::string last_fan_mode_{};
  uint16_t last_co2_ppm_{0};
  uint16_t last_boost_duration_minutes_{0};
  bool has_co2_ppm_{false};
  bool has_boost_duration_{false};
  bool has_seen_co2_sensor_{false};
  bool co2_sensor_present_{false};
  sensor::Sensor *co2_sensor_{nullptr};
  sensor::Sensor *boost_duration_sensor_{nullptr};
  text_sensor::TextSensor *fan_mode_sensor_{nullptr};
  binary_sensor::BinarySensor *co2_presence_sensor_{nullptr};

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
