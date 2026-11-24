#include "ramses_esp.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "cc1101_param.h"
#include "ramses_payload.h"

#include <cstdio>

namespace esphome {
namespace ramses_esp {

static const char *const TAG = "ramses_esp";
static const uint32_t CC1101_RESET_TIMEOUT_MS = 100;
static const uint8_t CC_MARCSTATE_IDLE = 0x01;
static const uint8_t CC_MARCSTATE_RX = 0x0D;
static const uint8_t CC_MARCSTATE_MASK = 0x1F;
static const uint8_t CC_RX_TIMEOUT_MS = 100;
static const uint8_t CC_PKTCTRL0_ASYNC_INFINITE = 0x32;
static const uint8_t CC_IOCFG0_UNUSED = 0x2E;
static const uint32_t RAMSES_SYNC_WORD = 0x00335553;
static const uint8_t RAMSES_TRAILER = 0x35;
static const uint8_t RAMSES_MAX_ENCODED_BYTES = 162;

enum RxMessageState : uint8_t {
  RX_MSG_HEADER,
  RX_MSG_ADDRESSES,
  RX_MSG_PARAMS,
  RX_MSG_OPCODE,
  RX_MSG_LENGTH,
  RX_MSG_PAYLOAD,
  RX_MSG_CHECKSUM,
  RX_MSG_COMPLETE,
};

static const uint8_t RAMSES_ADDRESS_MASKS[4] = {0x07, 0x04, 0x05, 0x03};
static const uint8_t MANCHESTER_DECODE[16] = {0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x03, 0x02, 0x0F,
                                               0x0F, 0x01, 0x00, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F};

#ifndef RAMSES_ESP_SOURCE_INFO
#define RAMSES_ESP_SOURCE_INFO "source metadata unavailable"
#endif

struct RegisterCheck {
  uint8_t address;
  const char *name;
};

void RamsesEsp::setup() {
  ESP_LOGI(TAG, "Component source: %s", RAMSES_ESP_SOURCE_INFO);
  this->spi_setup();

  if (!this->initialize_cc1101_()) {
    ESP_LOGE(TAG, "CC1101 initialization failed; check SPI wiring, chip select, and module DIP switches");
    this->mark_failed();
    return;
  }

  if (!this->start_rx_()) {
    ESP_LOGE(TAG, "CC1101 failed to enter RX mode");
    this->mark_failed();
    return;
  }

  ESP_LOGI(TAG, "CC1101 initialized with the RAMSES 868.3 MHz profile; RX sync detection is active");
}

void RamsesEsp::loop() {
  // Drain bytes buffered by ESPHome's UART component without blocking the main loop.
  this->read_available([this](uint8_t byte) { this->process_rx_byte_(byte); });

  const uint32_t now = millis();
  if (this->co2_sensor_present_ && this->has_seen_co2_sensor_ &&
      now - this->last_co2_sensor_seen_ms_ >= this->sensor_timeout_ms_) {
    this->co2_sensor_present_ = false;
    if (this->co2_presence_sensor_ != nullptr)
      this->co2_presence_sensor_->publish_state(false);
    ESP_LOGI(TAG, "Configured CO2 sensor %s has timed out", this->co2_sensor_id_.c_str());
  }
  if (now - this->rx_report_ms_ >= 5000) {
    ESP_LOGD(TAG, "UART RX bytes=%lu, syncs=%lu, valid frames=%lu, rejected frames=%lu",
             static_cast<unsigned long>(this->rx_byte_count_),
             static_cast<unsigned long>(this->rx_sync_count_),
             static_cast<unsigned long>(this->rx_valid_frame_count_),
             static_cast<unsigned long>(this->rx_rejected_frame_count_));
    this->rx_report_ms_ = now;
  }
}

void RamsesEsp::dump_config() {
  ESP_LOGCONFIG(TAG, "RAMSES ESP:");
  ESP_LOGCONFIG(TAG, "  CC1101 SPI: mode 0, MSB first, 1 MHz");
  ESP_LOGCONFIG(TAG, "  Radio profile: 868.3 MHz GFSK, 38.4 kbaud UART RX");
  ESP_LOGCONFIG(TAG, "  RX: RAMSES Manchester/message validation enabled; TX is not started");
  ESP_LOGCONFIG(TAG, "  Filtered telemetry: paired fan and learned peers; sensor timeout=%lu s",
                static_cast<unsigned long>(this->sensor_timeout_ms_ / 1000UL));
}

bool RamsesEsp::start_rx_() {
  // Follow the upstream RX mode: asynchronous serial output on GDO2 and an
  // infinite-length asynchronous packet. GDO0 is unused during this RX-only stage.
  this->strobe_(CC_SIDLE);
  this->write_register_(CC_IOCFG0, CC_IOCFG0_UNUSED);
  this->write_register_(CC_PKTCTRL0, CC_PKTCTRL0_ASYNC_INFINITE);
  this->strobe_(CC_SFRX);
  this->strobe_(CC_SRX);

  const uint32_t started = millis();
  uint8_t marcstate = this->read_register_(CC_MARCSTATE) & CC_MARCSTATE_MASK;
  while (marcstate != CC_MARCSTATE_RX && (millis() - started) < CC_RX_TIMEOUT_MS) {
    delay(1);
    marcstate = this->read_register_(CC_MARCSTATE) & CC_MARCSTATE_MASK;
  }

  if (marcstate != CC_MARCSTATE_RX) {
    ESP_LOGE(TAG, "CC1101 did not enter RX within %u ms (MARCSTATE=0x%02X)",
             static_cast<unsigned>(CC_RX_TIMEOUT_MS), marcstate);
    return false;
  }

  this->rx_sync_buffer_ = 0;
  this->rx_sync_count_ = 0;
  this->rx_valid_frame_count_ = 0;
  this->rx_rejected_frame_count_ = 0;
  this->rx_byte_count_ = 0;
  this->rx_report_ms_ = millis();
  ESP_LOGI(TAG, "CC1101 MARCSTATE=0x%02X (RX); waiting for RAMSES sync 00 33 55 53", marcstate);
  return true;
}

void RamsesEsp::process_rx_byte_(uint8_t byte) {
  this->rx_byte_count_++;
  if (this->rx_frame_active_) {
    this->process_rx_frame_byte_(byte);
    return;
  }

  this->rx_sync_buffer_ = (this->rx_sync_buffer_ << 8) | byte;
  if (this->rx_sync_buffer_ == RAMSES_SYNC_WORD) {
    this->rx_sync_count_++;
    ESP_LOGI(TAG, "RAMSES sync detected on UART RX (count=%lu); decoding frame",
             static_cast<unsigned long>(this->rx_sync_count_));
    this->reset_rx_frame_();
    this->rx_frame_active_ = true;
  }
}

void RamsesEsp::process_rx_frame_byte_(uint8_t byte) {
  // Upstream frame.c recognizes the non-Manchester trailer before decoding
  // the data stream. A complete message must also pass its checksum.
  if (byte == RAMSES_TRAILER) {
    if (this->rx_message_state_ == RX_MSG_COMPLETE && this->rx_message_checksum_ == 0) {
      this->rx_valid_frame_count_++;
      this->log_valid_frame_();
      this->process_application_frame_();
      this->reset_rx_frame_();
    } else {
      this->reject_rx_frame_("trailer before complete message or invalid checksum");
    }
    return;
  }

  if (++this->rx_encoded_byte_count_ >= RAMSES_MAX_ENCODED_BYTES) {
    this->reject_rx_frame_("encoded frame exceeds upstream MAX_RAW limit");
    return;
  }

  const uint8_t low_code = MANCHESTER_DECODE[byte & 0x0F];
  const uint8_t high_code = MANCHESTER_DECODE[(byte >> 4) & 0x0F];
  if (low_code == 0x0F || high_code == 0x0F) {
    this->reject_rx_frame_("invalid Manchester code");
    return;
  }

  // This is the upstream frame.c nibble order: low encoded nibble first,
  // high encoded nibble second, assembled into a decoded message byte.
  const uint8_t decoded_nibble = low_code | (high_code << 2);
  if (!this->rx_have_half_nibble_) {
    this->rx_half_nibble_ = decoded_nibble;
    this->rx_have_half_nibble_ = true;
    return;
  }

  const uint8_t message_byte = static_cast<uint8_t>((this->rx_half_nibble_ << 4) | decoded_nibble);
  this->rx_have_half_nibble_ = false;
  if (!this->process_message_byte_(message_byte))
    this->reject_rx_frame_("invalid RAMSES message structure");
}

bool RamsesEsp::address_matches_(uint8_t slot, const std::string &expected) const {
  return !expected.empty() && this->address_string_(slot) == expected;
}

std::string RamsesEsp::address_string_(uint8_t slot) const {
  if (slot >= 3 || (this->rx_address_mask_ & (1U << slot)) == 0)
    return {};
  const uint8_t *addr = this->rx_addresses_[slot];
  const uint8_t device_class = (addr[0] & 0xFC) >> 2;
  const uint32_t device_id = (static_cast<uint32_t>(addr[0] & 0x03) << 16) |
                             (static_cast<uint32_t>(addr[1]) << 8) | addr[2];
  char formatted[12];
  std::snprintf(formatted, sizeof(formatted), "%02u:%06lu", static_cast<unsigned>(device_class),
                static_cast<unsigned long>(device_id));
  return formatted;
}

void RamsesEsp::set_fan_id(const std::string &id) {
  if (this->fan_id_ == id)
    return;
  this->fan_id_ = id;
  this->observed_remote_id_.clear();
  this->co2_sensor_id_.clear();
  this->has_seen_co2_sensor_ = false;
  this->co2_sensor_present_ = false;
  this->has_co2_ppm_ = false;
  this->has_boost_duration_ = false;
  this->last_fan_mode_.clear();
}

void RamsesEsp::mark_co2_sensor_seen_() {
  this->last_co2_sensor_seen_ms_ = millis();
  this->has_seen_co2_sensor_ = true;
  if (!this->co2_sensor_present_) {
    this->co2_sensor_present_ = true;
    if (this->co2_presence_sensor_ != nullptr)
      this->co2_presence_sensor_->publish_state(true);
    ESP_LOGI(TAG, "Paired CO2 sensor %s detected", this->co2_sensor_id_.c_str());
  }
}

void RamsesEsp::publish_fan_mode_(uint8_t mode) {
  // This is the Orcon 3-byte 31D9 mode field mapping used by ramses_rf.
  static const char *const MODES[] = {"Away", "Low", "Medium", "High", "Auto", "Auto (alternate)", "Boost", "Off"};
  char value[24];
  if (mode < sizeof(MODES) / sizeof(MODES[0]))
    std::snprintf(value, sizeof(value), "%s", MODES[mode]);
  else
    std::snprintf(value, sizeof(value), "Unknown (0x%02X)", mode);
  if (this->last_fan_mode_ == value)
    return;
  this->last_fan_mode_ = value;
  if (this->fan_mode_sensor_ != nullptr)
    this->fan_mode_sensor_->publish_state(this->last_fan_mode_);
  ESP_LOGI(TAG, "MVS %s reported fan mode %s", this->fan_id_.c_str(), value);
}

void RamsesEsp::process_application_frame_() {
  // The observations used here are checksum-validated Information frames.
  if (this->rx_message_type_ != 1)
    return;

  const uint16_t opcode = (static_cast<uint16_t>(this->rx_opcode_[0]) << 8) | this->rx_opcode_[1];
  bool from_co2 = this->address_matches_(0, this->co2_sensor_id_);
  const bool from_fan = this->address_matches_(0, this->fan_id_);
  const bool to_fan = this->address_matches_(1, this->fan_id_);

  // The fan address is supplied by the completed explicit pairing flow.
  // Learn its CO2 sensor from a paired 31E0 exchange, and a physical remote
  // from its first mode/timer command addressed to that fan. This avoids
  // embedding captured RAMSES identities or selecting unrelated RF neighbors.
  if (this->fan_id_.empty())
    return;

  if (opcode == 0x31E0 && to_fan && this->rx_payload_length_ == 8) {
    if (this->co2_sensor_id_.empty()) {
      this->co2_sensor_id_ = this->address_string_(0);
      ESP_LOGI(TAG, "Learned CO2 sensor identity from paired 31E0 traffic");
      from_co2 = true;
    }
    if (from_co2)
      this->mark_co2_sensor_seen_();
    return;
  }

  if ((opcode == 0x22F1 || opcode == 0x22F3) && to_fan && this->observed_remote_id_.empty()) {
    this->observed_remote_id_ = this->address_string_(0);
    ESP_LOGI(TAG, "Learned remote identity from a command addressed to the paired fan");
  }

  if (opcode == 0x1298 && from_co2 && this->rx_payload_length_ == 3) {
    uint16_t ppm;
    if (decode_co2_payload(this->rx_payload_, this->rx_payload_length_, &ppm)) {
      this->mark_co2_sensor_seen_();
      if (!this->has_co2_ppm_ || ppm != this->last_co2_ppm_) {
        this->has_co2_ppm_ = true;
        this->last_co2_ppm_ = ppm;
        if (this->co2_sensor_ != nullptr)
          this->co2_sensor_->publish_state(ppm);
        ESP_LOGI(TAG, "CO2 sensor %s reports %u ppm", this->co2_sensor_id_.c_str(), ppm);
      }
    } else {
      ESP_LOGW(TAG, "CO2 sensor %s reported a protocol fault value 0x%04X", this->co2_sensor_id_.c_str(), ppm);
    }
    return;
  }

  if (opcode == 0x31D9 && from_fan && this->rx_payload_length_ == 3) {
    uint8_t mode;
    if (decode_orcon_fan_mode(this->rx_payload_, this->rx_payload_length_, &mode))
      this->publish_fan_mode_(mode);
    return;
  }

  if (opcode == 0x22F3 && this->address_matches_(0, this->observed_remote_id_) && to_fan &&
      this->rx_payload_length_ == 7) {
    OrconBoostTimer timer;
    // The learned remote identity prevents other nearby installations reaching this.
    if (decode_orcon_boost_timer(this->rx_payload_, this->rx_payload_length_, &timer)) {
      if (!this->has_boost_duration_ || timer.minutes != this->last_boost_duration_minutes_) {
        this->has_boost_duration_ = true;
        this->last_boost_duration_minutes_ = timer.minutes;
        if (this->boost_duration_sensor_ != nullptr)
          this->boost_duration_sensor_->publish_state(timer.minutes);
        ESP_LOGI(TAG, "Paired remote requested %u-minute Orcon boost (speed=%u)", timer.minutes, timer.speed);
      }
    } else {
      ESP_LOGD(TAG, "Ignored unsupported 22F3 dialect from learned remote");
    }
  }
}

bool RamsesEsp::process_message_byte_(uint8_t byte) {
  this->rx_message_checksum_ = static_cast<uint8_t>(this->rx_message_checksum_ + byte);

  switch (this->rx_message_state_) {
    case RX_MSG_HEADER: {
      this->rx_message_type_ = (byte >> 4) & 0x03;
      this->rx_address_mask_ = RAMSES_ADDRESS_MASKS[(byte >> 2) & 0x03];
      this->rx_param_mask_ = static_cast<uint8_t>(((byte & 0x02) ? 0x01 : 0x00) | ((byte & 0x01) ? 0x02 : 0x00));
      this->rx_address_slot_ = 0;
      this->rx_address_byte_ = 0;
      this->rx_param_slot_ = 0;

      while (this->rx_address_slot_ < 3 && (this->rx_address_mask_ & (1U << this->rx_address_slot_)) == 0)
        this->rx_address_slot_++;
      if (this->rx_address_slot_ < 3) {
        this->rx_message_state_ = RX_MSG_ADDRESSES;
      } else if (this->rx_param_mask_ != 0) {
        this->rx_message_state_ = RX_MSG_PARAMS;
      } else {
        this->rx_message_state_ = RX_MSG_OPCODE;
      }
      return true;
    }

    case RX_MSG_ADDRESSES:
      this->rx_addresses_[this->rx_address_slot_][this->rx_address_byte_++] = byte;
      if (this->rx_address_byte_ == 3) {
        this->rx_address_byte_ = 0;
        this->rx_address_slot_++;
        while (this->rx_address_slot_ < 3 && (this->rx_address_mask_ & (1U << this->rx_address_slot_)) == 0)
          this->rx_address_slot_++;
        if (this->rx_address_slot_ == 3)
          this->rx_message_state_ = this->rx_param_mask_ != 0 ? RX_MSG_PARAMS : RX_MSG_OPCODE;
      }
      return true;

    case RX_MSG_PARAMS:
      while (this->rx_param_slot_ < 2 && (this->rx_param_mask_ & (1U << this->rx_param_slot_)) == 0)
        this->rx_param_slot_++;
      if (this->rx_param_slot_ >= 2)
        return false;
      this->rx_params_[this->rx_param_slot_++] = byte;
      if (this->rx_param_slot_ >= 2 || (this->rx_param_mask_ & (1U << this->rx_param_slot_)) == 0) {
        while (this->rx_param_slot_ < 2 && (this->rx_param_mask_ & (1U << this->rx_param_slot_)) == 0)
          this->rx_param_slot_++;
        if (this->rx_param_slot_ >= 2)
          this->rx_message_state_ = RX_MSG_OPCODE;
      }
      return true;

    case RX_MSG_OPCODE:
      this->rx_opcode_[this->rx_opcode_bytes_++] = byte;
      if (this->rx_opcode_bytes_ == 2)
        this->rx_message_state_ = RX_MSG_LENGTH;
      return true;

    case RX_MSG_LENGTH:
      if (byte > sizeof(this->rx_payload_))
        return false;
      this->rx_payload_length_ = byte;
      this->rx_payload_count_ = 0;
      this->rx_message_state_ = byte == 0 ? RX_MSG_CHECKSUM : RX_MSG_PAYLOAD;
      return true;

    case RX_MSG_PAYLOAD:
      if (this->rx_payload_count_ >= this->rx_payload_length_ || this->rx_payload_count_ >= sizeof(this->rx_payload_))
        return false;
      this->rx_payload_[this->rx_payload_count_++] = byte;
      if (this->rx_payload_count_ == this->rx_payload_length_)
        this->rx_message_state_ = RX_MSG_CHECKSUM;
      return true;

    case RX_MSG_CHECKSUM:
      this->rx_message_state_ = RX_MSG_COMPLETE;
      return true;

    case RX_MSG_COMPLETE:
    default:
      return false;
  }
}

void RamsesEsp::reset_rx_frame_() {
  this->rx_frame_active_ = false;
  this->rx_sync_buffer_ = 0;
  this->rx_have_half_nibble_ = false;
  this->rx_half_nibble_ = 0;
  this->rx_message_state_ = RX_MSG_HEADER;
  this->rx_message_checksum_ = 0;
  this->rx_message_type_ = 0;
  this->rx_address_mask_ = 0;
  this->rx_param_mask_ = 0;
  this->rx_encoded_byte_count_ = 0;
  this->rx_address_slot_ = 0;
  this->rx_address_byte_ = 0;
  this->rx_param_slot_ = 0;
  this->rx_opcode_bytes_ = 0;
  this->rx_payload_length_ = 0;
  this->rx_payload_count_ = 0;
}

void RamsesEsp::reject_rx_frame_(const char *reason) {
  this->rx_rejected_frame_count_++;
  ESP_LOGW(TAG, "Rejected RAMSES frame (count=%lu): %s",
           static_cast<unsigned long>(this->rx_rejected_frame_count_), reason);
  this->reset_rx_frame_();
}

void RamsesEsp::log_valid_frame_() {
  static const char *const TYPES[] = {"RQ", "I", "W", "RP"};
  char payload_hex[sizeof(this->rx_payload_) * 2 + 1];
  static const char HEX[] = "0123456789ABCDEF";
  for (uint8_t i = 0; i < this->rx_payload_length_; i++) {
    payload_hex[i * 2] = HEX[this->rx_payload_[i] >> 4];
    payload_hex[i * 2 + 1] = HEX[this->rx_payload_[i] & 0x0F];
  }
  payload_hex[this->rx_payload_length_ * 2] = '\0';

  char addresses[3][12];
  for (uint8_t slot = 0; slot < 3; slot++) {
    if ((this->rx_address_mask_ & (1U << slot)) == 0) {
      std::snprintf(addresses[slot], sizeof(addresses[slot]), "--:------");
    } else {
      const uint8_t *addr = this->rx_addresses_[slot];
      const uint8_t device_class = (addr[0] & 0xFC) >> 2;
      const uint32_t id = (static_cast<uint32_t>(addr[0] & 0x03) << 16) |
                          (static_cast<uint32_t>(addr[1]) << 8) | addr[2];
      std::snprintf(addresses[slot], sizeof(addresses[slot]), "%02u:%06lu", static_cast<unsigned>(device_class),
                    static_cast<unsigned long>(id));
    }
  }

  ESP_LOGI(TAG, "Valid RAMSES frame #%lu type=%s addr0=%s addr1=%s addr2=%s opcode=%02X%02X payload_len=%u payload=%s",
           static_cast<unsigned long>(this->rx_valid_frame_count_), TYPES[this->rx_message_type_], addresses[0],
           addresses[1], addresses[2], this->rx_opcode_[0], this->rx_opcode_[1], this->rx_payload_length_, payload_hex);
}

uint8_t RamsesEsp::read_register_(uint8_t address) {
  // Keep the two-byte CC1101 read as one SPI transaction, matching the
  // upstream driver. Separate TX/RX buffers avoid relying on in-place full
  // duplex behavior in the ESPHome SPI backend.
  const uint8_t tx[2] = {static_cast<uint8_t>(address | CC_READ), 0x00};
  uint8_t rx[2] = {0x00, 0x00};
  this->enable();
  this->delegate_->transfer(tx, rx, sizeof(tx));
  this->disable();
  return rx[1];
}

uint8_t RamsesEsp::write_register_(uint8_t address, uint8_t value) {
  const uint8_t tx[2] = {address, value};
  uint8_t rx[2] = {0x00, 0x00};
  this->enable();
  this->delegate_->transfer(tx, rx, sizeof(tx));
  this->disable();
  return rx[0];
}

uint8_t RamsesEsp::strobe_(uint8_t command) {
  this->enable();
  const uint8_t status = this->transfer_byte(command);
  this->disable();
  return status;
}

bool RamsesEsp::initialize_cc1101_() {
  // Port the upstream CSN reset pulse through ESPHome's registered SPI device
  // so CS remains owned by SPIDevice and the shared SPI bus is not reinitialized.
  // spi_setup() leaves CS high, as required at the start of this sequence.
  delayMicroseconds(1);
  this->enable();
  delayMicroseconds(10);
  this->disable();
  delayMicroseconds(41);

  // Reset the radio before reading status registers.
  this->strobe_(CC_SRES);

  const uint32_t reset_started = millis();
  uint8_t status = this->strobe_(CC_SNOP);
  while ((status & CC_CHIP_RDY) != 0 && (millis() - reset_started) < CC1101_RESET_TIMEOUT_MS) {
    delay(1);
    status = this->strobe_(CC_SNOP);
  }
  if ((status & CC_CHIP_RDY) != 0) {
    ESP_LOGE(TAG, "CC1101 CHIP_RDYn timeout after %u ms (status=0x%02X)",
             static_cast<unsigned>(CC1101_RESET_TIMEOUT_MS), status);
    return false;
  }

  const uint8_t part_number = this->read_register_(CC_PARTNUM);
  const uint8_t version = this->read_register_(CC_VERSION);

  // A fully floating MISO line commonly reads as all ones. Do not reject a
  // specific silicon revision: compatible CC1101 modules can report variants.
  if (part_number == 0xFF && version == 0xFF) {
    ESP_LOGE(TAG, "No CC1101 response (PARTNUM=0x%02X, VERSION=0x%02X)", part_number, version);
    return false;
  }
  ESP_LOGI(TAG, "CC1101 PARTNUM=0x%02X VERSION=0x%02X", part_number, version);

  uint8_t config[CC_PARAM_MAX];
  const uint8_t config_length = cc_cfg_get(0, config, sizeof(config));
  if (config_length != sizeof(config)) {
    ESP_LOGE(TAG, "Invalid CC1101 register profile length: %u", config_length);
    return false;
  }

  for (uint8_t reg = 0; reg < config_length; reg++) {
    this->write_register_(reg, config[reg]);
  }

  // Match upstream: program the FIFO threshold override after the full profile.
  const uint8_t fifo_threshold = (config[CC_FIFOTHR] & 0xF0) | 11;
  this->write_register_(CC_FIFOTHR, fifo_threshold);

  uint8_t pa_table[CC_PA_MAX];
  const uint8_t pa_length = cc_pa_get(pa_table);
  for (uint8_t i = 0; i < pa_length; i++) {
    this->write_register_(CC_PATABLE, pa_table[i]);
  }

  // Check the programmed RF frequency, synthesizer, modulation, and packet
  // profile. FIFOTHR uses the upstream override written after the profile.
  static const RegisterCheck checks[] = {
      {CC_IOCFG2, "IOCFG2"},     {CC_IOCFG0, "IOCFG0"},     {CC_FIFOTHR, "FIFOTHR"},
      {CC_SYNC1, "SYNC1"},       {CC_SYNC0, "SYNC0"},       {CC_PKTLEN, "PKTLEN"},
      {CC_PKTCTRL1, "PKTCTRL1"}, {CC_PKTCTRL0, "PKTCTRL0"}, {CC_CHANNR, "CHANNR"},
      {CC_FSCTRL1, "FSCTRL1"},   {CC_FSCTRL0, "FSCTRL0"},   {CC_FREQ2, "FREQ2"},
      {CC_FREQ1, "FREQ1"},       {CC_FREQ0, "FREQ0"},       {CC_MDMCFG4, "MDMCFG4"},
      {CC_MDMCFG3, "MDMCFG3"},   {CC_MDMCFG2, "MDMCFG2"},   {CC_MDMCFG1, "MDMCFG1"},
      {CC_MDMCFG0, "MDMCFG0"},   {CC_DEVIATN, "DEVIATN"},   {CC_MCSM1, "MCSM1"},
      {CC_MCSM0, "MCSM0"},       {CC_FOCCFG, "FOCCFG"},     {CC_BSCFG, "BSCFG"},
      {CC_AGCCTRL2, "AGCCTRL2"}, {CC_AGCCTRL1, "AGCCTRL1"}, {CC_AGCCTRL0, "AGCCTRL0"},
  };

  for (const auto &check : checks) {
    uint8_t expected = config[check.address];
    if (check.address == CC_FIFOTHR) {
      expected = (expected & 0xF0) | 11;
    }

    const uint8_t actual = this->read_register_(check.address);
    if (actual != expected) {
      ESP_LOGE(TAG, "CC1101 register %s (0x%02X) mismatch: expected 0x%02X, read 0x%02X",
               check.name, check.address, expected, actual);
      return false;
    }
  }

  // Leave the radio in idle. The later frame component will explicitly start
  // RX after its UART/GDO path is ready.
  this->strobe_(CC_SIDLE);
  const uint8_t marcstate = this->read_register_(CC_MARCSTATE);
  if ((marcstate & CC_MARCSTATE_MASK) != CC_MARCSTATE_IDLE) {
    ESP_LOGE(TAG, "CC1101 MARCSTATE is not IDLE after initialization: expected 0x%02X, read 0x%02X",
             CC_MARCSTATE_IDLE, marcstate & CC_MARCSTATE_MASK);
    return false;
  }
  ESP_LOGI(TAG, "CC1101 MARCSTATE=0x%02X (IDLE)", marcstate & CC_MARCSTATE_MASK);
  return true;
}

}  // namespace ramses_esp
}  // namespace esphome
