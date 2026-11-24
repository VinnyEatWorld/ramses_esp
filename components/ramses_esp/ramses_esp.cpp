#include "ramses_esp.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "cc1101_param.h"

namespace esphome {
namespace ramses_esp {

static const char *const TAG = "ramses_esp";
static const uint32_t CC1101_RESET_TIMEOUT_MS = 100;
static const uint8_t CC_MARCSTATE_IDLE = 0x01;
static const uint8_t CC_MARCSTATE_MASK = 0x1F;

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

  ESP_LOGI(TAG, "CC1101 initialized with the RAMSES 868.3 MHz register profile");
}

void RamsesEsp::dump_config() {
  ESP_LOGCONFIG(TAG, "RAMSES ESP:");
  ESP_LOGCONFIG(TAG, "  CC1101 SPI: mode 0, MSB first, 10 MHz");
  ESP_LOGCONFIG(TAG, "  Radio profile: 868.3 MHz (initialization only; RX/TX is not started)");
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
