#include "ramses_esp.h"

#include "esphome/core/log.h"
#include "cc1101_param.h"

namespace esphome {
namespace ramses_esp {

static const char *const TAG = "ramses_esp";

void RamsesEsp::setup() {
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
  uint8_t data[2] = {static_cast<uint8_t>(address | CC_READ), 0x00};
  this->enable();
  this->transfer_array(data, sizeof(data));
  this->disable();
  return data[1];
}

uint8_t RamsesEsp::write_register_(uint8_t address, uint8_t value) {
  uint8_t data[2] = {address, value};
  this->enable();
  this->transfer_array(data, sizeof(data));
  this->disable();
  return data[0];
}

uint8_t RamsesEsp::strobe_(uint8_t command) {
  this->enable();
  const uint8_t status = this->transfer_byte(command);
  this->disable();
  return status;
}

bool RamsesEsp::initialize_cc1101_() {
  // spi_setup() has already set CS high. Reset the radio before reading status
  // registers, as required after power-up.
  this->strobe_(CC_SRES);
  delay(1);

  uint8_t part_number = this->read_register_(CC_PARTNUM);
  uint8_t version = this->read_register_(CC_VERSION);

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
    uint8_t value = config[reg];
    if (reg == CC_FIFOTHR) {
      // Preserve the upstream FIFO threshold setting used by the radio path.
      value = (value & 0xF0) | 11;
    }
    this->write_register_(reg, value);
  }

  uint8_t pa_table[CC_PA_MAX];
  const uint8_t pa_length = cc_pa_get(pa_table);
  for (uint8_t i = 0; i < pa_length; i++) {
    this->write_register_(CC_PATABLE, pa_table[i]);
  }

  // Read back the frequency profile so an absent or misrouted MISO line does
  // not look like a successful initialization.
  if (this->read_register_(CC_FREQ2) != config[CC_FREQ2]) {
    ESP_LOGE(TAG, "CC1101 profile readback failed (FREQ2=0x%02X)", this->read_register_(CC_FREQ2));
    return false;
  }

  // Leave the radio in idle. The later frame component will explicitly start
  // RX after its UART/GDO path is ready.
  this->strobe_(CC_SIDLE);
  return true;
}

}  // namespace ramses_esp
}  // namespace esphome
