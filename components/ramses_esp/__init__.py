import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import spi
from esphome.const import CONF_CS_PIN, CONF_ID
from esphome.core import CORE
import logging

DEPENDENCIES = ["spi"]

ramses_esp_ns = cg.esphome_ns.namespace("ramses_esp")
RamsesEsp = ramses_esp_ns.class_("RamsesEsp", cg.Component, spi.SPIDevice)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(RamsesEsp),
    }
).extend(spi.spi_device_schema(cs_pin_required=True)).extend(cv.COMPONENT_SCHEMA)

_SPI_FINAL_VALIDATE_SCHEMA = spi.final_validate_device_schema(
    "ramses_esp", require_mosi=True, require_miso=True
)

_LOGGER = logging.getLogger(__name__)
_ESP32_STRAPPING_PINS = {0, 2, 5, 12, 15}


def _validate_strapping_pin(config):
    config = _SPI_FINAL_VALIDATE_SCHEMA(config)
    cs_pin = config[CONF_CS_PIN]
    pin_number = cs_pin.get("number") if isinstance(cs_pin, dict) else cs_pin
    if CORE.is_esp32 and pin_number in _ESP32_STRAPPING_PINS:
        _LOGGER.warning(
            "ramses_esp CSN is GPIO%d, an ESP32 strapping pin. Verify the external "
            "CSN level during reset on the assembled board; this warning does not "
            "indicate a confirmed boot failure",
            pin_number,
        )
    return config


FINAL_VALIDATE_SCHEMA = _validate_strapping_pin


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await spi.register_spi_device(var, config)
