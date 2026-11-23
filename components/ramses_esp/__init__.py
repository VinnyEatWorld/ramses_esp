import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import spi
from esphome.const import CONF_ID

DEPENDENCIES = ["spi"]

ramses_esp_ns = cg.esphome_ns.namespace("ramses_esp")
RamsesEsp = ramses_esp_ns.class_("RamsesEsp", cg.Component, spi.SPIDevice)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(RamsesEsp),
    }
).extend(spi.spi_device_schema(cs_pin_required=True)).extend(cv.COMPONENT_SCHEMA)

FINAL_VALIDATE_SCHEMA = spi.final_validate_device_schema(
    "ramses_esp", require_mosi=True, require_miso=True
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await spi.register_spi_device(var, config)
