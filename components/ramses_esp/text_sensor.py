import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor

from . import RamsesEsp, ramses_esp_ns

DEPENDENCIES = ["ramses_esp"]

CONF_RAMSES_ESP_ID = "ramses_esp_id"

RamsesEspTextSensor = ramses_esp_ns.class_("RamsesEspTextSensor", text_sensor.TextSensor)

CONFIG_SCHEMA = text_sensor.text_sensor_schema(RamsesEspTextSensor, icon="mdi:fan-auto").extend(
    {
        cv.Required(CONF_RAMSES_ESP_ID): cv.use_id(RamsesEsp),
        cv.Required("type"): cv.one_of("fan_mode"),
    }
)


async def to_code(config):
    var = await text_sensor.new_text_sensor(config)
    parent = await cg.get_variable(config[CONF_RAMSES_ESP_ID])
    cg.add(parent.set_fan_mode_sensor(var))
