import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import DEVICE_CLASS_CONNECTIVITY

from . import RamsesEsp, ramses_esp_ns

DEPENDENCIES = ["ramses_esp"]

CONF_RAMSES_ESP_ID = "ramses_esp_id"

RamsesEspBinarySensor = ramses_esp_ns.class_("RamsesEspBinarySensor", binary_sensor.BinarySensor)

CONFIG_SCHEMA = binary_sensor.binary_sensor_schema(
    RamsesEspBinarySensor, device_class=DEVICE_CLASS_CONNECTIVITY
).extend(
    {
        cv.Required(CONF_RAMSES_ESP_ID): cv.use_id(RamsesEsp),
        cv.Required("type"): cv.one_of("co2_sensor_presence"),
    }
)


async def to_code(config):
    var = await binary_sensor.new_binary_sensor(config)
    parent = await cg.get_variable(config[CONF_RAMSES_ESP_ID])
    cg.add(parent.set_co2_presence_sensor(var))
