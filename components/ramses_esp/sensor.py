import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    DEVICE_CLASS_CARBON_DIOXIDE,
    STATE_CLASS_MEASUREMENT,
    UNIT_PARTS_PER_MILLION,
)

from . import RamsesEsp, ramses_esp_ns

DEPENDENCIES = ["ramses_esp"]

CONF_RAMSES_ESP_ID = "ramses_esp_id"
CONF_TYPE = "type"

RamsesEspSensor = ramses_esp_ns.class_("RamsesEspSensor", sensor.Sensor)

_COMMON = {cv.Required(CONF_RAMSES_ESP_ID): cv.use_id(RamsesEsp)}

CONFIG_SCHEMA = cv.Any(
    sensor.sensor_schema(
        RamsesEspSensor,
        unit_of_measurement=UNIT_PARTS_PER_MILLION,
        accuracy_decimals=0,
        device_class=DEVICE_CLASS_CARBON_DIOXIDE,
        state_class=STATE_CLASS_MEASUREMENT,
        icon="mdi:molecule-co2",
    ).extend({**_COMMON, cv.Required(CONF_TYPE): cv.one_of("co2")}),
    sensor.sensor_schema(
        RamsesEspSensor,
        unit_of_measurement="min",
        accuracy_decimals=0,
        state_class=STATE_CLASS_MEASUREMENT,
        icon="mdi:timer-outline",
    ).extend({**_COMMON, cv.Required(CONF_TYPE): cv.one_of("boost_duration")}),
)


async def to_code(config):
    var = await sensor.new_sensor(config)
    parent = await cg.get_variable(config[CONF_RAMSES_ESP_ID])
    if config[CONF_TYPE] == "co2":
        cg.add(parent.set_co2_sensor(var))
    else:
        cg.add(parent.set_boost_duration_sensor(var))
