import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_ID,
    CONF_TEMPERATURE_SOURCE,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_MEASUREMENT,
    UNIT_PARTS_PER_BILLION,
    UNIT_PARTS_PER_MILLION,
    UNIT_PERCENT,
    UNIT_CELSIUS,
)

CODEOWNERS = ["@mrtoy-me"]
DEPENDENCIES = ["i2c"]

ens160_ns = cg.esphome_ns.namespace("ens160")
ENS160Component = ens160_ns.class_(
    "ENS160Component", cg.PollingComponent, i2c.I2CDevice, sensor.Sensor
)

CONF_COMPENSATION = "compensation"
CONF_HUMIDITY_SOURCE = "humidity_source"
CONF_AQI = "aqi"
CONF_TVOC = "tvoc"
CONF_ECO2 = "eco2"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ENS160Component),
            cv.Required(CONF_AQI): sensor.sensor_schema(
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_TVOC): sensor.sensor_schema(
                unit_of_measurement=UNIT_PARTS_PER_BILLION,
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_ECO2): sensor.sensor_schema(
                unit_of_measurement=UNIT_PARTS_PER_MILLION,
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_COMPENSATION): cv.Schema(
                {
                    cv.Required(CONF_TEMPERATURE_SOURCE): cv.use_id(sensor.Sensor),
                    cv.Required(CONF_HUMIDITY_SOURCE): cv.use_id(sensor.Sensor),
                }
            ),
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x53))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    
    if CONF_AQI in config:
       sens = await sensor.new_sensor(config[CONF_AQI])
       cg.add(var.set_aqi_sensor(sens))
    if CONF_TVOC in config:
       sens = await sensor.new_sensor(config[CONF_TVOC])
       cg.add(var.set_tvoc_sensor(sens))
    if CONF_ECO2 in config:
       sens = await sensor.new_sensor(config[CONF_ECO2])
       cg.add(var.set_eco2_sensor(sens))
    if CONF_COMPENSATION in config:
        compensation_config = config[CONF_COMPENSATION]
        sens = await cg.get_variable(compensation_config[CONF_TEMPERATURE_SOURCE])
        cg.add(var.set_temperature_sensor(sens))
        sens = await cg.get_variable(compensation_config[CONF_HUMIDITY_SOURCE])
        cg.add(var.set_humidity_sensor(sens))
        