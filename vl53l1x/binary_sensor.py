import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor

from . import VL53L1XComponent, CONF_VL53L1X_ID 

CONF_THRESHOLD_EXCEEDED = "threshold_exceeded"
CONF_ABOVE_ALARM = "above_alarm"

CONFIG_SCHEMA = cv.Schema(
        {
            cv.GenerateID(CONF_VL53L1X_ID): cv.use_id(VL53L1XComponent),
            cv.Required(CONF_THRESHOLD_EXCEEDED): binary_sensor.binary_sensor_schema(),
            cv.Required(CONF_ABOVE_ALARM): cv.int_range(min=50, max=1200),
        }
)

async def to_code(config):
    hub = await cg.get_variable(config[CONF_VL53L1X_ID])

    if CONF_ABOVE_ALARM in config:
       cg.add(hub.set_above_alarm(config[CONF_ABOVE_ALARM]))

    if CONF_THRESHOLD_EXCEEDED in config:
        var = await binary_sensor.new_binary_sensor(config[CONF_THRESHOLD_EXCEEDED])
        cg.add(hub.set_threshold_exceeded_binary_sensor(var))
