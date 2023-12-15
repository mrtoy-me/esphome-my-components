import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor

from . import VL53L1XComponent, CONF_VL53L1X_ID 

CONF_ABOVE_THRESHOLD = "above_threshold"
CONF_BELOW_THRESHOLD = "below_threshold"
CONF_ABOVE_DISTANCE = "above_distance"
CONF_BELOW_DISTANCE = "below_distance"
CONF_RANGE_STATUS_VALID ="range_valid"

CONFIG_SCHEMA = cv.Schema(
        {
            cv.GenerateID(CONF_VL53L1X_ID): cv.use_id(VL53L1XComponent),
            cv.Optional(CONF_RANGE_STATUS_VALID): binary_sensor.binary_sensor_schema(),
            cv.Optional(CONF_ABOVE_THRESHOLD): binary_sensor.binary_sensor_schema().extend(
                 {
                   cv.Required(CONF_ABOVE_DISTANCE ): cv.int_range(min=50, max=3500),
                 }
            ),
            cv.Optional(CONF_BELOW_THRESHOLD): binary_sensor.binary_sensor_schema().extend(
                 {
                   cv.Required(CONF_BELOW_DISTANCE): cv.int_range(min=50, max=3500),
                 }
            ),    
        }
)

async def to_code(config):
    hub = await cg.get_variable(config[CONF_VL53L1X_ID])
    if CONF_RANGE_STATUS_VALID in config:
        var = await binary_sensor.new_binary_sensor(config[CONF_RANGE_STATUS_VALID])
        cg.add(hub.set_range_valid_binary_sensor(var))

    if CONF_ABOVE_THRESHOLD in config:
        var = await binary_sensor.new_binary_sensor(config[CONF_ABOVE_THRESHOLD])
        cg.add(hub.set_above_threshold_binary_sensor(var))
        above_threshold_config = config[CONF_ABOVE_THRESHOLD]
        cg.add(hub.set_above_distance(above_threshold_config[CONF_ABOVE_DISTANCE]))

    if CONF_BELOW_THRESHOLD in config:
        var = await binary_sensor.new_binary_sensor(config[CONF_BELOW_THRESHOLD])
        cg.add(hub.set_below_threshold_binary_sensor(var))
        below_threshold_config = config[CONF_BELOW_THRESHOLD]
        cg.add(hub.set_below_distance(below_threshold_config[CONF_BELOW_DISTANCE]))