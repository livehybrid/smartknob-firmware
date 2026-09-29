import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_POSITION,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
)

from . import CONF_SMARTKNOB_ID, SmartKnob

DEPENDENCIES = ["smartknob"]

CONF_STRAIN = "strain"
CONF_LOOP_TIME = "loop_time"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_SMARTKNOB_ID): cv.use_id(SmartKnob),
        # Current detent position.
        cv.Optional(CONF_POSITION): sensor.sensor_schema(
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            icon="mdi:knob",
        ),
        # Press signal (HX711 counts from baseline), for tuning the thresholds.
        cv.Optional(CONF_STRAIN): sensor.sensor_schema(
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:gesture-tap",
        ),
        # Longest control-loop iteration over the last 10 s.
        cv.Optional(CONF_LOOP_TIME): sensor.sensor_schema(
            unit_of_measurement="µs",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:timer-outline",
        ),
    }
)


async def to_code(config):
    knob = await cg.get_variable(config[CONF_SMARTKNOB_ID])
    for key, setter in (
        (CONF_POSITION, knob.set_position_sensor),
        (CONF_STRAIN, knob.set_strain_sensor),
        (CONF_LOOP_TIME, knob.set_loop_time_sensor),
    ):
        if conf := config.get(key):
            cg.add(setter(await sensor.new_sensor(conf)))
