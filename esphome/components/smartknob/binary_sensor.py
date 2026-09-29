import esphome.codegen as cg
from esphome.components import binary_sensor
import esphome.config_validation as cv

from . import CONF_SMARTKNOB_ID, SmartKnob

DEPENDENCIES = ["smartknob"]

CONF_PRESSED = "pressed"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_SMARTKNOB_ID): cv.use_id(SmartKnob),
        # Knob pressed (strain gauge). Use on_click / on_multi_click here for
        # short and long presses.
        cv.Optional(CONF_PRESSED): binary_sensor.binary_sensor_schema(
            icon="mdi:gesture-tap-button",
        ),
    }
)


async def to_code(config):
    knob = await cg.get_variable(config[CONF_SMARTKNOB_ID])
    if conf := config.get(CONF_PRESSED):
        cg.add(knob.set_pressed_binary_sensor(await binary_sensor.new_binary_sensor(conf)))
