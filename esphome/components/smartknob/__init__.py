"""SmartKnob haptic knob: BLDC motor + MT6701 encoder (+ optional HX711 press sensor).

Runs field-oriented control and a detent engine in a dedicated real-time task
and exposes the knob to ESPHome as triggers, actions and entities.
"""

import math

from esphome import automation, pins
import esphome.codegen as cg
from esphome.components import ota
from esphome.components.esp32 import (
    VARIANT_ESP32,
    VARIANT_ESP32C5,
    VARIANT_ESP32C6,
    VARIANT_ESP32H2,
    VARIANT_ESP32P4,
    VARIANT_ESP32S3,
    include_builtin_idf_component,
    only_on_variant,
)
import esphome.config_validation as cv
from esphome.const import (
    CONF_CALIBRATION,
    CONF_CLK_PIN,
    CONF_CS_PIN,
    CONF_DATA_PIN,
    CONF_DIRECTION,
    CONF_DURATION,
    CONF_FILTER,
    CONF_FREQUENCY,
    CONF_ID,
    CONF_ON_PRESS,
    CONF_ON_RELEASE,
    CONF_POSITION,
    CONF_PRESET,
)
import esphome.final_validate as fv

CODEOWNERS = []
DEPENDENCIES = ["esp32"]

smartknob_ns = cg.esphome_ns.namespace("smartknob")
SmartKnob = smartknob_ns.class_("SmartKnob", cg.Component)

SetProfileAction = smartknob_ns.class_(
    "SetProfileAction", automation.Action, cg.Parented.template(SmartKnob)
)
SetPositionAction = smartknob_ns.class_(
    "SetPositionAction", automation.Action, cg.Parented.template(SmartKnob)
)
ClickAction = smartknob_ns.class_(
    "ClickAction", automation.Action, cg.Parented.template(SmartKnob)
)
CalibrateAction = smartknob_ns.class_(
    "CalibrateAction", automation.Action, cg.Parented.template(SmartKnob)
)
SetHapticsAction = smartknob_ns.class_(
    "SetHapticsAction", automation.Action, cg.Parented.template(SmartKnob)
)
SetPressThresholdsAction = smartknob_ns.class_(
    "SetPressThresholdsAction", automation.Action, cg.Parented.template(SmartKnob)
)
SetLongPressTimeAction = smartknob_ns.class_(
    "SetLongPressTimeAction", automation.Action, cg.Parented.template(SmartKnob)
)

CONF_SMARTKNOB_ID = "smartknob_id"
CONF_MOTOR = "motor"
CONF_UH_PIN = "uh_pin"
CONF_UL_PIN = "ul_pin"
CONF_VH_PIN = "vh_pin"
CONF_VL_PIN = "vl_pin"
CONF_WH_PIN = "wh_pin"
CONF_WL_PIN = "wl_pin"
CONF_PWM_FREQUENCY = "pwm_frequency"
CONF_DEAD_TIME = "dead_time"
CONF_SUPPLY_VOLTAGE = "supply_voltage"
CONF_VOLTAGE_LIMIT = "voltage_limit"
CONF_CALIBRATION_VOLTAGE = "calibration_voltage"
CONF_POLE_PAIRS = "pole_pairs"
CONF_ZERO_OFFSET = "zero_offset"
CONF_ENCODER = "encoder"
CONF_SPI_HOST = "spi_host"
CONF_STRAIN_GAUGE = "strain_gauge"
CONF_DOUT_PIN = "dout_pin"
CONF_PRESS_THRESHOLD = "press_threshold"
CONF_RELEASE_THRESHOLD = "release_threshold"
CONF_PRESS_CLICK = "press_click"
CONF_RELEASE_CLICK = "release_click"
CONF_CLICK_DURATION = "click_duration"
CONF_MAX_VELOCITY = "max_velocity"
CONF_INVERT_DIRECTION = "invert_direction"
CONF_CONTROL_FREQUENCY = "control_frequency"
CONF_TASK_CORE = "task_core"
CONF_TASK_PRIORITY = "task_priority"
CONF_AUTO_CALIBRATE = "auto_calibrate"
CONF_INITIAL_PROFILE = "initial_profile"
CONF_MIN_POSITION = "min_position"
CONF_MAX_POSITION = "max_position"
CONF_POSITION_WIDTH = "position_width"
CONF_DETENT_STRENGTH = "detent_strength"
CONF_ENDSTOP_STRENGTH = "endstop_strength"
CONF_SNAP_POINT = "snap_point"
CONF_SNAP_POINT_BIAS = "snap_point_bias"
CONF_DETENT_POSITIONS = "detent_positions"
CONF_ON_POSITION_CHANGE = "on_position_change"
CONF_ON_CALIBRATION = "on_calibration"
CONF_ON_SHORT_PRESS = "on_short_press"
CONF_ON_LONG_PRESS = "on_long_press"
CONF_LONG_PRESS_TIME = "long_press_time"
CONF_STRENGTH = "strength"
CONF_ENABLED = "enabled"
CONF_PRESS = "press"
CONF_RELEASE = "release"

MAX_DETENT_POSITIONS = 8

# Haptic presets from Scott Bezek's original SmartKnob demo configurations.
# Widths in degrees; max_position < min_position means unbounded.
PROFILE_DEFAULTS = {
    CONF_MIN_POSITION: 0,
    CONF_MAX_POSITION: -1,
    CONF_POSITION_WIDTH: 10.0,
    CONF_DETENT_STRENGTH: 0.0,
    CONF_ENDSTOP_STRENGTH: 1.0,
    CONF_SNAP_POINT: 1.1,
    CONF_SNAP_POINT_BIAS: 0.0,
    CONF_DETENT_POSITIONS: [],
}


def _preset(**values):
    return {**PROFILE_DEFAULTS, **values}


PRESETS = {
    "unbounded": _preset(),
    "unbounded_detents": _preset(detent_strength=1.0),
    "bounded_0_10": _preset(max_position=10),
    "multi_rev": _preset(max_position=72),
    "on_off": _preset(
        max_position=1, position_width=60.0, detent_strength=1.0, snap_point=0.55
    ),
    "return_to_centre": _preset(
        max_position=0, position_width=60.0, detent_strength=0.01, endstop_strength=0.6
    ),
    "fine": _preset(max_position=255, position_width=1.0),
    "fine_detents": _preset(max_position=255, position_width=1.0, detent_strength=1.0),
    "coarse_strong": _preset(
        max_position=31, position_width=8.225806452, detent_strength=2.0
    ),
    "coarse_weak": _preset(
        max_position=31, position_width=8.225806452, detent_strength=0.2
    ),
    "magnetic": _preset(
        max_position=31,
        position_width=7.0,
        detent_strength=2.5,
        snap_point=0.7,
        detent_positions=[2, 10, 21, 22],
    ),
    "return_to_centre_detents": _preset(
        min_position=-6,
        max_position=6,
        position_width=60.0,
        detent_strength=1.0,
        snap_point=0.55,
        snap_point_bias=0.4,
    ),
}

PROFILE_FIELD_VALIDATORS = {
    CONF_MIN_POSITION: cv.int_,
    CONF_MAX_POSITION: cv.int_,
    CONF_POSITION_WIDTH: cv.All(cv.angle, cv.Range(min=0.5, max=180.0)),
    CONF_DETENT_STRENGTH: cv.float_range(min=0.0, max=5.0),
    CONF_ENDSTOP_STRENGTH: cv.float_range(min=0.0, max=5.0),
    CONF_SNAP_POINT: cv.float_range(min=0.5, max=5.0),
    CONF_SNAP_POINT_BIAS: cv.float_range(min=0.0, max=1.0),
}


def _check_profile(config):
    """Static checks for a profile whose fields are all constants."""
    values = {**PROFILE_DEFAULTS, **PRESETS.get(config.get(CONF_PRESET), {}), **config}
    if any(cg.is_template(values[k]) for k in PROFILE_FIELD_VALIDATORS):
        return config
    bounded = values[CONF_MAX_POSITION] >= values[CONF_MIN_POSITION]
    if values[CONF_DETENT_POSITIONS] and not bounded:
        raise cv.Invalid("detent_positions requires min_position <= max_position")
    position = config.get(CONF_POSITION)
    if (
        position is not None
        and not cg.is_template(position)
        and bounded
        and not values[CONF_MIN_POSITION] <= position <= values[CONF_MAX_POSITION]
    ):
        raise cv.Invalid(
            f"position {position} is outside {values[CONF_MIN_POSITION]}..{values[CONF_MAX_POSITION]}"
        )
    return config


def _profile_fields(templatable: bool) -> dict:
    wrap = cv.templatable if templatable else (lambda v: v)
    fields = {
        cv.Optional(CONF_PRESET): cv.one_of(*PRESETS, lower=True, space="_"),
        cv.Optional(CONF_DETENT_POSITIONS): cv.All(
            cv.ensure_list(cv.int_), cv.Length(max=MAX_DETENT_POSITIONS)
        ),
        cv.Optional(CONF_POSITION): wrap(cv.int_),
    }
    for key, validator in PROFILE_FIELD_VALIDATORS.items():
        fields[cv.Optional(key)] = wrap(validator)
    return fields


def _resolve_profile(config):
    values = dict(PROFILE_DEFAULTS)
    values.update(PRESETS.get(config.get(CONF_PRESET), {}))
    values.update({k: v for k, v in config.items() if k != CONF_PRESET})
    return values


def _gpio_out(value):
    return pins.internal_gpio_output_pin_number(value)


def _gpio_in(value):
    return pins.internal_gpio_input_pin_number(value)


MOTOR_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_UH_PIN): _gpio_out,
        cv.Required(CONF_UL_PIN): _gpio_out,
        cv.Required(CONF_VH_PIN): _gpio_out,
        cv.Required(CONF_VL_PIN): _gpio_out,
        cv.Required(CONF_WH_PIN): _gpio_out,
        cv.Required(CONF_WL_PIN): _gpio_out,
        cv.Optional(CONF_PWM_FREQUENCY, default="25kHz"): cv.All(
            cv.frequency, cv.Range(min=10000, max=60000)
        ),
        cv.Optional(CONF_DEAD_TIME, default="500ns"): cv.All(
            cv.positive_time_period_nanoseconds,
            cv.Range(min=cv.TimePeriod(nanoseconds=100), max=cv.TimePeriod(nanoseconds=2000)),
        ),
        cv.Optional(CONF_SUPPLY_VOLTAGE, default="5V"): cv.All(
            cv.voltage, cv.Range(min=3.0, max=12.0)
        ),
        # Largest q-axis voltage applied. Space-vector PWM tops out at
        # supply / sqrt(3) (2.89 V at 5 V); higher values just saturate.
        cv.Optional(CONF_VOLTAGE_LIMIT): cv.All(cv.voltage, cv.Range(min=0.5, max=12.0)),
        cv.Optional(CONF_CALIBRATION_VOLTAGE, default="2.5V"): cv.All(
            cv.voltage, cv.Range(min=0.5, max=6.0)
        ),
        # Pin a calibration here to skip the automatic one (values are logged
        # after every successful calibration).
        cv.Optional(CONF_CALIBRATION): cv.Schema(
            {
                cv.Required(CONF_POLE_PAIRS): cv.int_range(min=3, max=12),
                cv.Required(CONF_DIRECTION): cv.one_of(1, -1, int=True),
                cv.Required(CONF_ZERO_OFFSET): cv.float_range(
                    min=-math.pi - 1e-3, max=math.pi + 1e-3
                ),
            }
        ),
    }
)

ENCODER_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_CLK_PIN): _gpio_out,
        cv.Required(CONF_DATA_PIN): _gpio_in,
        cv.Required(CONF_CS_PIN): _gpio_out,
        # The encoder takes a whole SPI peripheral. SPI3 by default so that a
        # display bus left on "interface: any" still gets SPI2.
        cv.Optional(CONF_SPI_HOST, default="spi3"): cv.one_of("spi2", "spi3", lower=True),
        cv.Optional(CONF_FREQUENCY, default="4MHz"): cv.All(
            cv.frequency, cv.Range(min=100e3, max=15e6)
        ),
        # Low-pass on the angle used by the detent engine (0 disables).
        cv.Optional(CONF_FILTER, default="300Hz"): cv.All(
            cv.frequency, cv.Range(min=0, max=2000)
        ),
    }
)

STRAIN_DIRECTIONS = {"both": 0, "positive": 1, "negative": -1}

STRAIN_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_DOUT_PIN): _gpio_in,
        cv.Required(CONF_CLK_PIN): _gpio_out,
        # Raw HX711 counts away from the idle baseline.
        cv.Optional(CONF_PRESS_THRESHOLD, default=30000): cv.positive_float,
        cv.Optional(CONF_RELEASE_THRESHOLD, default=15000): cv.positive_float,
        cv.Optional(CONF_DIRECTION, default="both"): cv.enum(STRAIN_DIRECTIONS, lower=True),
        cv.Optional(CONF_PRESS_CLICK, default=2.5): cv.float_range(min=0.0, max=6.0),
        cv.Optional(CONF_RELEASE_CLICK, default=1.5): cv.float_range(min=0.0, max=6.0),
        cv.Optional(
            CONF_CLICK_DURATION, default="6ms"
        ): cv.positive_time_period_microseconds,
        # Presses are ignored while the knob turns faster than this (rad/s).
        cv.Optional(CONF_MAX_VELOCITY, default=8.0): cv.positive_float,
    }
)


def _validate_strain(config):
    strain = config.get(CONF_STRAIN_GAUGE)
    if strain and strain[CONF_RELEASE_THRESHOLD] >= strain[CONF_PRESS_THRESHOLD]:
        raise cv.Invalid(
            "strain_gauge: release_threshold must be below press_threshold"
        )
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SmartKnob),
            cv.Required(CONF_MOTOR): MOTOR_SCHEMA,
            cv.Required(CONF_ENCODER): ENCODER_SCHEMA,
            cv.Optional(CONF_STRAIN_GAUGE): STRAIN_SCHEMA,
            # Flip if turning clockwise lowers the position.
            cv.Optional(CONF_INVERT_DIRECTION, default=False): cv.boolean,
            # The loop code runs from flash; faster rates leave less of core 1
            # for ESPHome (the task backs off if it overruns).
            cv.Optional(CONF_CONTROL_FREQUENCY, default="1kHz"): cv.All(
                cv.frequency, cv.Range(min=500, max=5000)
            ),
            cv.Optional(CONF_TASK_CORE, default=1): cv.int_range(min=0, max=1),
            cv.Optional(CONF_TASK_PRIORITY, default=20): cv.int_range(min=2, max=24),
            cv.Optional(CONF_AUTO_CALIBRATE, default=True): cv.boolean,
            cv.Optional(CONF_INITIAL_PROFILE, default={}): cv.All(
                cv.Schema(_profile_fields(False)), _check_profile
            ),
            cv.Optional(CONF_ON_POSITION_CHANGE): automation.validate_automation({}),
            cv.Optional(
                CONF_LONG_PRESS_TIME, default="350ms"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_ON_PRESS): automation.validate_automation({}),
            cv.Optional(CONF_ON_RELEASE): automation.validate_automation({}),
            cv.Optional(CONF_ON_SHORT_PRESS): automation.validate_automation({}),
            cv.Optional(CONF_ON_LONG_PRESS): automation.validate_automation({}),
            cv.Optional(CONF_ON_CALIBRATION): automation.validate_automation({}),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    only_on_variant(
        supported=[
            VARIANT_ESP32,
            VARIANT_ESP32S3,
            VARIANT_ESP32C5,
            VARIANT_ESP32C6,
            VARIANT_ESP32H2,
            VARIANT_ESP32P4,
        ],
        msg_prefix="smartknob (needs MCPWM)",
    ),
    _validate_strain,
)


def _final_validate(config):
    # The encoder owns its SPI peripheral outright: no spi: bus may share it.
    host_index = {"spi2": 0, "spi3": 1}[config[CONF_ENCODER][CONF_SPI_HOST]]
    for bus in fv.full_config.get().get("spi", []):
        if bus.get("interface_index") == host_index:
            other = "spi2" if host_index == 1 else "spi3"
            raise cv.Invalid(
                f"The SmartKnob encoder uses {config[CONF_ENCODER][CONF_SPI_HOST].upper()}, "
                f"but spi bus '{bus[CONF_ID]}' is on it too. Set 'interface: {other}' "
                f"on that bus, or change encoder: spi_host."
            )
    return config


FINAL_VALIDATE_SCHEMA = _final_validate

_CALLBACK_AUTOMATIONS = (
    automation.CallbackAutomation(
        CONF_ON_POSITION_CHANGE, "add_on_position_callback", [(cg.int32, "x")]
    ),
    automation.CallbackAutomation(CONF_ON_PRESS, "add_on_press_callback"),
    automation.CallbackAutomation(CONF_ON_RELEASE, "add_on_release_callback"),
    automation.CallbackAutomation(CONF_ON_SHORT_PRESS, "add_on_short_press_callback"),
    automation.CallbackAutomation(CONF_ON_LONG_PRESS, "add_on_long_press_callback"),
    automation.CallbackAutomation(
        CONF_ON_CALIBRATION, "add_on_calibration_callback", [(cg.bool_, "success")]
    ),
)


async def to_code(config):
    include_builtin_idf_component("esp_driver_mcpwm")
    include_builtin_idf_component("esp_driver_gptimer")
    # Coast the motor while an OTA update writes flash (no-op without ota:).
    ota.request_ota_state_listeners()

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    motor = config[CONF_MOTOR]
    cg.add(
        var.set_motor_pins(
            motor[CONF_UH_PIN],
            motor[CONF_UL_PIN],
            motor[CONF_VH_PIN],
            motor[CONF_VL_PIN],
            motor[CONF_WH_PIN],
            motor[CONF_WL_PIN],
        )
    )
    cg.add(var.set_pwm_frequency(int(motor[CONF_PWM_FREQUENCY])))
    cg.add(var.set_dead_time_ns(int(motor[CONF_DEAD_TIME].total_nanoseconds)))
    supply = motor[CONF_SUPPLY_VOLTAGE]
    cg.add(var.set_supply_voltage(supply))
    limit = motor.get(CONF_VOLTAGE_LIMIT, round(supply / math.sqrt(3.0), 2))
    cg.add(var.set_voltage_limit(limit))
    cg.add(var.set_calibration_voltage(motor[CONF_CALIBRATION_VOLTAGE]))
    if calibration := motor.get(CONF_CALIBRATION):
        cg.add(
            var.set_static_calibration(
                calibration[CONF_POLE_PAIRS],
                calibration[CONF_DIRECTION],
                calibration[CONF_ZERO_OFFSET],
            )
        )

    encoder = config[CONF_ENCODER]
    cg.add(
        var.set_encoder_pins(
            encoder[CONF_CLK_PIN], encoder[CONF_DATA_PIN], encoder[CONF_CS_PIN]
        )
    )
    cg.add(var.set_encoder_spi_host(3 if encoder[CONF_SPI_HOST] == "spi3" else 2))
    cg.add(var.set_encoder_clock(int(encoder[CONF_FREQUENCY])))
    cg.add(var.set_sensor_filter(encoder[CONF_FILTER]))

    if strain := config.get(CONF_STRAIN_GAUGE):
        cg.add(var.set_strain_pins(strain[CONF_DOUT_PIN], strain[CONF_CLK_PIN]))
        cg.add(
            var.set_press_thresholds(
                strain[CONF_PRESS_THRESHOLD], strain[CONF_RELEASE_THRESHOLD]
            )
        )
        cg.add(var.set_strain_sign(strain[CONF_DIRECTION]))
        cg.add(
            var.set_press_clicks(
                strain[CONF_PRESS_CLICK],
                strain[CONF_RELEASE_CLICK],
                int(strain[CONF_CLICK_DURATION].total_microseconds),
            )
        )
        cg.add(var.set_press_max_velocity(strain[CONF_MAX_VELOCITY]))

    cg.add(var.set_invert(config[CONF_INVERT_DIRECTION]))
    cg.add(var.set_control_frequency(int(config[CONF_CONTROL_FREQUENCY])))
    cg.add(var.set_task(config[CONF_TASK_CORE], config[CONF_TASK_PRIORITY]))
    cg.add(var.set_auto_calibrate(config[CONF_AUTO_CALIBRATE]))
    cg.add(var.set_long_press_time(config[CONF_LONG_PRESS_TIME].total_milliseconds))

    profile = _resolve_profile(config[CONF_INITIAL_PROFILE])
    cg.add(
        var.set_initial_profile(
            profile[CONF_MIN_POSITION],
            profile[CONF_MAX_POSITION],
            math.radians(profile[CONF_POSITION_WIDTH]),
            profile[CONF_DETENT_STRENGTH],
            profile[CONF_ENDSTOP_STRENGTH],
            profile[CONF_SNAP_POINT],
            profile[CONF_SNAP_POINT_BIAS],
            profile.get(CONF_POSITION, 0),
        )
    )
    for position in profile[CONF_DETENT_POSITIONS]:
        cg.add(var.add_initial_detent_position(position))

    await automation.build_callback_automations(var, config, _CALLBACK_AUTOMATIONS)


# ---------------------------------------------------------------- actions ----

KNOB_ID_SCHEMA = cv.Schema({cv.GenerateID(): cv.use_id(SmartKnob)})


@automation.register_action(
    "smartknob.set_profile",
    SetProfileAction,
    cv.All(KNOB_ID_SCHEMA.extend(_profile_fields(True)), _check_profile),
    synchronous=True,
)
async def set_profile_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    profile = _resolve_profile({k: v for k, v in config.items() if k != CONF_ID})
    for key, setter, type_ in (
        (CONF_MIN_POSITION, var.set_min_position, cg.int32),
        (CONF_MAX_POSITION, var.set_max_position, cg.int32),
        (CONF_POSITION_WIDTH, var.set_position_width, cg.float_),
        (CONF_DETENT_STRENGTH, var.set_detent_strength, cg.float_),
        (CONF_ENDSTOP_STRENGTH, var.set_endstop_strength, cg.float_),
        (CONF_SNAP_POINT, var.set_snap_point, cg.float_),
        (CONF_SNAP_POINT_BIAS, var.set_snap_point_bias, cg.float_),
    ):
        cg.add(setter(await cg.templatable(profile[key], args, type_)))
    if CONF_POSITION in profile:
        cg.add(var.set_position(await cg.templatable(profile[CONF_POSITION], args, cg.int32)))
    if profile[CONF_DETENT_POSITIONS]:
        cg.add(var.set_detent_positions(profile[CONF_DETENT_POSITIONS]))
    return var


@automation.register_action(
    "smartknob.set_position",
    SetPositionAction,
    cv.maybe_simple_value(
        KNOB_ID_SCHEMA.extend({cv.Required(CONF_POSITION): cv.templatable(cv.int_)}),
        key=CONF_POSITION,
    ),
    synchronous=True,
)
async def set_position_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    cg.add(var.set_position(await cg.templatable(config[CONF_POSITION], args, cg.int32)))
    return var


@automation.register_action(
    "smartknob.click",
    ClickAction,
    KNOB_ID_SCHEMA.extend(
        {
            cv.Optional(CONF_STRENGTH, default=1.5): cv.templatable(
                cv.float_range(min=0.0, max=6.0)
            ),
            cv.Optional(CONF_DURATION, default="6ms"): cv.templatable(
                cv.positive_time_period_microseconds
            ),
        }
    ),
    synchronous=True,
)
async def click_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    cg.add(var.set_strength(await cg.templatable(config[CONF_STRENGTH], args, cg.float_)))
    duration = config[CONF_DURATION]
    if not cg.is_template(duration):
        duration = int(duration.total_microseconds)
    cg.add(var.set_duration(await cg.templatable(duration, args, cg.uint32)))
    return var


@automation.register_action(
    "smartknob.calibrate", CalibrateAction, KNOB_ID_SCHEMA, synchronous=True
)
async def calibrate_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var


@automation.register_action(
    "smartknob.set_haptics",
    SetHapticsAction,
    cv.maybe_simple_value(
        KNOB_ID_SCHEMA.extend({cv.Required(CONF_ENABLED): cv.templatable(cv.boolean)}),
        key=CONF_ENABLED,
    ),
    synchronous=True,
)
async def set_haptics_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    cg.add(var.set_enabled(await cg.templatable(config[CONF_ENABLED], args, cg.bool_)))
    return var


@automation.register_action(
    "smartknob.set_press_thresholds",
    SetPressThresholdsAction,
    cv.All(
        KNOB_ID_SCHEMA.extend(
            {
                cv.Optional(CONF_PRESS): cv.templatable(cv.positive_float),
                cv.Optional(CONF_RELEASE): cv.templatable(cv.positive_float),
            }
        ),
        cv.has_at_least_one_key(CONF_PRESS, CONF_RELEASE),
    ),
    synchronous=True,
)
async def set_press_thresholds_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    if CONF_PRESS in config:
        cg.add(var.set_press(await cg.templatable(config[CONF_PRESS], args, cg.float_)))
    if CONF_RELEASE in config:
        cg.add(var.set_release(await cg.templatable(config[CONF_RELEASE], args, cg.float_)))
    return var


@automation.register_action(
    "smartknob.set_long_press_time",
    SetLongPressTimeAction,
    cv.maybe_simple_value(
        KNOB_ID_SCHEMA.extend(
            {cv.Required(CONF_LONG_PRESS_TIME): cv.templatable(cv.positive_time_period_milliseconds)}
        ),
        key=CONF_LONG_PRESS_TIME,
    ),
    synchronous=True,
)
async def set_long_press_time_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    time = config[CONF_LONG_PRESS_TIME]
    if not cg.is_template(time):
        time = int(time.total_milliseconds)
    cg.add(var.set_time(await cg.templatable(time, args, cg.uint32)))
    return var
