import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor, fan, switch, text_sensor
from esphome.const import (
    CONF_ID,
    UNIT_MICROGRAMS_PER_CUBIC_METER,
    DEVICE_CLASS_PM25,
    STATE_CLASS_MEASUREMENT,
    ENTITY_CATEGORY_CONFIG,
)

CODEOWNERS = ["@lukednguyen"]
DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "fan", "switch", "text_sensor"]

ns = cg.esphome_ns.namespace("air_purifier_vital200s")
AirPurifier = ns.class_("AirPurifier", cg.PollingComponent, uart.UARTDevice)

# Entity classes
PurifierFan = ns.class_("PurifierFan", fan.Fan, cg.Parented.template(AirPurifier))
DisplaySwitch = ns.class_("DisplaySwitch", switch.Switch, cg.Parented.template(AirPurifier))
DisplayLockSwitch = ns.class_("DisplayLockSwitch", switch.Switch, cg.Parented.template(AirPurifier))
LightDetectionSwitch = ns.class_("LightDetectionSwitch", switch.Switch, cg.Parented.template(AirPurifier))

# Filter reset (commented out - filter life not readable from MCU)
# FilterResetButton = ns.class_("FilterResetButton", button.Button, cg.Parented.template(AirPurifier))

# Config keys
CONF_PM25 = "pm25"
CONF_FAN = "fan"
CONF_DISPLAY = "display"
CONF_DISPLAY_LOCK = "display_lock"
CONF_LIGHT_DETECTION = "light_detection"
CONF_AIR_QUALITY = "air_quality"

# Filter reset (commented out - filter life not readable from MCU)
# CONF_FILTER_RESET = "filter_reset"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(AirPurifier),
            # Sensors
            cv.Optional(CONF_PM25): sensor.sensor_schema(
                unit_of_measurement=UNIT_MICROGRAMS_PER_CUBIC_METER,
                device_class=DEVICE_CLASS_PM25,
                state_class=STATE_CLASS_MEASUREMENT,
                accuracy_decimals=0,
                icon="mdi:blur",
            ),
            cv.Optional(CONF_AIR_QUALITY): text_sensor.text_sensor_schema(
                icon="mdi:weather-windy",
            ),
            # Fan (restore_mode is included in fan_schema)
            cv.Optional(CONF_FAN): fan.fan_schema(
                PurifierFan,
                icon="mdi:air-filter",
            ),
            # Switches
            cv.Optional(CONF_DISPLAY): switch.switch_schema(
                DisplaySwitch,
                icon="mdi:lightbulb",
                entity_category=ENTITY_CATEGORY_CONFIG,
            ),
            cv.Optional(CONF_DISPLAY_LOCK): switch.switch_schema(
                DisplayLockSwitch,
                icon="mdi:lock",
                entity_category=ENTITY_CATEGORY_CONFIG,
            ),
            cv.Optional(CONF_LIGHT_DETECTION): switch.switch_schema(
                LightDetectionSwitch,
                icon="mdi:brightness-auto",
                entity_category=ENTITY_CATEGORY_CONFIG,
            ),
            # Filter reset (commented out - filter life not readable from MCU)
            # cv.Optional(CONF_FILTER_RESET): button.button_schema(
            #     FilterResetButton,
            #     icon="mdi:refresh",
            #     entity_category=ENTITY_CATEGORY_CONFIG,
            # ),
        }
    )
    .extend(cv.polling_component_schema("250ms"))
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    if c := config.get(CONF_PM25):
        s = await sensor.new_sensor(c)
        cg.add(var.set_pm25_sensor(s))

    if c := config.get(CONF_AIR_QUALITY):
        s = await text_sensor.new_text_sensor(c)
        cg.add(var.set_air_quality_sensor(s))

    if c := config.get(CONF_FAN):
        f = cg.new_Pvariable(c[CONF_ID])
        await fan.register_fan(f, c)
        cg.add(var.set_fan(f))

    if c := config.get(CONF_DISPLAY):
        s = await switch.new_switch(c)
        cg.add(var.set_display_switch(s))

    if c := config.get(CONF_DISPLAY_LOCK):
        s = await switch.new_switch(c)
        cg.add(var.set_display_lock_switch(s))

    if c := config.get(CONF_LIGHT_DETECTION):
        s = await switch.new_switch(c)
        cg.add(var.set_light_detection_switch(s))

    # Filter reset (commented out - filter life not readable from MCU)
    # if c := config.get(CONF_FILTER_RESET):
    #     b = await button.new_button(c)
    #     cg.add(var.set_filter_reset_button(b))