import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor, switch, select, number, binary_sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_CONNECTIVITY,
    DEVICE_CLASS_HUMIDITY,
    DEVICE_CLASS_MOISTURE,
    DEVICE_CLASS_RUNNING,
    STATE_CLASS_MEASUREMENT,
    UNIT_PERCENT,
    ENTITY_CATEGORY_NONE,
    ENTITY_CATEGORY_CONFIG,
    ENTITY_CATEGORY_DIAGNOSTIC,
)

CODEOWNERS = ["@lukednguyen"]
DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "switch", "select", "number", "binary_sensor"]

ns = cg.esphome_ns.namespace("humidifier_oasismist1000s")
Humidifier = ns.class_("Humidifier", cg.PollingComponent, uart.UARTDevice)

PowerSwitch = ns.class_("PowerSwitch", switch.Switch, cg.Parented.template(Humidifier))
DisplaySwitch = ns.class_("DisplaySwitch", switch.Switch, cg.Parented.template(Humidifier))
ModeSelect = ns.class_("ModeSelect", select.Select, cg.Parented.template(Humidifier))
TargetHumidityNumber = ns.class_("TargetHumidityNumber", number.Number, cg.Parented.template(Humidifier))
MistLevelNumber = ns.class_("MistLevelNumber", number.Number, cg.Parented.template(Humidifier))

# Config keys
CONF_HUMIDITY = "humidity"
CONF_RESERVOIR = "reservoir"
CONF_WATER = "water"
CONF_MISTING = "misting"
CONF_POWER = "power"
CONF_DISPLAY = "display"
CONF_MODE = "mode"
CONF_TARGET_HUMIDITY = "target_humidity"
CONF_MIST_LEVEL = "mist_level"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Humidifier),
            # Sensor
            cv.Optional(CONF_HUMIDITY): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_HUMIDITY,
                state_class=STATE_CLASS_MEASUREMENT,
                icon="mdi:water-percent",
            ),
            # Binary Sensors
            cv.Optional(CONF_RESERVOIR): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_CONNECTIVITY,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon="mdi:cup-water",
            ),
            cv.Optional(CONF_WATER): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_MOISTURE,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon="mdi:water",
            ),
            cv.Optional(CONF_MISTING): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_RUNNING,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon="mdi:waves",
            ),
            # Switches
            cv.Optional(CONF_POWER): switch.switch_schema(
                PowerSwitch,
                icon="mdi:power",
                entity_category=ENTITY_CATEGORY_NONE,
            ),
            cv.Optional(CONF_DISPLAY): switch.switch_schema(
                DisplaySwitch,
                icon="mdi:monitor",
                entity_category=ENTITY_CATEGORY_CONFIG,
            ),
            # Select
            cv.Optional(CONF_MODE): select.select_schema(
                ModeSelect,
                icon="mdi:format-list-bulleted",
                entity_category=ENTITY_CATEGORY_NONE,
            ),
            # Numbers
            cv.Optional(CONF_TARGET_HUMIDITY): number.number_schema(
                TargetHumidityNumber,
                unit_of_measurement=UNIT_PERCENT,
                icon="mdi:water-percent",
                entity_category=ENTITY_CATEGORY_NONE,
            ),
            cv.Optional(CONF_MIST_LEVEL): number.number_schema(
                MistLevelNumber,
                icon="mdi:weather-fog",
                entity_category=ENTITY_CATEGORY_NONE,
            ),
        }
    )
    .extend(cv.polling_component_schema("250ms"))
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    if c := config.get(CONF_HUMIDITY):
        s = await sensor.new_sensor(c)
        cg.add(var.set_humidity_sensor(s))

    if c := config.get(CONF_RESERVOIR):
        s = await binary_sensor.new_binary_sensor(c)
        cg.add(var.set_reservoir_sensor(s))

    if c := config.get(CONF_WATER):
        s = await binary_sensor.new_binary_sensor(c)
        cg.add(var.set_water_sensor(s))

    if c := config.get(CONF_MISTING):
        s = await binary_sensor.new_binary_sensor(c)
        cg.add(var.set_misting_sensor(s))

    if c := config.get(CONF_POWER):
        s = await switch.new_switch(c)
        cg.add(var.set_power_switch(s))

    if c := config.get(CONF_DISPLAY):
        s = await switch.new_switch(c)
        cg.add(var.set_display_switch(s))

    if c := config.get(CONF_MODE):
        s = await select.new_select(c, options=["Auto", "Manual", "Sleep"])
        cg.add(var.set_mode_select(s))

    if c := config.get(CONF_TARGET_HUMIDITY):
        n = await number.new_number(c, min_value=40, max_value=80, step=1)
        cg.add(var.set_target_humidity_number(n))

    if c := config.get(CONF_MIST_LEVEL):
        n = await number.new_number(c, min_value=1, max_value=9, step=1)
        cg.add(var.set_mist_level_number(n))