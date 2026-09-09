import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor, fan, switch, number, binary_sensor
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
AUTO_LOAD = ["sensor", "fan", "switch", "number", "binary_sensor"]

ns = cg.esphome_ns.namespace("humidifier_oasismist1000s")
Humidifier = ns.class_("Humidifier", cg.PollingComponent, uart.UARTDevice)

HumidifierFan = ns.class_("HumidifierFan", fan.Fan, cg.Parented.template(Humidifier))
DisplaySwitch = ns.class_("DisplaySwitch", switch.Switch, cg.Parented.template(Humidifier))
TargetHumidityNumber = ns.class_("TargetHumidityNumber", number.Number, cg.Parented.template(Humidifier))

# Config keys
CONF_HUMIDITY = "humidity"
CONF_RESERVOIR = "reservoir"
CONF_WATER = "water"
CONF_MISTING = "misting"
CONF_FAN = "fan"
CONF_DISPLAY = "display"
CONF_TARGET_HUMIDITY = "target_humidity"
CONF_WIFI_STATUS_LED = "wifi_status_led"

# keep in sync with types.h HUMIDITY_MIN/MAX
HUMIDITY_MIN = 40
HUMIDITY_MAX = 80

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Humidifier),
            # Behaviour
            cv.Optional(CONF_WIFI_STATUS_LED, default=False): cv.boolean,
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
            # Fan (restore_mode comes from fan_schema)
            cv.Optional(CONF_FAN): fan.fan_schema(
                HumidifierFan,
                icon="mdi:air-humidifier",
            ),
            # Switches
            cv.Optional(CONF_DISPLAY): switch.switch_schema(
                DisplaySwitch,
                icon="mdi:monitor",
                entity_category=ENTITY_CATEGORY_CONFIG,
            ),
            # Numbers
            cv.Optional(CONF_TARGET_HUMIDITY): number.number_schema(
                TargetHumidityNumber,
                unit_of_measurement=UNIT_PERCENT,
                icon="mdi:water-percent",
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

    cg.add(var.set_wifi_status_led(config[CONF_WIFI_STATUS_LED]))

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

    if c := config.get(CONF_FAN):
        f = cg.new_Pvariable(c[CONF_ID])
        await fan.register_fan(f, c)
        cg.add(var.set_fan(f))

    if c := config.get(CONF_DISPLAY):
        s = await switch.new_switch(c)
        cg.add(var.set_display_switch(s))

    if c := config.get(CONF_TARGET_HUMIDITY):
        n = await number.new_number(c, min_value=HUMIDITY_MIN, max_value=HUMIDITY_MAX, step=1)
        cg.add(var.set_target_humidity_number(n))
