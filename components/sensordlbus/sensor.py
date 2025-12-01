import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, text_sensor
from esphome.const import (
    CONF_ID,
    UNIT_CELSIUS,
    ICON_THERMOMETER,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
)



# Namespace für die External Component
sensordlbus_ns = cg.esphome_ns.namespace("sensordlbus")
SensorDLBus = sensordlbus_ns.class_("SensorDLBus", cg.PollingComponent)

# **Hier wird der neue Key für device_type gesetzt**
CONF_DEVICE_TYPE = "devicetype"

# **Erweitertes Schema mit Debug-Ausgabe**
CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(SensorDLBus),
    cv.Optional(CONF_DEVICE_TYPE): sensor.sensor_schema(),
    cv.Optional("temperature_1"): sensor.sensor_schema(unit_of_measurement=UNIT_CELSIUS, icon=ICON_THERMOMETER, accuracy_decimals=1, device_class=DEVICE_CLASS_TEMPERATURE, state_class=STATE_CLASS_MEASUREMENT),
    cv.Optional("temperature_2"): sensor.sensor_schema(unit_of_measurement=UNIT_CELSIUS, icon=ICON_THERMOMETER, accuracy_decimals=1, device_class=DEVICE_CLASS_TEMPERATURE, state_class=STATE_CLASS_MEASUREMENT),
    cv.Optional("temperature_3"): sensor.sensor_schema(unit_of_measurement=UNIT_CELSIUS, icon=ICON_THERMOMETER, accuracy_decimals=1, device_class=DEVICE_CLASS_TEMPERATURE, state_class=STATE_CLASS_MEASUREMENT),
    cv.Optional("temperature_4"): sensor.sensor_schema(unit_of_measurement=UNIT_CELSIUS, icon=ICON_THERMOMETER, accuracy_decimals=1, device_class=DEVICE_CLASS_TEMPERATURE, state_class=STATE_CLASS_MEASUREMENT),
    cv.Optional("temperature_5"): sensor.sensor_schema(unit_of_measurement=UNIT_CELSIUS, icon=ICON_THERMOMETER, accuracy_decimals=1, device_class=DEVICE_CLASS_TEMPERATURE, state_class=STATE_CLASS_MEASUREMENT),
    cv.Optional("temperature_6"): sensor.sensor_schema(unit_of_measurement=UNIT_CELSIUS, icon=ICON_THERMOMETER, accuracy_decimals=1, device_class=DEVICE_CLASS_TEMPERATURE, state_class=STATE_CLASS_MEASUREMENT),
    cv.Optional("output_a1"): sensor.sensor_schema(),
    cv.Optional("output_a2"): sensor.sensor_schema(),
    cv.Optional("output_a3"): sensor.sensor_schema(),
    cv.Optional("output_a4"): sensor.sensor_schema(),
    cv.Optional("output_a5"): sensor.sensor_schema(),
    cv.Optional("output_a6"): sensor.sensor_schema(),
    cv.Optional("output_a7"): sensor.sensor_schema(),
}).extend(cv.polling_component_schema("30s"))



async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # Debug: Gibt den Inhalt von config aus
    print("CONFIG:", config)

    if CONF_DEVICE_TYPE in config:
        sens = await sensor.new_sensor(config[CONF_DEVICE_TYPE])
        cg.add(var.set_device_type_sensor(sens))

    for i in range(1, 7):
        key = f"temperature_{i}"
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(var, f"set_temp_sensor{i}")(sens))

    for i in range(1, 8):
        key = f"output_a{i}"
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(var, f"set_output_a{i}_sensor")(sens))
