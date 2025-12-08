import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import (
    UNIT_CELSIUS,
    ICON_THERMOMETER,
)

from . import sensordlbus_ns, SensorDLBus, CONF_SENSORDLBUS_ID

DEPENDENCIES = ["sensordlbus"]

RoomTemperatureNumber = sensordlbus_ns.class_(
    "RoomTemperatureNumber", number.Number, cg.Component
)

CONF_ROOM_TEMPERATURE = "room_temperature"

CONFIG_SCHEMA = cv.Schema({
    cv.Required(CONF_SENSORDLBUS_ID): cv.use_id(SensorDLBus),
    cv.Optional(CONF_ROOM_TEMPERATURE): number.number_schema(
        RoomTemperatureNumber,
        unit_of_measurement=UNIT_CELSIUS,
        icon=ICON_THERMOMETER,
    ),
})

async def to_code(config):
    parent = await cg.get_variable(config[CONF_SENSORDLBUS_ID])

    if CONF_ROOM_TEMPERATURE in config:
        num = await number.new_number(
            config[CONF_ROOM_TEMPERATURE],
            min_value=13.0,
            max_value=25.0,
            step=0.1,
        )
        cg.add(parent.set_room_temperature_number(num))