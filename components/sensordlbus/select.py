import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import select
from esphome.const import ICON_RADIATOR

from . import sensordlbus_ns, SensorDLBus, CONF_SENSORDLBUS_ID

DEPENDENCIES = ["sensordlbus"]

HeatingModeSelect = sensordlbus_ns.class_(
    "HeatingModeSelect", select.Select, cg.Component
)

CONF_HEATING_MODE = "heating_mode"

CONFIG_SCHEMA = cv.Schema({
    cv.Required(CONF_SENSORDLBUS_ID): cv.use_id(SensorDLBus),
    cv.Optional(CONF_HEATING_MODE): select.select_schema(
        HeatingModeSelect,
        icon=ICON_RADIATOR,
    ),
})

async def to_code(config):
    parent = await cg.get_variable(config[CONF_SENSORDLBUS_ID])

    if CONF_HEATING_MODE in config:
        sel = await select.new_select(
            config[CONF_HEATING_MODE],
            options=["AUTOMATIC", "NORMAL", "ENERGYSAVING", "STANDBY"],
        )
        cg.add(parent.set_heating_mode_select(sel))