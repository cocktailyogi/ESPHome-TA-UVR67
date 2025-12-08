import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome import pins

CODEOWNERS = ["@cocktailyogi"]

sensordlbus_ns = cg.esphome_ns.namespace("sensordlbus")
SensorDLBus = sensordlbus_ns.class_("SensorDLBus", cg.PollingComponent)

CONF_SENSORDLBUS_ID = "sensordlbus_id"
CONF_INPUT_PIN = "dlbus_rx_pin"
CONF_OUTPUT_PIN = "dlbus_tx_pin"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(SensorDLBus),
    cv.Required(CONF_INPUT_PIN): pins.internal_gpio_input_pin_number,
    cv.Required(CONF_OUTPUT_PIN): pins.internal_gpio_output_pin_number,
}).extend(cv.polling_component_schema("30s"))

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_input_pin(config[CONF_INPUT_PIN]))
    cg.add(var.set_output_pin(config[CONF_OUTPUT_PIN]))