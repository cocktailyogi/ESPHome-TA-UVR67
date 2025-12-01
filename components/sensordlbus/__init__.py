import esphome.codegen as cg

# WICHTIG: text_sensor als Abhängigkeit!
DEPENDENCIES = ["text_sensor"]

sensordlbus_ns = cg.esphome_ns.namespace("sensordlbus")