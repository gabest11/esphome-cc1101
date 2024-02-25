import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.components import spi
from esphome.const import CONF_ID
from esphome import automation
from esphome.automation import maybe_simple_id

DEPENDENCIES = ["spi"]

CONF_CC1101_GDO0 = "gdo0"
CONF_CC1101_GDO2 = "gdo2"
CONF_CC1101_BANDWIDTH = "bandwidth"
CONF_CC1101_FREQUENCY = "frequency"

cc1101_ns = cg.esphome_ns.namespace("cc1101")

CC1101_SWITCH = cc1101_ns.class_("CC1101Switch", switch.Switch, cg.Component, spi.SPIDevice)

BeginTxAction = cc1101_ns.class_("BeginTxAction", automation.Action)
EndTxAction = cc1101_ns.class_("EndTxAction", automation.Action)

CONFIG_SCHEMA = (
    switch.SWITCH_SCHEMA.extend({cv.GenerateID(): cv.declare_id(CC1101_SWITCH)})
    .extend(cv.COMPONENT_SCHEMA)
    .extend(spi.spi_device_schema(cs_pin_required=True))
    .extend(
        cv.Schema(
            {
                cv.Required(CONF_CC1101_GDO0): cv.uint8_t,
                cv.Optional(CONF_CC1101_GDO2): cv.uint8_t,
                cv.Required(CONF_CC1101_BANDWIDTH): cv.uint32_t,
                cv.Required(CONF_CC1101_FREQUENCY): cv.uint32_t,
            }
        )
    )
)

CC1101_ACTION_SCHEMA = maybe_simple_id(
    {
        cv.Required(CONF_ID): cv.use_id(CC1101_SWITCH),
    }
)

@automation.register_action(
    "cc1101.begin_tx", 
    BeginTxAction, 
    CC1101_ACTION_SCHEMA
)

@automation.register_action(
    "cc1101.end_tx", 
    EndTxAction, 
    CC1101_ACTION_SCHEMA
)

async def cc1101_action_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var

def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)
    yield switch.register_switch(var, config)
    yield spi.register_spi_device(var, config)

    cg.add(var.set_cc1101_gdo0(config[CONF_CC1101_GDO0]))
    if CONF_CC1101_GDO2 in config:
        cg.add(var.set_cc1101_gdo2(config[CONF_CC1101_GDO2]))
    cg.add(var.set_cc1101_bandwidth(config[CONF_CC1101_BANDWIDTH]))
    cg.add(var.set_cc1101_frequency(config[CONF_CC1101_FREQUENCY]))

