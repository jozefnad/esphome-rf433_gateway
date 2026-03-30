"""RF433 Gateway — multi-protocol 433 MHz component for ESPHome.

Supported protocols:
  - A-OK (TX + RX) — motorised blinds/pergolas with optional light control
  - Nexus / Solight TE81 (RX) — temperature/humidity sensors

Dooya is NOT reimplemented here — ESPHome has built-in Dooya support via
remote_transmitter.transmit_dooya and the dooya: binary_sensor platform.
Use those directly in your YAML.

Architecture:
  1. Exposes a required top-level 'rf433_gw:' YAML key.
  2. Attaches RF433GWReceiver (a RemoteReceiverListener) to an existing
     remote_receiver component via 'receiver_id:'.
  3. Fires 'on_aok:' and 'on_nexus:' automations with optional filters.
  4. Registers 'rf433_gw.transmit_aok' action for transmitting A-OK commands.

YAML EXAMPLE:
  rf433_gw:
    receiver_id: rf_rx
    debounce: 500ms
    on_aok:
      - remote_id: 0xD3F5D3
        command: UP
        then:
          - cover.open: my_cover
    on_nexus:
      - id: 42
        channel: 1
        then:
          - logger.log: "Got temperature reading"

  button:
    - platform: template
      name: "Blind Down"
      on_press:
        - rf433_gw.transmit_aok:
            transmitter_id: rf_tx
            remote_id: 0xABCDEF
            address: 0x0001
            command: DOWN
            repeat:
              times: 3
              wait_time: 25ms
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import remote_base
from esphome.const import CONF_ID, CONF_REPEAT, CONF_TIMES, CONF_WAIT_TIME

CODEOWNERS = []
AUTO_LOAD = ["remote_base"]
DEPENDENCIES = ["remote_base"]
MULTI_CONF = True

# ─── C++ namespace & class references ────────────────────────────────────────
rf433_gw_ns = cg.esphome_ns.namespace("rf433_gw")

AOKData          = rf433_gw_ns.struct("AOKData")
NexusData        = rf433_gw_ns.struct("NexusData")
RF433GWReceiver  = rf433_gw_ns.class_(
    "RF433GWReceiver",
    cg.Component,
    remote_base.RemoteReceiverListener,
)

AOKTrigger       = rf433_gw_ns.class_(
    "AOKTrigger",
    automation.Trigger.template(AOKData),
)
NexusTrigger     = rf433_gw_ns.class_(
    "NexusTrigger",
    automation.Trigger.template(NexusData),
)
AOKTransmitAction = rf433_gw_ns.class_("AOKTransmitAction", automation.Action)

# ─── A-OK Commands enum ─────────────────────────────────────────────────────
AOKCommand = rf433_gw_ns.enum("AOKCommand")
AOK_COMMANDS = {
    "UP":        AOKCommand.AOK_CMD_UP,
    "STOP":      AOKCommand.AOK_CMD_STOP,
    "DOWN":      AOKCommand.AOK_CMD_DOWN,
    "PROGRAM":   AOKCommand.AOK_CMD_PROGRAM,
    "LIGHT_ON":  AOKCommand.AOK_CMD_LIGHT_ON,
    "LIGHT_OFF": AOKCommand.AOK_CMD_LIGHT_OFF,
}

# ─── Config constants ───────────────────────────────────────────────────────
CONF_RECEIVER_ID    = "receiver_id"
CONF_TRANSMITTER_ID = "transmitter_id"
CONF_REMOTE_ID      = "remote_id"
CONF_ADDRESS        = "address"
CONF_COMMAND        = "command"
CONF_DEBOUNCE       = "debounce"
CONF_CHANNEL        = "channel"
CONF_SENSOR_ID      = "sensor_id"

DEFAULT_REPEAT_WAIT_TIME = "25ms"
DEFAULT_DEBOUNCE = "500ms"

# ─── on_aok: trigger schema ─────────────────────────────────────────────────
AOK_TRIGGER_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(AOKTrigger),
        cv.Optional(CONF_REMOTE_ID): cv.hex_int_range(min=0, max=0xFFFFFF),
        cv.Optional(CONF_ADDRESS): cv.hex_int_range(min=0, max=0xFFFF),
        cv.Optional(CONF_COMMAND): cv.enum(AOK_COMMANDS, upper=True),
    }
)

# ─── on_nexus: trigger schema ───────────────────────────────────────────────
NEXUS_TRIGGER_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(NexusTrigger),
        cv.Optional(CONF_SENSOR_ID): cv.int_range(min=0, max=255),
        cv.Optional(CONF_CHANNEL): cv.int_range(min=1, max=4),
    }
)

# ─── Top-level rf433_gw: schema ─────────────────────────────────────────────
CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(RF433GWReceiver),
        cv.Required(CONF_RECEIVER_ID): cv.use_id(
            remote_base.RemoteReceiverBase
        ),
        cv.Optional(CONF_DEBOUNCE, default=DEFAULT_DEBOUNCE):
            cv.positive_time_period_milliseconds,
        cv.Optional("on_aok"): automation.validate_automation(
            AOK_TRIGGER_SCHEMA
        ),
        cv.Optional("on_nexus"): automation.validate_automation(
            NEXUS_TRIGGER_SCHEMA
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


# ─── Code generation ────────────────────────────────────────────────────────
async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # Attach to remote_receiver
    receiver = await cg.get_variable(config[CONF_RECEIVER_ID])
    cg.add(receiver.register_listener(var))

    # Debounce
    cg.add(var.set_debounce(config[CONF_DEBOUNCE].total_milliseconds))

    # Build on_aok: triggers
    for trig_conf in config.get("on_aok", []):
        trig = cg.new_Pvariable(trig_conf[CONF_ID])
        await automation.build_automation(trig, [(AOKData, "x")], trig_conf)
        cg.add(var.add_aok_trigger(trig))
        if CONF_REMOTE_ID in trig_conf:
            cg.add(trig.set_remote_id(trig_conf[CONF_REMOTE_ID]))
        if CONF_ADDRESS in trig_conf:
            cg.add(trig.set_address(trig_conf[CONF_ADDRESS]))
        if CONF_COMMAND in trig_conf:
            cg.add(trig.set_command(trig_conf[CONF_COMMAND]))

    # Build on_nexus: triggers
    for trig_conf in config.get("on_nexus", []):
        trig = cg.new_Pvariable(trig_conf[CONF_ID])
        await automation.build_automation(
            trig, [(NexusData, "x")], trig_conf
        )
        cg.add(var.add_nexus_trigger(trig))
        if CONF_SENSOR_ID in trig_conf:
            cg.add(trig.set_id(trig_conf[CONF_SENSOR_ID]))
        if CONF_CHANNEL in trig_conf:
            cg.add(trig.set_channel(trig_conf[CONF_CHANNEL]))


# ─── rf433_gw.transmit_aok action ───────────────────────────────────────────
AOK_TRANSMIT_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_TRANSMITTER_ID): cv.use_id(
            remote_base.RemoteTransmitterBase
        ),
        cv.Optional(CONF_RECEIVER_ID): cv.use_id(RF433GWReceiver),
        cv.Required(CONF_REMOTE_ID): cv.templatable(
            cv.hex_int_range(min=0, max=0xFFFFFF)
        ),
        cv.Required(CONF_ADDRESS): cv.templatable(
            cv.hex_int_range(min=0, max=0xFFFF)
        ),
        cv.Required(CONF_COMMAND): cv.templatable(
            cv.enum(AOK_COMMANDS, upper=True)
        ),
        cv.Optional(CONF_REPEAT): cv.Schema(
            {
                cv.Required(CONF_TIMES): cv.templatable(cv.positive_int),
                cv.Optional(
                    CONF_WAIT_TIME, default=DEFAULT_REPEAT_WAIT_TIME
                ): cv.templatable(cv.positive_time_period_microseconds),
            }
        ),
    }
)


@automation.register_action(
    "rf433_gw.transmit_aok",
    AOKTransmitAction,
    AOK_TRANSMIT_SCHEMA,
    synchronous=True,
)
async def aok_transmit_action_to_code(config, action_id, template_arg, args):
    transmitter = await cg.get_variable(config[CONF_TRANSMITTER_ID])

    # Receiver is optional — for self-reception protection
    if CONF_RECEIVER_ID in config:
        receiver = await cg.get_variable(config[CONF_RECEIVER_ID])
        var = cg.new_Pvariable(action_id, template_arg, transmitter, receiver)
    else:
        var = cg.new_Pvariable(
            action_id, template_arg, transmitter, cg.nullptr
        )

    templ = await cg.templatable(config[CONF_REMOTE_ID], args, cg.uint32)
    cg.add(var.set_remote_id(templ))
    templ = await cg.templatable(config[CONF_ADDRESS], args, cg.uint16)
    cg.add(var.set_address(templ))
    templ = await cg.templatable(config[CONF_COMMAND], args, cg.uint8)
    cg.add(var.set_command(templ))

    if CONF_REPEAT in config:
        repeat = config[CONF_REPEAT]
        templ = await cg.templatable(repeat[CONF_TIMES], args, cg.uint32)
        cg.add(var.set_send_times(templ))
        templ = await cg.templatable(repeat[CONF_WAIT_TIME], args, cg.uint32)
        cg.add(var.set_send_wait(templ))

    return var
