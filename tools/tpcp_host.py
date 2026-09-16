#!/usr/bin/env python3
"""Host-side master for the TunePulse CAN Protocol (TPCP) v1.

The drive is a protocol *slave*: it answers requests, streams telemetry and
reports faults, but it never initiates a transaction. This script is the other
half - the master a bench needs in order to talk to it at all.

The wire format implemented here mirrors src/api/tp_wire.h and tp_chunks.h
exactly; src/api/CAN_PROTOCOL.md is the specification.

Setup
-----
    pip install python-can

    # Simplest: let it find the drive's own USB link by itself
    ./tpcp_host.py info 1
    ./tpcp_host.py move 1 0.25

    # Pin the port instead of auto-detecting (or export TPCP_CHANNEL)
    ./tpcp_host.py --channel /dev/cu.usbmodem1234 info 1

    # CANable / CANtact in slcan mode (the usual cheap adapter on macOS)
    ./tpcp_host.py --interface slcan --channel /dev/tty.usbmodem1234 scan

    # PEAK PCAN-USB
    ./tpcp_host.py --interface pcan --channel PCAN_USBBUS1 info 1

    # Linux SocketCAN
    ./tpcp_host.py --interface socketcan --channel can0 monitor

    # No CAN adapter at all: speak the protocol over the board's USB console
    ./tpcp_host.py --interface serial --channel /dev/tty.usbmodemXXXX --log info 1
    ./tpcp_host.py --interface hid info 1

Registers are read with `get` and written with `set field=value ...`, which
performs a read-modify-write so untouched fields keep their value:

    ./tpcp_host.py get 1 motor_props_1
    ./tpcp_host.py set 1 motor_props_1 max_current=1.5
    ./tpcp_host.py get 1 axis:led
    ./tpcp_host.py set 1 axis:led mode=0 r=0 g=255 b=0
    ./tpcp_host.py get 1 axis:io_map function=3          # keyed record

Bit rate must match the drive. A drive built with the cln17_v3_bench
environment comes up as node 1 on classic CAN at 500 kbit/s, which is the
default here. The shipping firmware comes up unaddressed on CAN-FD at
1 Mbit/s / 5 Mbit/s: use --fd --bitrate 1000000 --data-bitrate 5000000 and run
`scan` first to give it an address.

Copyright 2026, creapunk
Licensed under the Apache License, Version 2.0
"""

import argparse
import math
import os
import re
import struct
import sys
import time

try:
    import can
except ImportError:
    sys.exit("python-can is required:  pip install python-can")

# ============================================================================
# Identifier map (src/api/wire/tp_ids.h)
# ============================================================================

ID_EMCY = 0x100       # + node   node -> bus     emergency / fault
ID_COMMAND = 0x300    # + node   master -> node  registers, setpoints, flashing
ID_RESPONSE = 0x400   # + node   node -> master  responses, discovery replies
ID_TELEMETRY = 0x500  # + node   node -> bus     PulseBeat and telemetry

NODE_BROADCAST = 0x00
NODE_UNASSIGNED = 0xFF
UID_BITS = 48
UID_BYTES = 6

# ============================================================================
# CMD byte (src/api/wire/tp_cmds.h)
# ============================================================================

CMD_WRITE = 0x80

# Read-form register addresses by name. Sub-space registers carry the record in
# byte 1 and are addressed as "name:record" on the command line.
CMD = {
    "direct": 0x00, "axis": 0x01, "telemetry": 0x02,
    "bus": 0x10, "discovery_assign": 0x11, "discovery_search": 0x12,
    "system_control": 0x20, "motor_props_1": 0x21, "motor_props_2": 0x22,
    "power_limits": 0x23, "motor_thermal": 0x24, "driver_thermal": 0x25,
    "supply_protection": 0x26, "current_protection": 0x27,
    "motion_limits": 0x2A, "error_limits": 0x2B, "watchdog": 0x2D, "cal_table": 0x2F,
    "batch_read": 0x40, "range_read": 0x41, "currents": 0x42, "diag": 0x43,
    "device_info": 0x50, "system_status": 0x51, "motor_status": 0x52,
    "motion_status_1": 0x53, "motion_status_2": 0x54, "motion_status_3": 0x55,
    "homing_travel": 0x56, "encoder_position": 0x57, "driver_status": 0x58,
    "ack": 0x5E, "nack": 0x5F, "pulsebeat": 0x70,
    "motion_task_1": 0xC0, "motion_task_2": 0xC1, "motion_task_3": 0xC2, "motion_task_4": 0xC3,
    "save_config": 0xCF, "midi": 0xD0, "burst": 0xE0,
}
CMD_NAME = {v: k for k, v in CMD.items()}

SUB_AXIS = {"axis_control": 0, "led": 1, "gearbox": 2, "io_map": 3, "encoder_1": 4, "encoder_2": 5,
            "homing_quick": 6, "homing_fine": 7, "ref_cw": 8, "ref_ccw": 9, "pos_zero": 10,
            "zero_offset_cw": 11, "zero_offset_ccw": 12, "field_cw": 13, "field_ccw": 14,
            "internal_offset": 15}
SUB_CURRENTS = {"dq": 0, "ab": 1}
SUB_DIAG = {"comm": 0, "calibration": 1, "queue": 2, "persist": 3, "encoder": 4, "calib_result": 5, "foc": 6,
            "encoder_link": 7, "ident_fail": 8}

SAVE_KEY = 0x000010AD
DEFAULTS_KEY = 0x0000DEF0
ERASE_KEY = 0x0000E7A5
CAL_TABLE_SIZE_KEY = 0xCA1B513E

SYS_ENABLE, SYS_DISABLE = 0xE1, 0x0F
SYS_FAULT_RESET, SYS_REBOOT, SYS_REBOOT_DFU = 0xC1, 0xB0, 0xDF
# Torque-ripple feed-forward map: start averaging / fold in and enable / zero and disable (RAM only)
SYS_RIPPLE_LEARN, SYS_RIPPLE_COMMIT, SYS_RIPPLE_CLEAR = 0xA1, 0xA2, 0xA3
CAL_BITS = {"encoder": 0x01, "offset": 0x02, "phase": 0x04, "resistance": 0x08,
            "inductance": 0x10, "flux": 0x20, "mechanical": 0x40}
CAL_BITS["electrical"] = CAL_BITS["phase"] | CAL_BITS["resistance"] | CAL_BITS["inductance"]
CAL_BITS["angle"] = CAL_BITS["encoder"]
CAL_BITS["full"] = CAL_BITS["electrical"] | CAL_BITS["encoder"]

ERRORS = {
    0x00: "none", 0x01: "unknown cmd", 0x02: "bad length", 0x03: "bad parameter",
    0x04: "read only", 0x05: "write only", 0x06: "unknown sub", 0x07: "out of range",
    0x08: "wrong state", 0x09: "busy", 0x0A: "queue full", 0x0B: "not implemented",
    0x0C: "access denied", 0x0D: "fault active", 0x0E: "nvm error", 0x0F: "not calibrated",
    0x10: "segment mismatch",
}
STATES = ["boot", "calibrating", "idle", "ready", "running", "fault", "unaddressed", "rebooting"]
MODES = ["disabled", "torque", "velocity", "position", "motion", "tone"]
FAULT_BITS = ["overvoltage", "overcurrent", "overheat", "external", "undervoltage", "overpower", "phase", "encoder"]
WARN_BITS = ["not-calibrated", "flash-busy", "watchdog", "queue-low", "queue-drop", "comm", "temperature", "supply"]
FAULT_GROUPS = {0: "none", 1: "overvoltage", 2: "overcurrent", 3: "overheat", 4: "external",
                5: "undervoltage", 6: "overpower", 7: "phase", 15: "multiple"}
MT_MODES = {"hold": 0, "freerun": 1, "freerun_lerp": 2, "freewheel": 3, "brake": 4, "scurve": 5,
            "pvt": 6, "anchor": 7, "limit": 8, "homing": 9, "step_dir": 10, "reset": 15}
MT_MODE_NAME = {v: k for k, v in MT_MODES.items()}
BAUD_CODES = {0: "keep", 1: 125, 2: 250, 3: 500, 4: 1000, 5: 2000, 6: 4000, 7: 5000, 8: 8000}

# ============================================================================
# Codecs (src/api/wire/tp_codecs.h)
# ============================================================================

BF16_NAN = 0x7FC0
F24_NAN = 0x7FC000


def bf16_to_float(raw):
    """bfloat16 -> float: the top 16 bits of an IEEE-754 float."""
    return struct.unpack("<f", struct.pack("<I", (raw & 0xFFFF) << 16))[0]


def float_to_bf16(value):
    """float -> bfloat16 (truncating)."""
    return (struct.unpack("<I", struct.pack("<f", float(value)))[0] >> 16) & 0xFFFF


def f24_to_float(raw):
    """float24 -> float: the top 24 bits of an IEEE-754 float (pure shift, no sign extension)."""
    return struct.unpack("<f", struct.pack("<I", (raw & 0xFFFFFF) << 8))[0]


def float_to_f24(value):
    """float -> float24 (truncating)."""
    return (struct.unpack("<I", struct.pack("<f", float(value)))[0] >> 8) & 0xFFFFFF


def signext(value, bits):
    """Sign-extend a `bits`-wide two's-complement value."""
    value &= (1 << bits) - 1
    return value - (1 << bits) if value & (1 << (bits - 1)) else value


def is_nan(value):
    """True for a float that carries no value."""
    return isinstance(value, float) and math.isnan(value)


# ============================================================================
# Register field model
# ============================================================================
#
# Each register is a list of fields; each field knows where it lives and how it
# is encoded, so one generic encoder and decoder serve every layout and the
# `set` command can do a read-modify-write of arbitrary fields by name.
#
# Kinds: u8 u16 u32 i8 i16 i32 u24 bf16 f24 q16 (i32 rev) q24 (48-bit rev, 24 frac)
#        q32 (48-bit rev, 32 frac) q32_56 (56-bit rev, 32 frac) uid (6-byte hex)
#        bits (byte-level bit field: byte, shift, width, optional signed)
#        wbits (16-bit-word bit field: byte, shift, width)

WIDTHS = {"u8": 1, "i8": 1, "u16": 2, "i16": 2, "bf16": 2, "u24": 3, "f24": 3, "u32": 4, "i32": 4,
          "q16": 4, "q24": 6, "q32": 6, "uid": 6, "q32_56": 7}


class Field:
    """One named field of a register frame."""

    def __init__(self, name, kind, byte, shift=0, width=0, signed=False, ro=False, doc=""):
        self.name, self.kind, self.byte = name, kind, byte
        self.shift, self.width, self.signed, self.ro, self.doc = shift, width, signed, ro, doc

    def decode(self, d):
        """Read this field out of an 8-byte frame."""
        k, b = self.kind, self.byte
        if k == "bits":
            v = (d[b] >> self.shift) & ((1 << self.width) - 1)
            return signext(v, self.width) if self.signed else v
        if k == "wbits":
            w = d[b] | (d[b + 1] << 8)
            v = (w >> self.shift) & ((1 << self.width) - 1)
            return signext(v, self.width) if self.signed else v
        if k == "u8":
            return d[b]
        if k == "i8":
            return signext(d[b], 8)
        if k == "u16":
            return d[b] | (d[b + 1] << 8)
        if k == "i16":
            return signext(d[b] | (d[b + 1] << 8), 16)
        if k == "u24":
            return d[b] | (d[b + 1] << 8) | (d[b + 2] << 16)
        if k == "u32":
            return struct.unpack_from("<I", d, b)[0]
        if k == "i32":
            return struct.unpack_from("<i", d, b)[0]
        if k == "bf16":
            return bf16_to_float(d[b] | (d[b + 1] << 8))
        if k == "f24":
            return f24_to_float(d[b] | (d[b + 1] << 8) | (d[b + 2] << 16))
        if k == "q16":
            return struct.unpack_from("<i", d, b)[0] / 65536.0
        if k == "q24":
            return signext(int.from_bytes(d[b:b + 6], "little"), 48) / float(1 << 24)
        if k == "q32":
            return signext(int.from_bytes(d[b:b + 6], "little"), 48) / float(1 << 32)
        if k == "q32_56":
            return signext(int.from_bytes(d[b:b + 7], "little"), 56) / float(1 << 32)
        if k == "uid":
            return int.from_bytes(d[b:b + 6], "little")
        raise ValueError(k)

    def encode(self, d, value):
        """Write this field into a mutable 8-byte frame."""
        k, b = self.kind, self.byte
        if k in ("bits", "wbits"):
            mask = (1 << self.width) - 1
            v = int(value) & mask
            if k == "bits":
                d[b] = (d[b] & ~(mask << self.shift) & 0xFF) | (v << self.shift)
            else:
                w = (d[b] | (d[b + 1] << 8)) & ~(mask << self.shift) & 0xFFFF | (v << self.shift)
                d[b], d[b + 1] = w & 0xFF, w >> 8
            return
        if k in ("u8", "i8"):
            d[b] = int(value) & 0xFF
        elif k in ("u16", "i16"):
            d[b:b + 2] = struct.pack("<H", int(value) & 0xFFFF)
        elif k == "u24":
            d[b:b + 3] = (int(value) & 0xFFFFFF).to_bytes(3, "little")
        elif k in ("u32", "i32"):
            d[b:b + 4] = struct.pack("<I", int(value) & 0xFFFFFFFF)
        elif k == "bf16":
            d[b:b + 2] = struct.pack("<H", BF16_NAN if is_nan(value) else float_to_bf16(value))
        elif k == "f24":
            raw = F24_NAN if is_nan(value) else float_to_f24(value)
            d[b:b + 3] = raw.to_bytes(3, "little")
        elif k == "q16":
            d[b:b + 4] = struct.pack("<i", int(round(float(value) * 65536.0)))
        elif k == "q24":
            d[b:b + 6] = (int(round(float(value) * (1 << 24))) & 0xFFFFFFFFFFFF).to_bytes(6, "little")
        elif k == "q32":
            d[b:b + 6] = (int(round(float(value) * (1 << 32))) & 0xFFFFFFFFFFFF).to_bytes(6, "little")
        elif k == "q32_56":
            d[b:b + 7] = (int(round(float(value) * (1 << 32))) & ((1 << 56) - 1)).to_bytes(7, "little")
        elif k == "uid":
            d[b:b + 6] = (int(value) & 0xFFFFFFFFFFFF).to_bytes(6, "little")
        else:
            raise ValueError(k)

    def parse(self, text):
        """Turn a command-line string into a value for encode()."""
        if self.kind in ("bf16", "f24", "q16", "q24", "q32", "q32_56"):
            return float("nan") if text.lower() == "nan" else float(text)
        return int(text, 0)


def F(name, kind, byte, **kw):
    """Shorthand constructor."""
    return Field(name, kind, byte, **kw)


class Reg:
    """A register layout: CMD, optional sub record, key field and fields."""

    def __init__(self, cmd, fields, sub=None, keyed=None, writable=True):
        self.cmd, self.fields, self.sub, self.keyed, self.writable = cmd, fields, sub, keyed, writable

    def request(self, key=None):
        """Build the read request frame for this register."""
        d = bytearray(8)
        d[0] = self.cmd
        if self.sub is not None:
            d[1] = self.sub
        if self.keyed is not None and key is not None:
            self.by_name(self.keyed).encode(d, key)
        return bytes(d)

    def by_name(self, name):
        for f in self.fields:
            if f.name == name:
                return f
        raise KeyError(name)

    def decode(self, d):
        return {f.name: f.decode(d) for f in self.fields}


def _bits_field(name, byte, shift, width, **kw):
    return Field(name, "bits", byte, shift=shift, width=width, **kw)


def _wbits_field(name, byte, shift, width, **kw):
    return Field(name, "wbits", byte, shift=shift, width=width, **kw)


REGISTERS = {
    # ---- 0x00 - 0x3F read/write ------------------------------------------
    "direct": Reg(0x00, [F("address", "u24", 1), F("value", "u32", 4)]),
    "axis:axis_control": Reg(0x01, [F("axis_id", "u8", 2), _bits_field("inverted", 3, 0, 1),
                                    F("offset", "q16", 4)], sub=0),
    "axis:led": Reg(0x01, [F("mode", "u8", 2), _bits_field("on_time", 3, 0, 4), _bits_field("off_time", 3, 4, 4),
                           F("led_id", "u8", 4), F("r", "u8", 5), F("g", "u8", 6), F("b", "u8", 7)], sub=1),
    "axis:gearbox": Reg(0x01, [F("backlash", "bf16", 2), F("ratio_mul", "i16", 4), F("ratio_div", "u16", 6)], sub=2),
    "axis:io_map": Reg(0x01, [F("function", "u8", 2), F("on_pin", "u8", 3), F("off_pin", "u8", 4),
                              _bits_field("on_inverted", 5, 0, 1), _bits_field("off_inverted", 5, 1, 1),
                              _bits_field("state", 5, 2, 1)], sub=3, keyed="function"),
    "axis:encoder_1": Reg(0x01, [_bits_field("type", 2, 0, 4), _bits_field("interface", 2, 4, 4),
                                 _bits_field("ccw", 3, 0, 1), _bits_field("index", 3, 1, 1), F("ppr", "u32", 4)], sub=4),
    "axis:encoder_2": Reg(0x01, [_bits_field("type", 2, 0, 4), _bits_field("interface", 2, 4, 4),
                                 _bits_field("ccw", 3, 0, 1), _bits_field("index", 3, 1, 1), F("ppr", "u32", 4)], sub=5),
    "axis:homing_quick": Reg(0x01, [_wbits_field("mode", 2, 0, 2), _wbits_field("trigger", 2, 2, 1),
                                    _wbits_field("ccw", 2, 3, 1), _wbits_field("bidir", 2, 4, 1),
                                    _wbits_field("validation", 2, 5, 1), _wbits_field("fine_trigger", 2, 6, 1),
                                    F("velocity", "bf16", 4), F("torque", "bf16", 6)], sub=6),
    "axis:homing_fine": Reg(0x01, [F("retract", "bf16", 2), F("velocity", "bf16", 4), F("torque", "bf16", 6)], sub=7),
    "axis:ref_cw": Reg(0x01, [F("offset", "bf16", 2), F("ref_angle", "u32", 4)], sub=8),
    "axis:ref_ccw": Reg(0x01, [F("offset", "bf16", 2), F("ref_angle", "u32", 4)], sub=9),
    "axis:pos_zero": Reg(0x01, [F("ref_angle", "u32", 4)], sub=10),
    "axis:zero_offset_cw": Reg(0x01, [F("offset", "q32", 2)], sub=11),
    "axis:zero_offset_ccw": Reg(0x01, [F("offset", "q32", 2)], sub=12),
    "axis:field_cw": Reg(0x01, [F("limit", "q32", 2)], sub=13),
    "axis:field_ccw": Reg(0x01, [F("limit", "q32", 2)], sub=14),
    "axis:internal_offset": Reg(0x01, [F("offset", "q32", 2)], sub=15),
    "bus": Reg(0x10, [_bits_field("type", 1, 0, 4), _bits_field("comm", 1, 4, 4), F("baud1", "u8", 2),
                      F("baud2", "u8", 3), F("sync", "u32", 4)]),
    "discovery_assign": Reg(0x11, [F("addr", "u8", 1), F("uid", "uid", 2)]),
    "discovery_search": Reg(0x12, [F("depth", "u8", 1), F("prefix", "uid", 2)]),
    "system_control": Reg(0x20, [F("enable", "u8", 1), F("burst_step", "u8", 2), F("calibration", "u8", 3),
                                 F("action", "u8", 4)]),
    "motor_props_1": Reg(0x21, [_bits_field("mode", 1, 0, 4), _bits_field("type", 1, 4, 4),
                                _wbits_field("phase", 2, 0, 2), _wbits_field("direction", 2, 2, 2, signed=True),
                                _wbits_field("pole_pairs", 2, 4, 12), F("max_voltage", "bf16", 4),
                                F("max_current", "bf16", 6)]),
    "motor_props_2": Reg(0x22, [F("temp_limit", "u8", 1), F("resistance", "bf16", 2), F("inductance", "bf16", 4),
                                F("kt", "bf16", 6)]),
    "power_limits": Reg(0x23, [F("overdrive_s", "u8", 1), F("baseline", "bf16", 2), F("peak", "bf16", 4),
                               F("capacity", "bf16", 6)]),
    "motor_thermal": Reg(0x24, [F("temp_now", "u8", 1, ro=True), F("temp_base", "u8", 2), F("temp_limit", "u8", 3),
                                F("derating", "bf16", 4), F("rth", "bf16", 6)]),
    "driver_thermal": Reg(0x25, [F("temp_now", "u8", 1, ro=True), F("ot_warn", "u8", 2), F("ot_fault", "u8", 3),
                                 F("hysteresis", "u8", 4)]),
    "supply_protection": Reg(0x26, [F("hysteresis", "bf16", 1), F("recovery", "u8", 3), F("ov", "bf16", 4),
                                    F("uv", "bf16", 6)]),
    "current_protection": Reg(0x27, [F("disbalance", "u8", 1), F("alpha", "u8", 2), F("recovery", "u8", 3),
                                     F("peak", "bf16", 4), F("oc", "bf16", 6)]),
    "motion_limits": Reg(0x2A, [F("velocity", "f24", 1), F("accel", "bf16", 4), F("jerk", "bf16", 6)]),
    "error_limits": Reg(0x2B, [F("velocity", "f24", 1), F("accel", "bf16", 4), F("jerk", "bf16", 6)]),
    "watchdog": Reg(0x2D, [_bits_field("retries", 1, 0, 7), _bits_field("force_retry", 1, 7, 1),
                           F("period", "bf16", 2), F("delay", "bf16", 4), F("timeout", "bf16", 6)]),
    "cal_table": Reg(0x2F, [_bits_field("mode", 1, 0, 2), _bits_field("absolute", 1, 2, 1),
                            _bits_field("resolution", 1, 3, 2), _bits_field("table", 1, 5, 3),
                            F("sample", "u16", 2), F("data", "u32", 4)], keyed="sample", writable=False),
    # ---- 0x40 - 0x7F read only ---------------------------------------------
    "currents:dq": Reg(0x42, [F("supply", "bf16", 2), F("iq", "bf16", 4), F("id", "bf16", 6)], sub=0, writable=False),
    "currents:ab": Reg(0x42, [F("supply", "bf16", 2), F("ia", "bf16", 4), F("ib", "bf16", 6)], sub=1, writable=False),
    "diag:comm": Reg(0x43, [F("bus_state", "u8", 2), F("tec", "u8", 3), F("rec", "u8", 4), F("dropped", "u8", 5),
                            F("rejected", "u16", 6)], sub=0, writable=False),
    "diag:calibration": Reg(0x43, [F("running", "u8", 2), F("stage", "u8", 3), F("progress", "u8", 4),
                                   F("flags", "u8", 5), F("cal_id", "u8", 6), F("pole_pairs", "u8", 7)],
                            sub=1, writable=False),
    "diag:queue": Reg(0x43, [F("state", "u8", 2), F("queued", "u8", 3), F("free", "u8", 4),
                             F("time_scale", "i16", 5), F("dropped", "u8", 7)], sub=2, writable=False),
    "diag:persist": Reg(0x43, [F("state", "u8", 2), F("config_ver", "u8", 3), F("cal_id", "u8", 4),
                               F("config_free", "u16", 5)], sub=3, writable=False),
    "diag:encoder": Reg(0x43, [F("angle", "u16", 2), F("turns", "i32", 4)], sub=4, writable=False),
    "diag:calib_result": Reg(0x43, [F("resistance", "bf16", 2), F("ld", "bf16", 4), F("lq", "bf16", 6)],
                             sub=5, writable=False),
    "diag:foc": Reg(0x43, [F("angle_el", "u16", 2), F("ud", "bf16", 4), F("uq", "bf16", 6)], sub=6, writable=False),
    "diag:encoder_link": Reg(0x43, [_bits_field("check_proven", 2, 0, 1), _bits_field("link_down", 2, 1, 1),
                                    _bits_field("overspeed", 2, 2, 1), _bits_field("weak_field", 2, 3, 1),
                                    _bits_field("undervoltage", 2, 4, 1), F("relinks", "u8", 3),
                                    F("frames_bad", "u16", 4), F("frames_ok", "u16", 6)],
                             sub=7, writable=False),
    "trace": Reg(0x44, [F("part", "u8", 1), F("index", "u16", 2), F("a", "u16", 4), F("b", "u16", 6)], writable=False),
    "diag:ident_fail": Reg(0x43, [F("reason", "u8", 2), F("stage", "u8", 3), F("fault", "u8", 4),
                                  F("volts_x10", "u8", 5), F("ia_x20", "i8", 6), F("ib_x20", "i8", 7)],
                           sub=8, writable=False),
    "diag:ripple": Reg(0x43, [F("index", "u8", 2), F("flags", "u8", 3), F("bin0", "i16", 4), F("bin1", "i16", 6)],
                       sub=9, keyed="index", writable=False),
    "device_info": Reg(0x50, [F("hw", "u8", 1), F("fw", "u16", 2), F("device", "u32", 4)], writable=False),
    "system_status": Reg(0x51, [F("io", "u8", 1), F("uptime", "bf16", 2), F("fault_word", "u32", 4)], writable=False),
    "motor_status": Reg(0x52, [_bits_field("state", 1, 0, 4), _bits_field("mode", 1, 4, 4),
                               _bits_field("type", 2, 0, 2), _bits_field("phase", 2, 2, 2), _bits_field("ccw", 2, 4, 1),
                               _bits_field("enabled", 2, 5, 1), _bits_field("identified", 2, 6, 1),
                               _bits_field("calibrated", 2, 7, 1), F("temp", "u8", 3), F("power", "bf16", 4),
                               F("torque", "bf16", 6)], writable=False),
    "motion_status_1": Reg(0x53, [Field("pos_err", "i24q16", 1), F("vel_err", "bf16", 4),
                                  _wbits_field("queued", 6, 0, 4), _wbits_field("time_scale", 6, 4, 12, signed=True)],
                           writable=False),
    "motion_status_2": Reg(0x54, [F("motion_mode", "u8", 1), F("position", "q24", 2)], writable=False),
    "motion_status_3": Reg(0x55, [F("velocity", "f24", 1), F("accel", "bf16", 4), F("torque", "bf16", 6)],
                           writable=False),
    "homing_travel": Reg(0x56, [F("travel", "q32_56", 1)], writable=False),
    "encoder_position": Reg(0x57, [F("position", "q32_56", 1)], writable=False),
    "driver_status": Reg(0x58, [F("fault", "u8", 1), F("warn", "u8", 2), F("op_status", "u8", 3), F("temp", "u8", 4),
                                F("usage", "i8", 5), F("supply", "bf16", 6)], writable=False),
}


# The 24-bit q1.7.16 following error is the one field with its own kind.
def _i24q16_decode(self, d):
    return signext(d[self.byte] | (d[self.byte + 1] << 8) | (d[self.byte + 2] << 16), 24) / 65536.0


_base_decode = Field.decode


def _decode(self, d):
    if self.kind == "i24q16":
        return _i24q16_decode(self, d)
    return _base_decode(self, d)


Field.decode = _decode


def find_register(name):
    """Resolve 'name' or 'group:record' into (key, Reg), or exit with the list."""
    if name in REGISTERS:
        return name, REGISTERS[name]
    matches = sorted(k for k in REGISTERS if k.startswith(name + ":"))
    if matches:
        sys.exit(f"'{name}' is a sub-space; pick one of: {', '.join(matches)}")
    sys.exit(f"unknown register '{name}'. Try: {', '.join(sorted(REGISTERS))}")


# ============================================================================
# CAN link
# ============================================================================


class Link:
    """Thin wrapper over python-can with request/response correlation."""

    def __init__(self, args):
        kwargs = dict(interface=args.interface, channel=args.channel, bitrate=args.bitrate)
        if args.fd:
            kwargs["fd"] = True
            kwargs["data_bitrate"] = args.data_bitrate
        self.bus = can.Bus(**kwargs)
        self.fd = args.fd
        self.verbose = args.verbose

    def close(self):
        self.bus.shutdown()

    def send(self, can_id, data=b"", fd=None):
        """Transmit one frame, padding the payload to 8 bytes."""
        payload = bytes(data).ljust(8, b"\x00")[:8]
        msg = can.Message(arbitration_id=can_id, data=payload, is_extended_id=False,
                          is_fd=self.fd if fd is None else fd,
                          bitrate_switch=self.fd if fd is None else fd)
        if self.verbose:
            print(f"  TX {can_id:03X}  {payload.hex(' ')}")
        self.bus.send(msg)

    def recv(self, timeout=0.2):
        """Receive one frame, or None on timeout."""
        msg = self.bus.recv(timeout)
        if msg is not None and self.verbose:
            print(f"  RX {msg.arbitration_id:03X}  {bytes(msg.data).hex(' ')}")
        return msg

    def request(self, node, payload, timeout=0.5):
        """Send a command and wait for the matching response frame."""
        self.send(ID_COMMAND + node, payload)
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.recv(deadline - time.time())
            if msg is not None and msg.arbitration_id == ID_RESPONSE + node:
                return bytes(msg.data).ljust(8, b"\x00")
        return None

    def drain(self, seconds):
        """Collect every frame seen over a fixed window."""
        out = []
        deadline = time.time() + seconds
        while time.time() < deadline:
            msg = self.recv(deadline - time.time())
            if msg is not None:
                out.append(msg)
        return out

    def send_many(self, can_id, payloads):
        """Transmit a burst. A CAN link has no batching, so this is a loop."""
        for payload in payloads:
            self.send(can_id, payload)


def probe_drive(channel, timeout=0.35):
    """Ask a port directly whether an addressed drive is behind it.

    Sends a broadcast Device Info read and waits briefly for a framed reply.
    The reply identifier reveals the node address for free, since responses
    come back on 0x400 + node. Returns (is_drive, node_id_or_None).
    """
    try:
        import serial
    except ImportError:
        return False, None
    try:
        port = serial.Serial(channel, 115200, timeout=0.05)
    except Exception:
        return False, None
    try:
        cid = ID_COMMAND + NODE_BROADCAST
        body = bytes([cid & 0xFF, (cid >> 8) & 0x07, CMD["device_info"], 0, 0, 0, 0, 0, 0, 0])
        chk = 0
        for b in body:
            chk ^= b
        port.write(b"\xa5\x5a" + body + bytes([chk]))
        buf = bytearray()
        end = time.time() + timeout
        while time.time() < end:
            chunk = port.read(64)
            if chunk:
                buf.extend(chunk)
            idx = buf.find(b"\xa5\x5a")
            while idx >= 0 and len(buf) >= idx + 13:
                body = bytes(buf[idx + 2:idx + 12])
                c = 0
                for b in body:
                    c ^= b
                if c == buf[idx + 12]:
                    cid = ((body[1] << 8) | body[0]) & 0x7FF
                    if (cid & 0x700) == ID_RESPONSE:
                        return True, cid - ID_RESPONSE
                del buf[:idx + 2]
                idx = buf.find(b"\xa5\x5a")
        return False, None
    finally:
        port.close()


def cache_path():
    """Where the last known-good port is remembered between invocations."""
    base = os.environ.get("XDG_CACHE_HOME") or os.path.expanduser("~/.cache")
    return os.path.join(base, "tpcp_host_port")


def cache_read():
    """Last port that answered, or None."""
    try:
        with open(cache_path()) as fh:
            dev = fh.read().strip()
        return dev if dev and os.path.exists(dev) else None
    except OSError:
        return None


def cache_write(channel):
    """Remember a port that answered, best effort."""
    try:
        os.makedirs(os.path.dirname(cache_path()), exist_ok=True)
        with open(cache_path(), "w") as fh:
            fh.write(channel)
    except OSError:
        pass


def autodetect_channel():
    """Find the drive's console without being told where it is."""
    import glob
    candidates = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/ttyACM*") +
                        glob.glob("/dev/cu.usbserial*"))
    if not candidates:
        return None, "no USB serial devices found"
    consoles = []
    for dev in candidates:
        found, _ = probe_drive(dev)
        if not found:
            found, _, _ = sniff_port(dev, 1.2)
        if found:
            consoles.append(dev)
    if len(consoles) == 1:
        return consoles[0], f"auto-detected drive console on {consoles[0]}"
    if len(consoles) > 1:
        return None, ("several drive consoles found (" + ", ".join(consoles) + ") - name one with --channel")
    return None, ("no drive console found among " + ", ".join(candidates) +
                  " (an ST-Link's own VCP is not the drive: the drive's console is on the board's own USB port)")


def normalize_port(channel):
    """Prefer the callout device on macOS: /dev/tty.* can block on open."""
    if channel.startswith("/dev/tty."):
        alt = channel.replace("/dev/tty.", "/dev/cu.", 1)
        if os.path.exists(alt):
            return alt, f"using {alt} instead of {channel} (callout device)"
    return channel, None


def sniff_port(channel, seconds=1.5):
    """Listen to a serial port and work out what is on the other end.

    Returns (is_tunepulse_console, node_id_or_None, sample_text). A TunePulse
    drive emits ASCII heartbeat lines unprompted; an slcan adapter says nothing
    until asked, so nothing needs to be transmitted to tell them apart.
    """
    try:
        import serial
    except ImportError:
        return False, None, ""
    try:
        port = serial.Serial(channel, 115200, timeout=0.2)
    except Exception:
        return False, None, ""
    buf = bytearray()
    end = time.time() + seconds
    while time.time() < end:
        chunk = port.read(512)
        if chunk:
            buf.extend(chunk)
            if (b"[HB]" in buf) or (b"TunePulse" in buf) or (b"\xa5\x5a" in buf):
                break
    port.close()
    text = buf.decode("utf-8", "replace")
    is_console = ("[HB]" in text) or ("TunePulse" in text) or (b"\xa5\x5a" in buf)
    node = None
    match = re.search(r"can\[node=(\d+)", text)
    if match:
        node = int(match.group(1))
    return is_console, node, text


# ============================================================================
# Serial (USB CDC) transport
# ============================================================================

TP_SERIAL_BATCH = 4            # Frames per write when streaming a burst
TP_SERIAL_BATCH_DELAY = 0.002  # Pause between writes


class SerialFrame:
    """Minimal stand-in for can.Message so both links share the decoders."""

    def __init__(self, arbitration_id, data):
        self.arbitration_id = arbitration_id
        self.data = data


class SerialLink:
    """Framed protocol transport over the board's USB CDC console.

    Speaks the framing in src/api/transport/tp_serial.h:

        A5 5A | id_lo id_hi | d0..d7 | xor

    Anything outside a frame is diagnostic log text, printed through with --log.
    """

    SYNC = b"\xa5\x5a"
    FRAME_LEN = 13

    def __init__(self, args):
        try:
            import serial
        except ImportError:
            sys.exit("pyserial is required for --interface serial:  pip install pyserial")
        self.ser = serial.Serial(args.channel, baudrate=115200, timeout=0.02)
        self.buf = bytearray()
        self.text = bytearray()
        self.verbose = args.verbose
        self.show_log = args.log

    def close(self):
        self.ser.close()

    def _framed(self, can_id, data):
        body = bytes([can_id & 0xFF, (can_id >> 8) & 0x07]) + bytes(data).ljust(8, b"\x00")[:8]
        chk = 0
        for b in body:
            chk ^= b
        return self.SYNC + body + bytes([chk])

    def send(self, can_id, data=b"", fd=None):
        """Frame and transmit one protocol frame."""
        if self.verbose:
            print(f"  TX {can_id:03X}  {bytes(data).ljust(8, b'0')[:8].hex(' ')}")
        self.ser.write(self._framed(can_id, data))

    def send_many(self, can_id, payloads):
        """Frame a burst and hand it to the port a few frames per write.

        One frame per write is slower than the drive consumes fine segments;
        the whole burst in one write overruns the board's CDC receive buffer.
        """
        blob = b"".join(self._framed(can_id, p) for p in payloads)
        if self.verbose:
            print(f"  TX {can_id:03X}  x{len(payloads)} frames ({len(blob)} bytes)")
        step = TP_SERIAL_BATCH * self.FRAME_LEN
        for off in range(0, len(blob), step):
            self.ser.write(blob[off:off + step])
            self.ser.flush()
            time.sleep(TP_SERIAL_BATCH_DELAY)

    def _flush_text(self):
        while b"\n" in self.text:
            line, _, rest = self.text.partition(b"\n")
            self.text = bytearray(rest)
            out = line.decode("utf-8", "replace").rstrip("\r")
            if out and self.show_log:
                print(f"  | {out}")

    def recv(self, timeout=0.2):
        """Return the next protocol frame, printing any log text on the way."""
        deadline = time.time() + max(timeout, 0.0)
        while True:
            chunk = self.ser.read(256)
            if chunk:
                self.buf.extend(chunk)
            while True:
                idx = self.buf.find(self.SYNC)
                if idx < 0:
                    if len(self.buf) > 1:
                        self.text.extend(self.buf[:-1])
                        del self.buf[:-1]
                    self._flush_text()
                    break
                if idx:
                    self.text.extend(self.buf[:idx])
                    del self.buf[:idx]
                    self._flush_text()
                if len(self.buf) < self.FRAME_LEN:
                    break
                body = bytes(self.buf[2:12])
                chk = 0
                for b in body:
                    chk ^= b
                if chk != self.buf[12]:
                    self.text.append(self.buf[0])
                    del self.buf[0]
                    continue
                del self.buf[:self.FRAME_LEN]
                msg = SerialFrame(((body[1] << 8) | body[0]) & 0x7FF, body[2:])
                if self.verbose:
                    print(f"  RX {msg.arbitration_id:03X}  {bytes(msg.data).hex(' ')}")
                return msg
            if time.time() >= deadline:
                return None

    def request(self, node, payload, timeout=0.5):
        """Send a command and wait for the matching response frame."""
        self.send(ID_COMMAND + node, payload)
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.recv(deadline - time.time())
            if msg is not None and msg.arbitration_id == ID_RESPONSE + node:
                return bytes(msg.data).ljust(8, b"\x00")
        return None

    def drain(self, seconds):
        """Collect every frame seen over a fixed window."""
        out = []
        deadline = time.time() + seconds
        while time.time() < deadline:
            msg = self.recv(deadline - time.time())
            if msg is not None:
                out.append(msg)
        return out


# ============================================================================
# HID transport
# ============================================================================

TP_HID_VID = 0x0483          # STMicroelectronics
TP_HID_PID = 0x5741          # TunePulse composite device (see usb_hid_setup.h)
TP_HID_USAGE_PAGE = 0xFF00   # Vendor-defined page the control interface declares
TP_HID_ITF = 2               # Interface number of the HID function
TP_HID_REPORT_LEN = 64       # Bytes per report, both directions
TP_HID_REC_LEN = 10          # Bytes per protocol frame inside a report
TP_HID_REC_PER_REPORT = (TP_HID_REPORT_LEN - 1) // TP_HID_REC_LEN  # 6


def find_hid_path(preferred=None):
    """Return the OS path of the drive's vendor HID interface, or None."""
    try:
        import hid
    except ImportError:
        return None
    if isinstance(preferred, str):
        preferred = preferred.encode()
    try:
        entries = hid.enumerate(TP_HID_VID, TP_HID_PID)
    except Exception:
        return None
    fallback = None
    for d in entries:
        path = d.get("path")
        if preferred is not None and path == preferred:
            return path
        if d.get("interface_number", -1) == TP_HID_ITF or d.get("usage_page", 0) == TP_HID_USAGE_PAGE:
            if fallback is None:
                fallback = path
    return fallback


class HidLink:
    """Protocol transport over the drive's vendor HID interface (the fast path).

    Speaks the report layout in src/api/transport/tp_hid.h: byte 0 is the
    record count, then records of ten bytes (id_lo, id_hi | len<<4, payload).
    """

    def __init__(self, args):
        try:
            import hid
        except ImportError:
            sys.exit("hidapi is required for --interface hid:  pip install hidapi")
        path = args.channel
        if isinstance(path, str):
            path = path.encode()
        self.dev = hid.device()
        self.dev.open_path(path)
        self.dev.set_nonblocking(0)
        self.pending = []
        self.verbose = args.verbose

    def close(self):
        self.dev.close()

    @staticmethod
    def _record(can_id, data):
        payload = bytes(data).ljust(8, b"\x00")[:8]
        return bytes([can_id & 0xFF, ((can_id >> 8) & 0x07) | (8 << 4)]) + payload

    def _send_records(self, records):
        for off in range(0, len(records), TP_HID_REC_PER_REPORT):
            batch = records[off:off + TP_HID_REC_PER_REPORT]
            report = bytearray(TP_HID_REPORT_LEN)
            report[0] = len(batch)
            for i, rec in enumerate(batch):
                report[1 + i * TP_HID_REC_LEN:1 + (i + 1) * TP_HID_REC_LEN] = rec
            self.dev.write(b"\x00" + bytes(report))

    def send(self, can_id, data=b"", fd=None):
        if self.verbose:
            print(f"  TX {can_id:03X}  {bytes(data).ljust(8, b'0')[:8].hex(' ')}")
        self._send_records([self._record(can_id, data)])

    def send_many(self, can_id, payloads):
        if self.verbose:
            print(f"  TX {can_id:03X}  x{len(payloads)} frames")
        self._send_records([self._record(can_id, p) for p in payloads])

    def recv(self, timeout=0.2):
        if self.pending:
            return self.pending.pop(0)
        deadline = time.time() + max(timeout, 0.0)
        while True:
            left_ms = int((deadline - time.time()) * 1000)
            if left_ms <= 0:
                return None
            data = self.dev.read(TP_HID_REPORT_LEN, timeout_ms=left_ms)
            if data:
                count = min(data[0], TP_HID_REC_PER_REPORT)
                for i in range(count):
                    rec = data[1 + i * TP_HID_REC_LEN:1 + (i + 1) * TP_HID_REC_LEN]
                    if len(rec) < TP_HID_REC_LEN:
                        break
                    can_id = ((rec[1] & 0x07) << 8) | rec[0]
                    self.pending.append(SerialFrame(can_id, bytes(rec[2:10])))
                if self.pending:
                    msg = self.pending.pop(0)
                    if self.verbose:
                        print(f"  RX {msg.arbitration_id:03X}  {bytes(msg.data).hex(' ')}")
                    return msg
            if time.time() >= deadline:
                return None

    def request(self, node, payload, timeout=0.5):
        self.send(ID_COMMAND + node, payload)
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.recv(deadline - time.time())
            if msg is not None and msg.arbitration_id == ID_RESPONSE + node:
                return bytes(msg.data).ljust(8, b"\x00")
        return None

    def drain(self, seconds):
        out = []
        deadline = time.time() + seconds
        while time.time() < deadline:
            msg = self.recv(deadline - time.time())
            if msg is not None:
                out.append(msg)
        return out


# ============================================================================
# Frame decoding
# ============================================================================


def flag_names(value, names):
    """Comma-separated names of the set bits."""
    return ",".join(n for i, n in enumerate(names) if value & (1 << i))


def fmt(v):
    """Compact rendering of a field value."""
    if isinstance(v, float):
        return "nan" if math.isnan(v) else f"{v:.6g}"
    return str(v)


def register_for_frame(d):
    """Find the Reg matching a received frame, or None."""
    cmd = d[0] & 0x7F if d[0] < 0xC0 else d[0]
    if 0x70 <= cmd <= 0x7F:
        return None
    for key, reg in REGISTERS.items():
        if reg.cmd == cmd and (reg.sub is None or reg.sub == d[1]):
            return key, reg
    return None


def decode_frame(d):
    """Render one 8-byte protocol frame as human-readable text."""
    d = bytes(d).ljust(8, b"\x00")
    cmd = d[0]
    if cmd >= 0xE0:
        t21 = ((cmd & 0x1F) << 16) | (d[1] << 8) | d[2]
        vel = bf16_to_float((d[3] << 8) | d[4])
        pos = signext((d[5] << 16) | (d[6] << 8) | d[7], 24) / 65536.0
        force = bf16_to_float((d[1] << 8) | d[2])
        return (f"burst   t21={t21} (frame={cmd & 0x1F} force={fmt(force)}) vel={vel:+.3f}RPS "
                f"pos24={pos:+.4f}rev")
    if 0x70 <= cmd <= 0x7F:
        qts = d[1] | (d[2] << 8)
        wme = d[4] | (d[5] << 8)
        return (f"BEAT    seq={cmd & 0xF} q={qts & 0xF} scale={signext(qts >> 4, 12) / 256:.3f}x "
                f"state={STATES[d[3] & 0xF]} fault={FAULT_GROUPS.get(d[3] >> 4, d[3] >> 4)} "
                f"warn={wme & 0xF} mode={MODES[min((wme >> 4) & 0x3F, 5)]} "
                f"err={signext(wme >> 10, 6) / 16:+.3f} load={signext(d[6], 8)} io=0x{d[7]:02X}")
    if cmd == CMD["ack"]:
        return f"ACK     {CMD_NAME.get(d[1], hex(d[1]))} detail={d[3]}"
    if cmd == CMD["nack"]:
        return f"NACK    {CMD_NAME.get(d[1] & 0xFF, hex(d[1]))}: {ERRORS.get(d[2], hex(d[2]))} (detail={d[3]})"
    if cmd >= 0xC0:
        name = CMD_NAME.get(cmd, f"0x{cmd:02X}")
        return f"{name:7s} {d.hex(' ')}"
    found = register_for_frame(d)
    if found is None:
        return f"0x{cmd:02X}    {d.hex(' ')}"
    key, reg = found
    vals = reg.decode(d)
    extra = ""
    if key == "system_status":
        w = vals["fault_word"]
        extra = (f"  [fault={flag_names(w & 0xFF, FAULT_BITS) or '-'} warn={flag_names((w >> 8) & 0xFF, WARN_BITS) or '-'}"
                 f" state={STATES[(w >> 16) & 0xF]} mode={MODES[min((w >> 20) & 0xF, 5)]}]")
    elif key == "driver_status":
        extra = f"  [fault={flag_names(vals['fault'], FAULT_BITS) or '-'} warn={flag_names(vals['warn'], WARN_BITS) or '-'}]"
    elif key == "motor_status":
        extra = f"  [{STATES[vals['state']]}/{MODES[min(vals['mode'], 5)]}]"
    elif key == "discovery_assign" or key == "discovery_search":
        vals = dict(vals)
        for k in ("uid", "prefix"):
            if k in vals:
                vals[k] = f"{vals[k]:012X}"
    elif key == "diag:calibration":
        fl = vals["flags"]
        extra = ("  [" + ",".join(n for b, n in ((1, "calibrated"), (2, "settling"), (4, "persisting"),
                                                (8, "failed"), (16, "identified"), (32, "profile")) if fl & b) + "]")
    body = " ".join(f"{k}={fmt(v)}" for k, v in vals.items())
    return f"{key:18s} {body}{extra}"


def expect_ok(reply, what):
    """Print the outcome of a command and say whether it was accepted."""
    if reply is None:
        print(f"{what}: NO RESPONSE")
        return False
    print(f"{what}: {decode_frame(reply)}")
    return reply[0] != CMD["nack"]


# ============================================================================
# Register access helpers
# ============================================================================


def read_register(link, node, name, key=None, timeout=0.5):
    """Read one register by name; returns (Reg, values) or (Reg, None)."""
    _, reg = find_register(name)
    reply = link.request(node, reg.request(key), timeout=timeout)
    if reply is None or reply[0] == CMD["nack"]:
        return reg, None
    return reg, reg.decode(reply)


def write_register(link, node, name, changes, key=None, timeout=0.5):
    """Read-modify-write a register; returns the raw reply or None."""
    _, reg = find_register(name)
    if not reg.writable:
        sys.exit(f"'{name}' is read only")
    reply = link.request(node, reg.request(key), timeout=timeout)
    if reply is None or reply[0] == CMD["nack"]:
        return reply
    d = bytearray(reply)
    d[0] = reg.cmd | CMD_WRITE
    for fname, value in changes.items():
        f = reg.by_name(fname)
        f.encode(d, value if not isinstance(value, str) else f.parse(value))
    return link.request(node, bytes(d), timeout=timeout)


def write_fields(link, node, name, timeout=0.5, **changes):
    """Write named fields of a register and return True on acceptance."""
    reply = write_register(link, node, name, changes, timeout=timeout)
    return reply is not None and reply[0] != CMD["nack"]


def system_control(link, node, enable=0, calibration=0, action=0, burst_step=None):
    """Write System Control with the given bytes (burst step kept unless given)."""
    reply = link.request(node, bytes([CMD["system_control"]]))
    if reply is None:
        return None
    step = reply[2] if burst_step is None else burst_step
    payload = bytes([CMD["system_control"] | CMD_WRITE, enable, step, calibration, action, 0, 0, 0])
    return link.request(node, payload)


# ============================================================================
# Motion helpers
# ============================================================================


def mt1_flags(mode, is_abs=False, pos_mode=0, torq_mode=0, time_mode=0, io_trig=False,
              is_vel=False, is_acc=False, is_jrk=False):
    """Pack a Motion Task 1 flags word (LSB first, as the firmware reads it)."""
    return ((mode & 0xF) | (0x10 if is_abs else 0) | ((pos_mode & 3) << 5) | ((torq_mode & 3) << 7) |
            ((time_mode & 3) << 9) | (0x800 if io_trig else 0) | (0x1000 if is_vel else 0) |
            (0x2000 if is_acc else 0) | (0x4000 if is_jrk else 0))


def mt1(seg, mode, time_100us=0, axis=0, **flags):
    return bytes([CMD["motion_task_1"], seg, axis]) + struct.pack("<H", mt1_flags(mode, **flags)) + \
        (int(time_100us) & 0xFFFFFF).to_bytes(3, "little")


def mt2(seg, revolutions):
    return bytes([CMD["motion_task_2"], seg]) + (int(round(revolutions * (1 << 24))) & 0xFFFFFFFFFFFF).to_bytes(6, "little")


def mt3(seg, torque=float("nan"), velocity=float("nan")):
    t = BF16_NAN if is_nan(torque) else float_to_bf16(torque)
    v = F24_NAN if is_nan(velocity) else float_to_f24(velocity)
    return bytes([CMD["motion_task_3"], seg]) + struct.pack("<H", t) + b"\x00" + v.to_bytes(3, "little")


def mt4(seg, accel=float("nan"), jerk=float("nan"), io_pos=0, io_neg=0):
    a = BF16_NAN if is_nan(accel) else float_to_bf16(accel)
    j = BF16_NAN if is_nan(jerk) else float_to_bf16(jerk)
    return bytes([CMD["motion_task_4"], seg]) + struct.pack("<HH", a, j) + bytes([io_pos, io_neg])


def burst_pvt(time21, vel, pos_rev):
    """Pack a PVT burst frame (MSB first)."""
    p = int(round(pos_rev * 65536.0)) & 0xFFFFFF
    v = float_to_bf16(vel)
    t = time21 & 0x1FFFFF
    return bytes([0xE0 | (t >> 16), (t >> 8) & 0xFF, t & 0xFF, v >> 8, v & 0xFF, p >> 16, (p >> 8) & 0xFF, p & 0xFF])


# ============================================================================
# Commands
# ============================================================================


def cmd_scan(link, args):
    """Find unaddressed nodes by binary HashUID search and hand out addresses.

    A probe at depth d with prefix P is answered by every unassigned node whose
    HashUID starts with those d bits; the answers are identical and merge on
    the bus, so a reply means "populated" and silence means "empty". The tree
    is walked depth first; a probe at depth 48 is answered by exactly one node.
    """
    def probe(depth, prefix):
        payload = bytes([CMD["discovery_search"], depth]) + prefix.to_bytes(6, "little")
        link.send(ID_COMMAND + NODE_BROADCAST, payload)
        for m in link.drain(args.window):
            d = bytes(m.data).ljust(8, b"\x00")
            if m.arbitration_id == ID_RESPONSE + NODE_UNASSIGNED and d[0] == CMD["discovery_search"]:
                return True
        return False

    print("discovery: probing the HashUID tree")
    found = []
    stack = [(0, 0)]
    probes = 0
    while stack and len(found) < args.limit:
        depth, prefix = stack.pop()
        probes += 1
        if not probe(depth, prefix):
            continue
        if depth == UID_BITS:
            found.append(prefix)
            print(f"discovery: found HashUID {prefix:012X} ({probes} probes)")
            continue
        stack.append((depth + 1, prefix | (1 << depth)))
        stack.append((depth + 1, prefix))
    if not found:
        print("discovery: no unaddressed node answered")
        return 0

    next_id = args.first_id
    for uid in found:
        payload = bytes([CMD["discovery_assign"] | CMD_WRITE, next_id]) + uid.to_bytes(6, "little")
        link.send(ID_COMMAND + NODE_BROADCAST, payload)
        confirmed = None
        for m in link.drain(0.3):
            if m.arbitration_id == ID_RESPONSE + next_id:
                confirmed = bytes(m.data).ljust(8, b"\x00")
        if confirmed is not None:
            print(f"           {uid:012X} -> node {next_id}: {decode_frame(confirmed)}")
        else:
            # Double-check by pinging the identity, as the specification asks.
            ping = bytes([CMD["discovery_assign"], 0]) + uid.to_bytes(6, "little")
            link.send(ID_COMMAND + NODE_BROADCAST, ping)
            got = [m for m in link.drain(0.3) if m.arbitration_id == ID_RESPONSE + next_id]
            print(f"           {uid:012X} -> node {next_id}: "
                  f"{'confirmed by ping' if got else 'WARNING: not confirmed'}")
        next_id += 1
    print(f"discovery: complete, {len(found)} node(s)")
    return 0


def cmd_ping(link, args):
    """Inventory every addressed node with a broadcast identity read."""
    link.send(ID_COMMAND + NODE_BROADCAST, bytes([CMD["discovery_assign"]]) + b"\x00" * 7)
    # Keyed by identity, not by address: every unassigned drive answers on
    # 0x4FF, and keying by address would collapse them into one entry.
    seen = {}
    for m in link.drain(args.window):
        d = bytes(m.data).ljust(8, b"\x00")
        if (m.arbitration_id & 0x700) == ID_RESPONSE and d[0] == CMD["discovery_assign"]:
            seen[int.from_bytes(d[2:8], "little")] = d[1]
    for uid, node in sorted(seen.items(), key=lambda kv: (kv[1], kv[0])):
        print(f"  node {node:3d}  HashUID {uid:012X}")
    if not seen:
        print("  nobody answered")
    return 0


INFO_REGISTERS = ["device_info", "system_status", "motor_status", "driver_status", "motion_status_1",
                  "motion_status_2", "motion_status_3", "motor_props_1", "motor_props_2", "system_control",
                  "currents:dq", "diag:comm", "diag:calibration", "diag:queue", "diag:persist",
                  "diag:calib_result", "bus", "watchdog"]


def cmd_info(link, args):
    """Read the interesting registers of one node."""
    for name in INFO_REGISTERS:
        reg = REGISTERS[name]
        reply = link.request(args.node, reg.request())
        print(f"  {decode_frame(reply) if reply else name + ': NO RESPONSE'}")
    return 0


def parse_key(reg, extra):
    """Pull the keyed selector (e.g. function=3) out of extra arguments."""
    if reg.keyed is None:
        return None, extra
    key, rest = None, []
    for item in extra:
        if item.startswith(reg.keyed + "="):
            key = reg.by_name(reg.keyed).parse(item.split("=", 1)[1])
        else:
            rest.append(item)
    return key, rest


def cmd_get(link, args):
    """Read one register (or a whole sub-space with 'group:*')."""
    if args.reg.endswith(":*"):
        group = args.reg[:-2]
        names = sorted(k for k in REGISTERS if k.startswith(group + ":"))
        if not names:
            sys.exit(f"unknown sub-space '{group}'")
    else:
        names = [find_register(args.reg)[0]]
    for name in names:
        reg = REGISTERS[name]
        key, _ = parse_key(reg, args.fields)
        reply = link.request(args.node, reg.request(key))
        print(f"  {decode_frame(reply) if reply else name + ': NO RESPONSE'}")
    return 0


def cmd_set(link, args):
    """Write fields of one register (read-modify-write) and show what is in force."""
    name, reg = find_register(args.reg)
    key, items = parse_key(reg, args.fields)
    changes = {}
    for item in items:
        if "=" not in item:
            sys.exit(f"expected field=value, got '{item}'")
        f, v = item.split("=", 1)
        if f not in {x.name for x in reg.fields}:
            sys.exit(f"'{name}' has no field '{f}'. Fields: {', '.join(x.name for x in reg.fields)}")
        changes[f] = v
    if not changes:
        sys.exit("nothing to write: give at least one field=value")
    reply = write_register(link, args.node, name, changes, key=key)
    print(f"  {decode_frame(reply) if reply else 'NO RESPONSE'}")
    return 0 if reply and reply[0] != CMD["nack"] else 1


def cmd_enable(link, args):
    """Enable or disable the output stage."""
    reply = system_control(link, args.node, enable=SYS_ENABLE if args.state == "on" else SYS_DISABLE)
    return 0 if expect_ok(reply, "enable") else 1


def cmd_mode(link, args):
    """Select the control mode, optionally energizing at the same time."""
    if not write_fields(link, args.node, "motor_props_1", mode=MODES.index(args.mode)):
        print("mode: rejected")
        return 1
    print(f"mode: {args.mode}")
    if args.enable:
        return 0 if expect_ok(system_control(link, args.node, enable=SYS_ENABLE), "enable") else 1
    return 0


def wait_settled(link, node, tol_rev=0.01, timeout=60.0):
    """Poll Motion Status 1 until the following error stays inside the window."""
    start = time.time()
    settled_polls, last, last_queued = 0, None, 0
    while time.time() - start < timeout:
        _, vals = read_register(link, node, "motion_status_1", timeout=0.3)
        if vals is None:
            time.sleep(0.05)
            continue
        last, last_queued = vals["pos_err"], vals["queued"]
        if abs(last) <= tol_rev and last_queued == 0:
            settled_polls += 1
            if settled_polls >= 3:
                print(f"  settled after {time.time() - start:.2f}s (perr={last:+.5f}rev)")
                return True
        else:
            settled_polls = 0
        time.sleep(0.02)
    if last is not None and last_queued and abs(last) <= tol_rev:
        print(f"  QUEUE NOT DRAINING: {last_queued} segment(s) waiting, rotor on target (perr={last:+.5f}rev).")
        print("  The queue only advances in MOTION mode with the stage enabled; clear it with: queue <node> clear")
    elif last is not None:
        print(f"  DID NOT SETTLE in {timeout:.0f}s: perr={last:+.5f}rev, {last_queued} queued")
    return False


def cmd_move(link, args):
    """Command a position setpoint (single-frame Motion Task 2: CSP) or a queued S-curve."""
    if args.scurve or args.relative:
        seg = 1 + int(time.time() * 10) % 250
        frames = [mt1(seg, MT_MODES["scurve"], is_abs=not args.relative, pos_mode=1,
                      is_vel=args.velocity is not None),
                  mt2(seg, args.revolutions)]
        if args.velocity is not None:
            frames.append(mt3(seg, velocity=args.velocity))
        link.send_many(ID_COMMAND + args.node, frames)
        print(f"  queued S-curve to {args.revolutions:+.4f}rev ({'relative' if args.relative else 'absolute'})")
    else:
        link.send(ID_COMMAND + args.node, mt2(0, args.revolutions))
        print(f"  position setpoint {args.revolutions:+.4f}rev")
    if args.wait:
        return 0 if wait_settled(link, args.node, args.tolerance, args.timeout) else 1
    return 0


def cmd_jog(link, args):
    """Command a velocity setpoint (single-frame Motion Task 3: CSV)."""
    link.send(ID_COMMAND + args.node, mt3(0, velocity=args.rps))
    print(f"  velocity setpoint {args.rps:+.3f}RPS")
    return 0


def cmd_torque(link, args):
    """Command a torque setpoint (single-frame Motion Task 3: CST)."""
    link.send(ID_COMMAND + args.node, mt3(0, torque=args.value))
    print(f"  torque setpoint {args.value:+.4g} (N*m, or A when Kt is unknown)")
    return 0


def cmd_stop(link, args):
    """Stop with a selectable end state (a lone Motion Task 1)."""
    mode = {"hold": "hold", "freewheel": "freewheel", "brake": "brake", "reset": "reset"}[args.style]
    link.send(ID_COMMAND + args.node, mt1(0, MT_MODES[mode]))
    print(f"  stop: {args.style}")
    return 0


def query_queue(link, node):
    """(queued, free, dropped) from the queue diagnostics, or None."""
    _, vals = read_register(link, node, "diag:queue", timeout=0.3)
    if vals is None:
        return None
    return vals["queued"], vals["free"], vals["dropped"]


def node_time(link, node):
    """The node clock in 50 us units, or None."""
    _, vals = read_register(link, node, "bus")
    return None if vals is None else vals["sync"]


def stream_pvt_leg(link, node, start_rev, revolutions, seconds, segments):
    """Stream one leg as PVT burst frames, paced against the drive's free slots."""
    per = revolutions / segments
    vel = revolutions / seconds
    t0 = node_time(link, node)
    if t0 is None:
        print("  no node clock: cannot time-tag a PVT stream")
        return start_rev
    print(f"  leg {revolutions:+.3f}rev over {seconds:.2f}s ({vel:+.3f} RPS), {segments} segments")
    step_us = seconds * 1e6 / segments
    frames = []
    for i in range(1, segments + 1):
        t_us = t0 * 50 + 50000 + i * step_us          # 50 ms of lead so the first tag is in the future
        t21 = int(t_us / 250) & 0x1FFFFF
        end_vel = vel if i < segments else 0.0
        frames.append(burst_pvt(t21, end_vel, start_rev + per * i))
    sent, dropped0 = 0, None
    deadline = time.time() + seconds + 30.0
    while sent < segments:
        stat = query_queue(link, node)
        if stat is None:
            link.send_many(ID_COMMAND + node, frames[sent:])
            return start_rev + revolutions
        _, free, dropped = stat
        if dropped0 is None:
            dropped0 = dropped
        elif dropped > dropped0:
            print(f"  WARNING: the drive dropped {dropped - dropped0} segment(s)")
            dropped0 = dropped
        room = min(free, segments - sent)
        if room > 0:
            link.send_many(ID_COMMAND + node, frames[sent:sent + room])
            sent += room
        else:
            time.sleep(min(seconds / segments, 0.05))
        if time.time() > deadline:
            print(f"  WARNING: queue never drained, {segments - sent} segments not sent")
            break
    return start_rev + revolutions


def require_motion_ready(link, args):
    """Enable the stage (and let the burst select MOTION mode) before streaming."""
    _, vals = read_register(link, args.node, "system_control")
    if vals is None:
        print(f"node {args.node} is not answering")
        return False
    if vals["enable"] != SYS_ENABLE:
        if not args.set_mode:
            print("the output stage is disabled: queued segments would never run (use --set-mode)")
            return False
        if not expect_ok(system_control(link, args.node, enable=SYS_ENABLE), "  enable"):
            return False
    if vals["burst_step"] != 0:
        print(f"  burst step is {vals['burst_step']} (PVcTF): switching to the time-tagged PVT stream")
        system_control(link, args.node, burst_step=0)
    return True


def cmd_pvt(link, args):
    """Run a profiled move through the trajectory queue as a PVT burst stream."""
    if not require_motion_ready(link, args):
        return 1
    _, pos = read_register(link, args.node, "motion_status_2")
    here = pos["position"] if pos else 0.0
    two_way = args.back or (args.cycles > 1)
    legs = (1.0, -1.0) if two_way else (1.0,)
    print(f"{args.cycles} cycle(s), {args.revolutions:+.3f}rev per leg over {args.seconds:.2f}s, "
          f"{args.segments} segments per leg")
    for _ in range(args.cycles):
        for direction in legs:
            here = stream_pvt_leg(link, args.node, here, args.revolutions * direction, args.seconds, args.segments)
            if args.wait:
                if not wait_settled(link, args.node, args.tolerance, args.seconds + 15.0):
                    return 1
            else:
                time.sleep(args.seconds + 0.2)
    return 0


def cmd_queue(link, args):
    """Inspect or control the trajectory queue."""
    if args.action == "status":
        _, vals = read_register(link, args.node, "diag:queue")
        print(f"  {vals if vals else 'NO RESPONSE'}")
        return 0
    if args.action == "clear":
        link.send(ID_COMMAND + args.node, mt1(0, MT_MODES["reset"]))
        print("  queue cleared")
        return 0
    sys.exit("pause/resume/scale are not part of TPCP v1")


def cmd_telemetry(link, args):
    """Configure one cyclic telemetry slot (period 0 disables it)."""
    reg = REGISTERS.get(args.chunk) if ":" in args.chunk or args.chunk in REGISTERS else None
    if args.chunk == "pulsebeat":
        code = 0x70
    elif reg is not None:
        code = reg.cmd
    else:
        sys.exit(f"'{args.chunk}' is not a readable register")
    payload = bytes([CMD["telemetry"] | CMD_WRITE, args.slot, code if args.period else 0,
                     0x01 if args.period else 0]) + struct.pack("<HH", args.period, 0)
    reply = link.request(args.node, payload)
    print(f"  {decode_frame(reply) if reply else 'NO RESPONSE'}")
    return 0


def cmd_watchdog(link, args):
    """Configure the master-liveness watchdog (timeout 0 disables it)."""
    reply = write_register(link, args.node, "watchdog", {"timeout": args.timeout / 1000.0})
    print(f"  {decode_frame(reply) if reply else 'NO RESPONSE'}")
    return 0


def cmd_save(link, args):
    """Persist the configuration (the drive must be disabled)."""
    catalog = {"all": 0xFFFFFF, "config": 1, "motor": 2}[args.target]
    payload = bytes([CMD["save_config"]]) + catalog.to_bytes(3, "little") + struct.pack("<I", SAVE_KEY)
    return 0 if expect_ok(link.request(args.node, payload), "save") else 1


def cmd_defaults(link, args):
    """Restore defaults into the live shadow, or erase the stored record."""
    if args.erase:
        print("WARNING: this erases the stored configuration. The node will come back UNADDRESSED.")
        key = ERASE_KEY
    else:
        key = DEFAULTS_KEY
    payload = bytes([CMD["save_config"]]) + (1).to_bytes(3, "little") + struct.pack("<I", key)
    return 0 if expect_ok(link.request(args.node, payload), "defaults") else 1


def cmd_calibrate(link, args):
    """Start a calibration routine and poll its progress."""
    bits = CAL_BITS[args.kind]
    if not expect_ok(system_control(link, args.node, calibration=bits), "calibrate"):
        return 1
    print("polling calibration status (Ctrl-C to stop)...")
    try:
        while True:
            reply = link.request(args.node, REGISTERS["diag:calibration"].request())
            if reply:
                print(f"  {decode_frame(reply)}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        return 0


def cmd_reset(link, args):
    """Clear latched faults."""
    return 0 if expect_ok(system_control(link, args.node, action=SYS_FAULT_RESET), "fault reset") else 1


def ripple_table(link, node):
    """Read the 64-bin torque-ripple map [mA] and its flags (bit0 enabled, bit1 learning)."""
    bins, flags = [], 0
    for i in range(0, 64, 2):
        _, v = read_register(link, node, "diag:ripple", key=i)
        if v is None:
            return None, 0
        bins += [v["bin0"], v["bin1"]]
        flags = v["flags"]
    return bins, flags


def ripple_harmonic(bins, k):
    """Amplitude [mA] and phase [deg] of the k-th harmonic of the map over one electrical period."""
    n = len(bins)
    re = sum(b * math.cos(2.0 * math.pi * k * i / n) for i, b in enumerate(bins))
    im = sum(b * math.sin(2.0 * math.pi * k * i / n) for i, b in enumerate(bins))
    return 2.0 * math.hypot(re, im) / n, math.degrees(math.atan2(im, re))


def cmd_ripple(link, args):
    """Learn, clear or show the torque-ripple feed-forward map (RAM only until a reset)."""
    if args.what == "clear":
        return 0 if expect_ok(system_control(link, args.node, action=SYS_RIPPLE_CLEAR), "ripple clear") else 1
    if args.what == "learn":
        # The loop's reaction is only a faithful copy of the ripple torque well
        # below the spring/rotor resonance, so the learning move is SLOW: at
        # 0.05 RPS the 2x and 4x electrical lines sit at 5 and 10 Hz. One full
        # revolution averages over every pole pair.
        if not require_motion_ready(link, args):
            return 1
        seconds = args.rev / args.rps
        seg = 1 + int(time.time() * 10) % 250
        link.send_many(ID_COMMAND + args.node, [mt1(seg, MT_MODES["scurve"], is_abs=False, pos_mode=1, is_vel=True),
                                                mt2(seg, args.rev), mt3(seg, velocity=args.rps)])
        print(f"  learning over {args.rev:g} rev at {args.rps:g} RPS ({seconds:.0f} s)")
        time.sleep(1.0)                                    # Past the ramp
        if not expect_ok(system_control(link, args.node, action=SYS_RIPPLE_LEARN), "ripple learn"):
            return 1
        time.sleep(max(seconds - 2.0, 0.5))                # Stop before the final ramp
        if not expect_ok(system_control(link, args.node, action=SYS_RIPPLE_COMMIT), "ripple commit"):
            return 1
        time.sleep(1.5)
    bins, flags = ripple_table(link, args.node)
    if bins is None:
        print("  the node has no ripple map register (old firmware?)")
        return 1
    print(f"  map {'enabled' if flags & 1 else 'disabled'}{', learning' if flags & 2 else ''}: "
          f"{min(bins):+d}..{max(bins):+d} mA, harmonics " +
          ", ".join(f"{k}x {ripple_harmonic(bins, k)[0]:.0f} mA" for k in (1, 2, 4, 8)))
    for row in range(0, 64, 16):
        print("  " + " ".join(f"{b:+4d}" for b in bins[row:row + 16]))
    return 0


def cmd_reboot(link, args):
    """Reboot, optionally into the USB DFU bootloader."""
    action = SYS_REBOOT_DFU if args.dfu else SYS_REBOOT
    return 0 if expect_ok(system_control(link, args.node, action=action), "reboot") else 1


def cmd_estop(link, args):
    """Broadcast a freewheel stop and a disable to every node."""
    link.send(ID_COMMAND + NODE_BROADCAST, mt1(0, MT_MODES["freewheel"]))
    link.send(ID_COMMAND + NODE_BROADCAST, bytes([CMD["system_control"] | CMD_WRITE, SYS_DISABLE, 0, 0, 0, 0, 0, 0]))
    print("emergency stop broadcast")
    return 0


def cmd_sync(link, args):
    """Broadcast the host clock so PVT time tags line up on every node."""
    now50 = int(time.time() * 1e6 / 50) & 0xFFFFFFFF
    payload = bytes([CMD["bus"] | CMD_WRITE, 0, 0, 0]) + struct.pack("<I", now50 or 1)
    link.send(ID_COMMAND + NODE_BROADCAST, payload)
    print(f"  sync broadcast: {now50}")
    return 0


def cmd_monitor(link, args):
    """Decode and print every protocol frame on the bus."""
    print("monitoring (Ctrl-C to stop)")
    t0 = time.time()
    tags = {ID_EMCY: "EMCY", ID_COMMAND: "CMD ", ID_RESPONSE: "RESP", ID_TELEMETRY: "TLM "}
    try:
        while True:
            msg = link.recv(1.0)
            if msg is None:
                continue
            cid = msg.arbitration_id
            tag = tags.get(cid & 0x700, "????")
            node = cid & 0xFF
            if args.node is not None and node != args.node and node != NODE_BROADCAST:
                continue
            print(f"{time.time() - t0:8.3f} {tag} {node:3d}  {decode_frame(bytes(msg.data))}")
    except KeyboardInterrupt:
        return 0


def cmd_raw(link, args):
    """Send an arbitrary frame, for poking at the protocol by hand."""
    payload = bytes.fromhex(args.hex.replace(" ", "")).ljust(8, b"\x00")[:8]
    reply = link.request(args.node, payload)
    print(f"  {decode_frame(reply) if reply else 'NO RESPONSE'}")
    return 0


# ============================================================================
# Black box (register 0x44, dev builds)
# ============================================================================

REG_TRACE = 0x44            # Control-rate recorder: one 24-byte record per pass, read in six parts
TRACE_FIELDS = ("angle_el", "iq_ma", "id_ma", "uq_cv", "ud_cv", "icmd_ma", "ia_ma", "ib_ma",
                "ua_cv", "ub_cv", "vbus_cv", "perr_mrev")


def _trace_part(link, node, part, index):
    """Read one part of one record (or the ring state for part 0); None on NACK."""
    reply = link.request(node, bytes([REG_TRACE, part]) + struct.pack("<H", index) + b"\x00" * 4)
    if not reply or reply[0] != REG_TRACE:
        return None
    return struct.unpack("<HHH", reply[2:8])      # index (depth for part 0), a, b


def cmd_trace(link, args):
    """Dump the drive's black box: the control passes around the last fault, as CSV."""
    state = _trace_part(link, args.node, 0, 0)
    if state is None:
        print("  trace: not implemented in this firmware (build with TP_TRACE_DEPTH)")
        return 1
    depth, count, flags = state
    frozen = flags & 1
    fault = flags >> 8
    print(f"  trace: depth={depth} records={count} frozen={frozen} fault_at_freeze=0x{fault:02x}"
          + ("" if frozen else "  (ring still recording: only the last passes, no trip captured)"))
    if not frozen and not args.force:
        print("  nothing frozen; pass --force to dump the live ring anyway")
        return 0
    s16 = lambda v: v - 65536 if v >= 32768 else v            # Wire u16 -> signed
    rows = []
    for i in range(count):
        parts = [_trace_part(link, args.node, k, i) for k in range(1, 7)]
        if any(p is None for p in parts):
            print(f"  read failed at record {i}")
            break
        vals = []
        for k, (_, a, b) in enumerate(parts):
            vals.append(a if k == 0 else s16(a))                # angle_el is unsigned, the rest signed
            vals.append(s16(b))
        rows.append(vals)
    out = args.out or f"trace_node{args.node}.csv"
    with open(out, "w") as f:
        f.write("i," + ",".join(TRACE_FIELDS) + "\n")
        for i, r in enumerate(rows):
            f.write(str(i) + "," + ",".join(str(v) for v in r) + "\n")
    print(f"  {len(rows)} records written to {out}"
          + (f" (the fault edge is ~{depth // 8} records before the end)" if frozen else ""))
    return 0


# ============================================================================
# Motor speaker (MIDI control, register 0xD0)
# ============================================================================

TONE_MIN_HZ = 40        # The drive refuses lower notes
TONE_MAX_HZ = 2000      # ... and higher ones
MIN_EVENT_S = 0.02      # Changes closer together than this are not separately audible


def midi_frame(note, volume=128, duration_ms=0.0, attack=0, decay=0):
    """Pack a MIDI Control frame."""
    return bytes([CMD["midi"], 0, note & 0x7F, volume & 0xFF]) + struct.pack("<H", float_to_bf16(duration_ms)) + \
        bytes([attack & 0xFF, decay & 0xFF])


def emit(schedule, t, note):
    """Append a schedule entry, collapsing anything too brief to be heard."""
    if schedule and (t - schedule[-1][0]) < MIN_EVENT_S:
        schedule[-1] = (schedule[-1][0], note)
        return
    schedule.append((t, note))


def _vlq(b, i):
    """Read one MIDI variable-length quantity. Returns (value, next index)."""
    v = 0
    while True:
        c = b[i]
        i += 1
        v = (v << 7) | (c & 0x7F)
        if not c & 0x80:
            return v, i


def fit_note(pitch):
    """Octave-shift a MIDI note into the range the drive accepts."""
    while 440.0 * 2 ** ((pitch - 69) / 12.0) < TONE_MIN_HZ:
        pitch += 12
    while 440.0 * 2 ** ((pitch - 69) / 12.0) > TONE_MAX_HZ:
        pitch -= 12
    return pitch


def parse_midi_melody(path, start=0.0, end=30.0, channel=None):
    """Reduce a standard MIDI file to a monophonic schedule of (time, note); note 0 is a rest."""
    data = open(path, "rb").read()
    if data[:4] != b"MThd":
        sys.exit(f"{path}: not a standard MIDI file")
    hlen = struct.unpack(">I", data[4:8])[0]
    _fmt, ntrk, div = struct.unpack(">HHH", data[8:14])
    if div & 0x8000:
        sys.exit("SMPTE-timed MIDI files are not supported")
    pos = 8 + hlen
    events, tempos = [], []
    for _ in range(ntrk):
        if data[pos:pos + 4] != b"MTrk":
            break
        ln = struct.unpack(">I", data[pos + 4:pos + 8])[0]
        i, track_end, tick, run = pos + 8, pos + 8 + ln, 0, None
        while i < track_end:
            d, i = _vlq(data, i)
            tick += d
            st = data[i]
            if st & 0x80:
                run = st
                i += 1
            else:
                st = run
            if st == 0xFF:
                mt = data[i]
                i += 1
                l, i = _vlq(data, i)
                if mt == 0x51:
                    tempos.append((tick, int.from_bytes(data[i:i + l], "big")))
                i += l
            elif st in (0xF0, 0xF7):
                l, i = _vlq(data, i)
                i += l
            else:
                hi = st & 0xF0
                n = 2 if hi in (0x80, 0x90, 0xA0, 0xB0, 0xE0) else 1
                d1 = data[i]
                d2 = data[i + 1] if n == 2 else 0
                i += n
                if hi == 0x90 and d2 > 0:
                    events.append((tick, 1, d1, st & 0x0F))
                elif hi == 0x80 or (hi == 0x90 and d2 == 0):
                    events.append((tick, 0, d1, st & 0x0F))
        pos = track_end
    tempos.sort()
    if not tempos:
        tempos = [(0, 500000)]

    def to_seconds(tk):
        total, last, cur = 0.0, 0, tempos[0][1]
        for tt, us in tempos:
            if tt >= tk:
                break
            total += (tt - last) * cur / 1e6 / div
            last, cur = tt, us
        return total + (tk - last) * cur / 1e6 / div

    events.sort(key=lambda e: (e[0], e[1]))
    voice = channel if channel is not None else choose_melody_channel(events)
    if voice is not None:
        events = [e for e in events if e[3] == voice]
    sounding, schedule, current, opened = set(), [], None, False
    i, n = 0, len(events)
    while i < n:
        tick = events[i][0]
        t = to_seconds(tick)
        if t > end:
            break
        while i < n and events[i][0] == tick:
            _, on, pitch, _ch = events[i]
            if on:
                sounding.add(pitch)
            else:
                sounding.discard(pitch)
            i += 1
        top = max(sounding) if sounding else None
        if t < start:
            current = top
            continue
        if not opened:
            schedule.append((0.0, 0 if current is None else fit_note(current)))
            opened = True
        if top != current:
            current = top
            emit(schedule, t - start, 0 if top is None else fit_note(top))
    schedule.append((end - start, 0))
    return schedule, voice


def choose_melody_channel(events):
    """Pick the MIDI channel most likely to carry the melody (most monophonic, then busiest)."""
    onsets = {}
    for tick, on, _pitch, ch in events:
        if on:
            onsets.setdefault(ch, {}).setdefault(tick, 0)
            onsets[ch][tick] += 1
    best, best_score = None, None
    for ch, ticks in onsets.items():
        mono = sum(1 for n in ticks.values() if n == 1) / len(ticks)
        score = (mono, len(ticks))
        if best_score is None or score > best_score:
            best, best_score = ch, score
    return best


def silence(link, node):
    """Stop the note and put the output stage back in its safe state."""
    link.request(node, midi_frame(0))
    system_control(link, node, enable=SYS_DISABLE)


def cmd_music(link, args):
    """Play a window of a MIDI file through the motor coils."""
    end = args.end if args.end is not None else args.start + args.seconds
    if end <= args.start:
        print(f"--end ({end:g}s) must be after --start ({args.start:g}s)")
        return 1
    schedule, voice = parse_midi_melody(args.file, args.start, end, args.voice)
    notes = sum(1 for _, n in schedule if n)
    print(f"{os.path.basename(args.file)}: {notes} notes from {args.start:g}s to {end:g}s "
          f"(melody from channel {voice}), volume {args.volume}/255")
    t0 = time.time()
    try:
        for t, note in schedule:
            delay = t0 + t - time.time()
            if delay > 0:
                time.sleep(delay)
            link.request(args.node, midi_frame(note, args.volume), timeout=0.2)
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        silence(link, args.node)
    print("done")
    return 0


def cmd_music_stop(link, args):
    """Silence the motor and disable the output stage."""
    silence(link, args.node)
    print("stopped")
    return 0


def cmd_tone(link, args):
    """Play one note (MIDI number) for a duration, or 0 to stop."""
    reply = link.request(args.node, midi_frame(args.note, args.volume, args.duration_ms))
    return 0 if expect_ok(reply, "tone") else 1


# ============================================================================
# Entry point
# ============================================================================


def main():
    p = argparse.ArgumentParser(
        description="Host-side master for the TunePulse CAN Protocol v1.",
        epilog="Registers for get/set: " + ", ".join(sorted(REGISTERS)))
    p.add_argument("--interface", default="auto",
                   help="'auto' (default) uses the drive's HID control interface when present, otherwise "
                        "detects whether the channel is a TunePulse console or a CAN adapter; 'hid', "
                        "'serial', or a python-can interface name such as slcan, pcan or socketcan")
    p.add_argument("--channel", default=None,
                   help="adapter channel or serial device; omitted, the drive is auto-detected "
                        "(TPCP_CHANNEL overrides)")
    p.add_argument("--bitrate", type=int, default=500000, help="arbitration bit rate (default: 500000)")
    p.add_argument("--data-bitrate", type=int, default=2000000, help="CAN-FD data-phase bit rate")
    p.add_argument("--fd", action="store_true", help="use CAN-FD frames with bit-rate switching")
    p.add_argument("-v", "--verbose", action="store_true", help="dump every frame sent and received")
    p.add_argument("--log", action="store_true", help="with --interface serial, also print the console output")
    sub = p.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("scan", help="discover unaddressed nodes and assign addresses")
    s.add_argument("--first-id", type=int, default=1, help="first address to hand out")
    s.add_argument("--window", type=float, default=0.03, help="reply collection window per probe [s]")
    s.add_argument("--limit", type=int, default=32, help="stop after this many nodes")
    s.set_defaults(fn=cmd_scan)

    s = sub.add_parser("ping", help="inventory every addressed node")
    s.add_argument("--window", type=float, default=0.3)
    s.set_defaults(fn=cmd_ping)

    s = sub.add_parser("info", help="dump the interesting registers of a node")
    s.add_argument("node", type=int)
    s.set_defaults(fn=cmd_info)

    s = sub.add_parser("get", help="read one register ('group:record', or 'group:*' for a whole sub-space)")
    s.add_argument("node", type=int)
    s.add_argument("reg")
    s.add_argument("fields", nargs="*", help="keyed selector, e.g. function=3")
    s.set_defaults(fn=cmd_get)

    s = sub.add_parser("set", help="write fields of one register: field=value ...")
    s.add_argument("node", type=int)
    s.add_argument("reg")
    s.add_argument("fields", nargs="+")
    s.set_defaults(fn=cmd_set)

    s = sub.add_parser("enable", help="enable or disable the output stage")
    s.add_argument("node", type=int)
    s.add_argument("state", choices=["on", "off"])
    s.set_defaults(fn=cmd_enable)

    s = sub.add_parser("mode", help="select the control mode")
    s.add_argument("node", type=int)
    s.add_argument("mode", choices=MODES)
    s.add_argument("--enable", action="store_true", help="energize at the same time")
    s.set_defaults(fn=cmd_mode)

    s = sub.add_parser("move", help="position setpoint, or a queued S-curve with --scurve / --relative")
    s.add_argument("node", type=int)
    s.add_argument("revolutions", type=float)
    s.add_argument("--relative", action="store_true", help="queue an S-curve displacement")
    s.add_argument("--scurve", action="store_true", help="queue an absolute S-curve move")
    s.add_argument("--velocity", type=float, default=None, help="cruise velocity for the S-curve [RPS]")
    s.add_argument("--wait", action="store_true", help="block until the rotor arrives")
    s.add_argument("--tolerance", type=float, default=0.01)
    s.add_argument("--timeout", type=float, default=60.0)
    s.set_defaults(fn=cmd_move)

    s = sub.add_parser("jog", help="velocity setpoint")
    s.add_argument("node", type=int)
    s.add_argument("rps", type=float)
    s.set_defaults(fn=cmd_jog)

    s = sub.add_parser("torque", help="torque setpoint (N*m, or A while Kt is unknown)")
    s.add_argument("node", type=int)
    s.add_argument("value", type=float)
    s.set_defaults(fn=cmd_torque)

    s = sub.add_parser("stop", help="stop with a selectable end state")
    s.add_argument("node", type=int)
    s.add_argument("style", nargs="?", default="hold", choices=["hold", "freewheel", "brake", "reset"])
    s.set_defaults(fn=cmd_stop)

    s = sub.add_parser("pvt", help="stream a PVT burst into the trajectory queue")
    s.add_argument("node", type=int)
    s.add_argument("revolutions", type=float)
    s.add_argument("--seconds", type=float, default=2.0)
    s.add_argument("--back", action="store_true")
    s.add_argument("--segments", type=int, default=20)
    s.add_argument("--wait", action="store_true")
    s.add_argument("--tolerance", type=float, default=0.01)
    s.add_argument("--cycles", type=int, default=1)
    s.add_argument("--set-mode", action="store_true", help="enable the stage first")
    s.set_defaults(fn=cmd_pvt)

    s = sub.add_parser("queue", help="inspect or clear the trajectory queue")
    s.add_argument("node", type=int)
    s.add_argument("action", nargs="?", default="status", choices=["status", "clear"])
    s.set_defaults(fn=cmd_queue)

    s = sub.add_parser("telemetry", help="configure a cyclic telemetry slot")
    s.add_argument("node", type=int)
    s.add_argument("slot", type=int, choices=[0, 1, 2, 3])
    s.add_argument("chunk", help="register name, e.g. motion_status_2 / pulsebeat / currents:dq")
    s.add_argument("period", type=int, help="period in ms (0 disables the slot)")
    s.set_defaults(fn=cmd_telemetry)

    s = sub.add_parser("watchdog", help="configure the master-liveness watchdog")
    s.add_argument("node", type=int)
    s.add_argument("timeout", type=int, help="timeout in ms (0 disables it)")
    s.set_defaults(fn=cmd_watchdog)

    s = sub.add_parser("save", help="persist configuration (drive must be disabled)")
    s.add_argument("node", type=int)
    s.add_argument("target", nargs="?", default="config", choices=["all", "config", "motor"])
    s.set_defaults(fn=cmd_save)

    s = sub.add_parser("defaults", help="restore defaults into the live shadow (--erase wipes the stored record)")
    s.add_argument("node", type=int)
    s.add_argument("--erase", action="store_true")
    s.set_defaults(fn=cmd_defaults)

    s = sub.add_parser("calibrate", help="start a calibration routine and poll its progress")
    s.add_argument("node", type=int)
    s.add_argument("kind", choices=["offset", "electrical", "angle", "full"])
    s.set_defaults(fn=cmd_calibrate)

    s = sub.add_parser("reset", help="clear latched faults")
    s.add_argument("node", type=int)
    s.set_defaults(fn=cmd_reset)

    s = sub.add_parser("ripple", help="torque-ripple feed-forward map: learn (slow move), clear, or show (RAM only)")
    s.add_argument("node", type=int)
    s.add_argument("what", choices=["learn", "clear", "show"])
    s.add_argument("--rps", type=float, default=0.05, help="learning speed [RPS]; keep it far below the loop resonance")
    s.add_argument("--rev", type=float, default=1.0, help="learning distance [rev]; one revolution covers every pole pair")
    s.add_argument("--set-mode", action="store_true", help="enable the stage first")
    s.set_defaults(fn=cmd_ripple)

    s = sub.add_parser("reboot", help="reboot a node")
    s.add_argument("node", type=int)
    s.add_argument("--dfu", action="store_true", help="jump into the USB DFU bootloader")
    s.set_defaults(fn=cmd_reboot)

    s = sub.add_parser("estop", help="broadcast freewheel + disable to every node")
    s.set_defaults(fn=cmd_estop)

    s = sub.add_parser("sync", help="broadcast the host clock (PVT time base)")
    s.set_defaults(fn=cmd_sync)

    s = sub.add_parser("monitor", help="decode every protocol frame on the bus")
    s.add_argument("node", type=int, nargs="?", default=None)
    s.set_defaults(fn=cmd_monitor)

    s = sub.add_parser("trace", help="dump the black box (0x44, dev builds) around the last fault to CSV")
    s.add_argument("node", type=int)
    s.add_argument("--out", help="CSV file (default trace_node<N>.csv)")
    s.add_argument("--force", action="store_true", help="dump even when no fault froze the ring")
    s.set_defaults(fn=cmd_trace)

    s = sub.add_parser("raw", help="send an arbitrary command frame")
    s.add_argument("node", type=int)
    s.add_argument("hex", help="payload bytes, e.g. '50' or 'A0 E1'")
    s.set_defaults(fn=cmd_raw)

    s = sub.add_parser("tone", help="play one MIDI note through the coils")
    s.add_argument("node", type=int)
    s.add_argument("note", type=int, help="MIDI note number (69 = A4); 0 stops")
    s.add_argument("--volume", type=int, default=128)
    s.add_argument("--duration-ms", type=float, default=0.0)
    s.set_defaults(fn=cmd_tone)

    s = sub.add_parser("music", help="play the opening of a MIDI file through the motor coils")
    s.add_argument("node", type=int)
    s.add_argument("file")
    s.add_argument("--start", type=float, default=0.0)
    s.add_argument("--end", type=float, default=None)
    s.add_argument("--seconds", type=float, default=30.0)
    s.add_argument("--volume", type=int, default=128)
    s.add_argument("--voice", type=int, default=None, choices=range(16), metavar="0-15")
    s.set_defaults(fn=cmd_music)

    s = sub.add_parser("music-stop", help="silence the motor and disable the output stage")
    s.add_argument("node", type=int)
    s.set_defaults(fn=cmd_music_stop)

    args = p.parse_args()

    explicit = args.channel is not None
    if args.channel is None:
        args.channel = os.environ.get("TPCP_CHANNEL")
        explicit = args.channel is not None

    node, is_console = None, False
    if args.interface in ("auto", "hid"):
        hid_path = find_hid_path(args.channel if explicit else None)
        if hid_path is not None:
            if args.interface == "auto":
                print("detected: TunePulse HID control interface")
            args.interface, args.channel = "hid", hid_path
            link = HidLink(args)
            try:
                return args.fn(link, args)
            finally:
                link.close()
        if args.interface == "hid":
            sys.exit("no TunePulse HID interface found (hidapi installed? udev rule on Linux?)")

    if not explicit:
        cached = cache_read()
        if cached is not None:
            found, node = probe_drive(cached)
            if found:
                args.channel, is_console = cached, True
    if args.channel is None:
        args.channel, why = autodetect_channel()
        if args.channel is None:
            sys.exit(f"could not find a drive: {why}")
        print(f"note: {why}")
        cache_write(args.channel)
    args.channel, note = normalize_port(args.channel)
    if note:
        print(f"note: {note}")
    if not is_console:
        found, node = probe_drive(args.channel)
        is_console = found
        if not is_console:
            is_console, node, _ = sniff_port(args.channel)
    if args.interface == "auto":
        args.interface = "serial" if is_console else "slcan"
        if not is_console:
            print("detected: no TunePulse console on this port -> assuming a CAN adapter (slcan)")
    if args.interface == "serial" and is_console:
        cache_write(args.channel)

    target = getattr(args, "node", None)
    if (node == NODE_UNASSIGNED) and (target is not None) and (args.cmd not in ("scan", "monitor", "estop", "sync")):
        print(f"\nThe drive reports node=255 (UNASSIGNED). It answers discovery only, so every request to "
              f"node {target} will time out. Give it an address first:\n\n"
              f"    {sys.argv[0]} --channel {args.channel} scan\n")
        return 1

    link = SerialLink(args) if args.interface == "serial" else Link(args)
    try:
        return args.fn(link, args)
    finally:
        link.close()


if __name__ == "__main__":
    sys.exit(main())
