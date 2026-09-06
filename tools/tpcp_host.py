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

    # Simplest: let it find the drive's own USB console by itself
    ./tpcp_host.py info 1
    ./tpcp_host.py move 1 0.25 --relative

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

The HID transport is the fast one and is picked automatically when the board
exposes it: fixed-size reports need no framing, the endpoint paces the host
itself, and six frames ride in every 1 ms transfer, so a trajectory burst
streams at full speed instead of four frames at a time with a sleep between.

The serial transport exercises the whole protocol except the CAN physical
layer: framing over a byte stream replaces bus framing, so bit timing,
arbitration, hardware filtering, bus-off handling and the discovery sweep are
NOT covered by it. Everything above that - dispatch, registers, telemetry,
motion, calibration, persistence, the watchdog - behaves identically.

Bit rate must match the drive. A drive built with the cln17_v3_bench
environment comes up as node 1 on classic CAN at 500 kbit/s, which is the
default here. The shipping firmware comes up unaddressed on CAN-FD at
1 Mbit/s / 5 Mbit/s: use --fd --bitrate 1000000 --data-bitrate 5000000 and run
`scan` first to give it an address.

Copyright 2026, creapunk
Licensed under the Apache License, Version 2.0
"""

import argparse
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
# Identifier map (src/api/tp_wire.h)
# ============================================================================

ID_EMCY = 0x000       # + node   node -> bus   emergency / fault
ID_NMT = 0x100        #          master -> bus network management
ID_SETPOINT = 0x200   # + node   master -> node fire-and-forget setpoints
ID_REQUEST = 0x300    # + node   master -> node requests
ID_RESPONSE = 0x400   # + node   node -> master responses
ID_DISCOVERY = 0x500  # | byte   node -> master discovery arbitration
ID_TELEMETRY = 0x600  # + node   node -> bus   heartbeat and telemetry

NODE_BROADCAST = 0xFF
NODE_UNASSIGNED = 0x00
UID_BYTES = 7

NMT_DISCOVER_BEGIN = ID_NMT + 0x0
NMT_DISCOVER_PREFIX = ID_NMT + 0x1
NMT_DISCOVER_ASSIGN = ID_NMT + 0x2
NMT_DISCOVER_END = ID_NMT + 0x3
NMT_GLOBAL_CONTROL = ID_NMT + 0x4
NMT_SYNC = ID_NMT + 0x5

REBOOT_KEY = 0x544F4F42    # "BOOT"
DEFAULTS_KEY = 0x54455344  # "DSET"

# ============================================================================
# Opcodes
# ============================================================================

OP = {
    "device_info": 0x01, "device_uid": 0x02, "device_status": 0x03,
    "motor_status": 0x04, "electric_status": 0x05,
    "motion1": 0x06, "motion2": 0x07, "motion3": 0x08,
    "motor_props": 0x09, "encoder": 0x0A, "driver_status": 0x0B,
    "calib_status": 0x0C, "comm_status": 0x0D, "power_status": 0x0E,
    "heartbeat": 0x0F,
    "control_mode": 0x10, "target_position": 0x11, "target_velocity": 0x12,
    "target_torque": 0x13, "motion_limits": 0x14, "stop": 0x15, "anchor": 0x16,
    "scurve": 0x1A, "queue_ctl": 0x1B, "queue_stat": 0x1C,
    "reg_read": 0x20, "reg_write": 0x21, "reg_info": 0x22,
    "param_save": 0x28, "param_load": 0x29, "param_defaults": 0x2A,
    "param_status": 0x2B,
    "calib_start": 0x30, "calib_abort": 0x31, "calib_result": 0x32,
    "sys_enable": 0x38, "fault_reset": 0x39, "reboot": 0x3A,
    "node_id": 0x3B, "telemetry": 0x3C, "watchdog": 0x3D,
    "ack": 0x7E, "nack": 0x7F,
}
OP_NAME = {v: k for k, v in OP.items()}

ERRORS = {
    0x00: "none", 0x01: "unknown opcode", 0x02: "bad length",
    0x03: "bad parameter", 0x04: "read only", 0x05: "write only",
    0x06: "unknown register", 0x07: "out of range", 0x08: "wrong state",
    0x09: "busy", 0x0A: "queue full", 0x0B: "not implemented",
    0x0C: "access denied", 0x0D: "fault active", 0x0E: "nvm error",
    0x0F: "not calibrated",
}

STATES = ["boot", "calibrating", "idle", "ready", "running", "fault",
          "config", "rebooting"]
MODES = ["disabled", "torque", "velocity", "position", "motion", "tone"]
FAULT_BITS = ["overvoltage", "overcurrent", "overheat", "external",
              "undervoltage", "overpower", "phase", "bit7"]
WARN_BITS = ["not-calibrated", "flash-busy", "watchdog", "queue-low",
             "queue-drop", "comm", "temperature", "supply"]

# Register map (src/api/tp_registers.h). name -> (id, type)
REGS = {
    "node_id": (0x0000, "u8"), "arb_bitrate": (0x0001, "u32"),
    "data_bitrate": (0x0002, "u32"), "link_flags": (0x0003, "u8"),
    "heartbeat_ms": (0x0004, "u16"), "watchdog_ms": (0x0005, "u16"),
    "watchdog_action": (0x0006, "u8"), "safe_state": (0x0007, "u8"),
    "hw_version": (0x0008, "u8"), "fw_version": (0x0009, "u16"),
    "device_code": (0x000A, "u32"), "uptime_s": (0x000B, "u32"),
    "protocol_version": (0x000C, "u16"), "uid_low": (0x000D, "u32"),
    "uid_high": (0x000E, "u32"),
    "motor_type": (0x0100, "u8"), "pole_pairs": (0x0101, "u16"),
    "phase_pattern": (0x0102, "u8"), "direction": (0x0103, "i8"),
    "resistance": (0x0104, "f32"), "inductance_d": (0x0105, "f32"),
    "inductance_q": (0x0106, "f32"), "torque_constant": (0x0107, "f32"),
    "flux_linkage": (0x0108, "f32"), "cal_id": (0x0109, "u8"),
    "cal_offset": (0x010A, "i32"), "max_torque": (0x010B, "f32"),
    "current_limit": (0x0200, "f32"), "max_voltage": (0x0201, "f32"),
    "max_power": (0x0202, "f32"), "oc_limit_a": (0x0203, "f32"),
    "oc_limit_b": (0x0204, "f32"), "oc_limit_c": (0x0205, "f32"),
    "ov_limit": (0x0206, "f32"), "uv_limit": (0x0207, "f32"),
    "ot_limit": (0x0208, "f32"), "ot_hysteresis": (0x0209, "f32"),
    "op_limit_w": (0x020A, "f32"), "op_limit_a": (0x020B, "f32"),
    "temp_limit": (0x020C, "u8"), "calib_current": (0x020D, "f32"),
    "control_mode": (0x0300, "u8"), "enable": (0x0301, "u8"),
    "epsilon_rev": (0x0302, "f32"), "damping_gain": (0x0303, "f32"),
    "v_soft_rps": (0x0304, "f32"), "obs_fc_min": (0x0305, "f32"),
    "obs_fc_max": (0x0306, "f32"), "obs_fc_per_rev": (0x0307, "f32"),
    "obs_fc_slew": (0x0308, "f32"), "obs_acc_limit": (0x0309, "f32"),
    "vel_limit": (0x0400, "f32"), "accel_limit": (0x0401, "f32"),
    "jerk_limit": (0x0402, "f32"), "time_scale": (0x0403, "i16"),
    "queue_depth": (0x0404, "u8"), "target_position": (0x0405, "q16"),
    "target_velocity": (0x0406, "f32"), "target_current": (0x0407, "f32"),
    "position": (0x0500, "q16"), "velocity": (0x0501, "f32"),
    "applied_current": (0x0502, "f32"), "supply_voltage": (0x0503, "f32"),
    "power": (0x0504, "f32"), "temperature": (0x0505, "f32"),
    "fault_flags": (0x0506, "u8"), "encoder_angle": (0x0507, "u32"),
    "encoder_turns": (0x0508, "i32"), "control_ticks": (0x0509, "u32"),
    "bus_current": (0x050A, "f32"),
    "tone_freq": (0x050B, "u16"), "tone_amp": (0x050C, "u16"),
}
REG_BY_ID = {v[0]: (k, v[1]) for k, v in REGS.items()}
TYPE_NAMES = ["u8", "u16", "u32", "i8", "i16", "i32", "f32", "q16"]

# ============================================================================
# Codecs (mirror of the C helpers in tp_wire.h)
# ============================================================================


def bf16_to_float(raw):
    """bfloat16 (top 16 bits of an IEEE-754 float) -> float."""
    return struct.unpack("<f", struct.pack("<I", (raw & 0xFFFF) << 16))[0]


def float_to_bf16(value):
    """float -> bfloat16, truncating."""
    return (struct.unpack("<I", struct.pack("<f", value))[0] >> 16) & 0xFFFF


def f24_to_float(raw):
    """float24 (top 24 bits of an IEEE-754 float) -> float.

    A pure left shift. Sign-extending here would overwrite the exponent and
    turn every negative value into an infinity or a NaN - the exact bug the
    firmware's draft codec shipped with.
    """
    return struct.unpack("<f", struct.pack("<I", (raw & 0xFFFFFF) << 8))[0]


def float_to_f24(value):
    """float -> float24, truncating."""
    return (struct.unpack("<I", struct.pack("<f", value))[0] >> 8) & 0xFFFFFF


def signext(value, bits):
    """Sign-extend an unsigned field of the given width."""
    mask = 1 << (bits - 1)
    return (value ^ mask) - mask


def q16_to_rev(raw):
    """Signed Q16.16 -> revolutions."""
    return signext(raw & 0xFFFFFFFF, 32) / 65536.0


def rev_to_q16(rev):
    """Revolutions -> signed Q16.16."""
    return int(round(rev * 65536.0)) & 0xFFFFFFFF


def i48_to_rev(data):
    """Little-endian 48-bit Q16.16 field -> revolutions."""
    return signext(int.from_bytes(data[:6], "little"), 48) / 65536.0


def rev_to_i48(rev):
    """Revolutions -> little-endian 48-bit Q16.16 field."""
    return (int(round(rev * 65536.0)) & 0xFFFFFFFFFFFF).to_bytes(6, "little")


def i24_to_rev(data):
    """Little-endian 24-bit Q16.16 field -> revolutions."""
    return signext(int.from_bytes(data[:3], "little"), 24) / 65536.0


def decode_reg(rtype, raw):
    """Decode a 32-bit register payload according to its declared type."""
    raw &= 0xFFFFFFFF
    if rtype == "f32":
        return struct.unpack("<f", struct.pack("<I", raw))[0]
    if rtype == "q16":
        return q16_to_rev(raw)
    if rtype == "i8":
        return signext(raw & 0xFF, 8)
    if rtype == "i16":
        return signext(raw & 0xFFFF, 16)
    if rtype == "i32":
        return signext(raw, 32)
    if rtype == "u8":
        return raw & 0xFF
    if rtype == "u16":
        return raw & 0xFFFF
    return raw


def encode_reg(rtype, text):
    """Encode a command-line value into a 32-bit register payload."""
    if rtype == "f32":
        return struct.unpack("<I", struct.pack("<f", float(text)))[0]
    if rtype == "q16":
        return rev_to_q16(float(text))
    return int(text, 0) & 0xFFFFFFFF


def flag_names(value, names):
    """Render a bit field as a comma-separated list of set flag names."""
    out = [n for i, n in enumerate(names) if value & (1 << i)]
    return ",".join(out) if out else "none"

# ============================================================================
# Transport
# ============================================================================


class Link:
    """Thin wrapper over python-can with request/response correlation."""

    def __init__(self, args):
        kwargs = dict(interface=args.interface, channel=args.channel,
                      bitrate=args.bitrate)
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
        payload = bytes(data).ljust(8, b"\x00") if data else bytes(data)
        msg = can.Message(arbitration_id=can_id, data=payload,
                          is_extended_id=False,
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
        """Send a request and wait for the matching response frame.

        Telemetry and emergency frames arriving in the meantime are skipped
        rather than mistaken for the answer.
        """
        self.send(ID_REQUEST + node, payload)
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.recv(deadline - time.time())
            if msg is None:
                continue
            if msg.arbitration_id == ID_RESPONSE + node:
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

    Sends a broadcast Device Info request and waits briefly for a framed
    reply. This is far quicker than listening for the drive to emit a heartbeat
    on its own - a second of waiting versus a third of one - and the reply
    identifier reveals the node address for free, since responses come back on
    0x400 + node.

    Returns (is_drive, node_id_or_None). A drive with no address assigned does
    not answer requests at all, so a miss here falls back to passive sniffing.
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
        body = bytes([(ID_REQUEST + NODE_BROADCAST) & 0xFF,
                      ((ID_REQUEST + NODE_BROADCAST) >> 8) & 0x07,
                      OP["device_info"], 0, 0, 0, 0, 0, 0, 0])
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
                    if ID_RESPONSE <= cid < ID_DISCOVERY:
                        return True, cid - ID_RESPONSE
                del buf[:idx + 2]
                idx = buf.find(b"\xa5\x5a")
        return False, None
    finally:
        port.close()


def cache_path():
    """Where the last known-good port is remembered between invocations."""
    import os
    base = os.environ.get("XDG_CACHE_HOME") or os.path.expanduser("~/.cache")
    return os.path.join(base, "tpcp_host_port")


def cache_read():
    """Last port that answered, or None."""
    import os
    try:
        with open(cache_path()) as fh:
            dev = fh.read().strip()
        return dev if dev and os.path.exists(dev) else None
    except OSError:
        return None


def cache_write(channel):
    """Remember a port that answered, best effort."""
    import os
    try:
        os.makedirs(os.path.dirname(cache_path()), exist_ok=True)
        with open(cache_path(), "w") as fh:
            fh.write(channel)
    except OSError:
        pass


def autodetect_channel():
    """Find the drive's console without being told where it is.

    Looks at every USB serial device on the machine and returns the first that
    sounds like a TunePulse drive. Ports are sniffed passively - nothing is
    transmitted - so probing an unrelated adapter cannot upset it.

    Returns (channel, note) or (None, reason).
    """
    import glob
    import os

    candidates = sorted(glob.glob("/dev/cu.usbmodem*") +      # macOS CDC
                        glob.glob("/dev/ttyACM*") +           # Linux CDC
                        glob.glob("/dev/cu.usbserial*"))      # USB-UART bridges
    if not candidates:
        return None, "no USB serial devices found"

    consoles = []
    for dev in candidates:
        # Cheap active probe first; only fall back to listening when the drive
        # is unaddressed and therefore answers nothing.
        found, _ = probe_drive(dev)
        if not found:
            found, _, _ = sniff_port(dev, 1.2)
        if found:
            consoles.append(dev)
    if len(consoles) == 1:
        return consoles[0], f"auto-detected drive console on {consoles[0]}"
    if len(consoles) > 1:
        return None, ("several drive consoles found (" + ", ".join(consoles) +
                      ") - name one with --channel")
    return None, ("no drive console found among " + ", ".join(candidates) +
                  " (an ST-Link's own VCP is not the drive: the drive's console is on "
                  "the board's own USB port)")


def normalize_port(channel):
    """Prefer the callout device on macOS.

    /dev/tty.* is the dial-in device: opening it can block waiting for carrier
    detect, which on a USB CDC port looks like a hang. /dev/cu.* is the callout
    device and is what you want for any host-initiated conversation.
    """
    if channel.startswith("/dev/tty.") :
        alt = channel.replace("/dev/tty.", "/dev/cu.", 1)
        import os
        if os.path.exists(alt):
            return alt, f"using {alt} instead of {channel} (callout device)"
    return channel, None


def sniff_port(channel, seconds=1.5):
    """Listen to a serial port and work out what is on the other end.

    Returns (is_tunepulse_console, node_id_or_None, sample_text).

    A TunePulse drive emits ASCII heartbeat lines unprompted; an slcan CAN
    adapter says nothing until it is asked. That difference is enough to tell
    them apart without sending anything, which matters because sending slcan
    commands to a drive console is exactly the mistake this exists to catch.
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
            # Stop the moment the question is answered rather than always
            # burning the whole window: a heartbeat lands within a second, and
            # waiting past it is pure latency on every single command.
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


# Frames per write, and the pause between writes, when streaming a burst.
TP_SERIAL_BATCH = 4
TP_SERIAL_BATCH_DELAY = 0.002


class SerialFrame:
    """Minimal stand-in for can.Message so both links share the decoders."""

    def __init__(self, arbitration_id, data):
        self.arbitration_id = arbitration_id
        self.data = data


class SerialLink:
    """Framed protocol transport over the board's USB CDC console.

    Speaks the framing in src/api/transport/tp_serial.h:

        A5 5A | id_lo id_hi | d0..d7 | xor

    Anything outside a frame is diagnostic log text. Because the sync byte has
    bit 7 set and the logs are ASCII, the two can never be confused, so this
    prints the log through as it goes - one cable carries both the console and
    the protocol.
    """

    SYNC = b"\xa5\x5a"
    FRAME_LEN = 13

    def __init__(self, args):
        try:
            import serial
        except ImportError:
            sys.exit("pyserial is required for --interface serial:  pip install pyserial")
        self.ser = serial.Serial(args.channel, baudrate=115200, timeout=0.02)
        self.buf = bytearray()      # Unparsed bytes from the port
        self.text = bytearray()     # Log bytes seen between frames
        self.verbose = args.verbose
        self.show_log = args.log

    def close(self):
        self.ser.close()

    def send(self, can_id, data=b"", fd=None):
        """Frame and transmit one protocol frame."""
        payload = bytes(data).ljust(8, b"\x00")[:8]
        body = bytes([can_id & 0xFF, (can_id >> 8) & 0x07]) + payload
        chk = 0
        for b in body:
            chk ^= b
        if self.verbose:
            print(f"  TX {can_id:03X}  {payload.hex(' ')}")
        self.ser.write(self.SYNC + body + bytes([chk]))

    def send_many(self, can_id, payloads):
        """Frame a whole burst and hand it to the port in ONE write.

        Streaming segment by segment with a sleep between each costs roughly ten
        milliseconds per frame once Python's timer granularity and USB CDC
        latency are counted. That is slower than the drive consumes fine
        segments, so the trajectory queue starves mid-move and the leg comes up
        short. One write for the whole burst removes the per-frame cost.
        """
        blob = bytearray()
        for payload in payloads:
            body = bytes([can_id & 0xFF, (can_id >> 8) & 0x07]) + bytes(payload).ljust(8, b"\x00")[:8]
            chk = 0
            for b in body:
                chk ^= b
            blob += self.SYNC + body + bytes([chk])
        if self.verbose:
            print(f"  TX {can_id:03X}  x{len(payloads)} frames ({len(blob)} bytes)")
        # Written in small batches, NOT all at once. The two failure modes sit
        # on either side of this: one frame at a time is slower than the drive
        # consumes fine segments, so the trajectory queue starves; the whole
        # burst in one write overruns the board's USB CDC receive buffer and
        # frames are lost silently. A few frames per write clears both.
        step = TP_SERIAL_BATCH * self.FRAME_LEN
        for off in range(0, len(blob), step):
            self.ser.write(bytes(blob[off:off + step]))
            self.ser.flush()
            time.sleep(TP_SERIAL_BATCH_DELAY)

    def _flush_text(self):
        """Print complete log lines collected between protocol frames."""
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
                    # No frame start in sight: everything but a trailing byte
                    # (which could be half a sync pair) is log text.
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
                    break                       # Frame started but is not complete yet
                body = bytes(self.buf[2:12])
                chk = 0
                for b in body:
                    chk ^= b
                if chk != self.buf[12]:
                    # False sync inside log text: drop one byte and rescan.
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
        """Send a request and wait for the matching response frame."""
        self.send(ID_REQUEST + node, payload)
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
    """Return the OS path of the drive's vendor HID interface, or None.

    A composite device is enumerated once per interface, so the CDC console and
    the control pipe appear as separate entries. The control pipe is interface
    2; on platforms that do not report interface numbers the vendor usage page
    identifies it just as well.
    """
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
    """Protocol transport over the drive's vendor HID interface.

    Speaks the report layout in src/api/transport/tp_hid.h:

        byte 0   record count, 0..6
        byte 1+  records of ten bytes: id_lo, id_hi | len<<4, then eight of payload

    This is the fast path. Unlike the CDC transport there is no framing to get
    wrong and no guessing about how much the board can absorb: a report arrives
    whole or not at all, and the endpoint NAKs while the drive is busy, so the
    host is paced by the device instead of by a sleep. That is what lets
    send_many() hand over a whole trajectory burst at full speed.
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
        self.pending = []       # Frames unpacked from a report but not yet returned
        self.verbose = args.verbose

    def close(self):
        self.dev.close()

    @staticmethod
    def _record(can_id, data):
        """Pack one protocol frame into its ten-byte record."""
        payload = bytes(data).ljust(8, b"\x00")[:8]
        # The length nibble is always eight, exactly as the CDC transport
        # reports it: the engine treats a full payload as the normal case and
        # every chunk codec reads only the bytes its opcode defines.
        return bytes([can_id & 0xFF, ((can_id >> 8) & 0x07) | (8 << 4)]) + payload

    def _send_records(self, records):
        """Pack records into reports, six at a time, and write them."""
        for off in range(0, len(records), TP_HID_REC_PER_REPORT):
            batch = records[off:off + TP_HID_REC_PER_REPORT]
            report = bytearray(TP_HID_REPORT_LEN)
            report[0] = len(batch)
            for i, rec in enumerate(batch):
                report[1 + i * TP_HID_REC_LEN:1 + (i + 1) * TP_HID_REC_LEN] = rec
            # hidapi expects the report ID as the first byte. This device
            # declares no report IDs, so it is zero and the report follows.
            self.dev.write(b"\x00" + bytes(report))

    def send(self, can_id, data=b"", fd=None):
        """Transmit one protocol frame."""
        if self.verbose:
            payload = bytes(data).ljust(8, b"\x00")[:8]
            print(f"  TX {can_id:03X}  {payload.hex(' ')}")
        self._send_records([self._record(can_id, data)])

    def send_many(self, can_id, payloads):
        """Transmit a burst.

        No batch size and no sleep between writes, unlike the CDC transport:
        six frames share each report and the interrupt endpoint's flow control
        stops the host when the drive is behind, so there is nothing to overrun
        and nothing to starve.
        """
        if self.verbose:
            print(f"  TX {can_id:03X}  x{len(payloads)} frames")
        self._send_records([self._record(can_id, p) for p in payloads])

    def recv(self, timeout=0.2):
        """Return the next protocol frame, or None on timeout."""
        if self.pending:
            return self.pending.pop(0)
        deadline = time.time() + max(timeout, 0.0)
        while True:
            left_ms = int(max(deadline - time.time(), 0.0) * 1000)
            data = self.dev.read(TP_HID_REPORT_LEN, timeout_ms=left_ms)
            if data:
                count = min(data[0], TP_HID_REC_PER_REPORT)
                for i in range(count):
                    rec = data[1 + i * TP_HID_REC_LEN:1 + (i + 1) * TP_HID_REC_LEN]
                    if len(rec) < TP_HID_REC_LEN:
                        break                      # Truncated report: ignore the tail
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
        """Send a request and wait for the matching response frame."""
        self.send(ID_REQUEST + node, payload)
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
# Chunk decoding
# ============================================================================


def decode_chunk(d):
    """Render one 8-byte protocol frame as human-readable text."""
    op = d[0]
    name = OP_NAME.get(op, f"0x{op:02X}")

    if op & 0x80:  # Bit-packed motion burst: no opcode byte at all
        disp = signext(((d[0] & 0x3F) << 16) | (d[1] << 8) | d[2], 22) / 65536.0
        vel = f24_to_float((d[3] << 16) | (d[4] << 8) | d[5])
        tail = (d[6] << 8) | d[7]
        kind = "PVcTF force" if d[0] & 0x40 else "PVT dur"
        return f"burst   disp={disp:+.4f}rev vel={vel:+.3f}RPS {kind}={tail}"

    if op == OP["device_info"]:
        hw, fw, dev = d[1], struct.unpack("<H", d[2:4])[0], struct.unpack("<I", d[4:8])[0]
        return f"info    hw=0x{hw:02X} fw={fw >> 8}.{fw & 0xFF} device=0x{dev:08X}"
    if op == OP["device_uid"]:
        return f"uid     {d[1:8].hex()}"
    if op == OP["device_status"]:
        up = int.from_bytes(d[1:4], "little")
        st = struct.unpack("<I", d[4:8])[0]
        return (f"status  up={up}s fault=[{flag_names(st & 0xFF, FAULT_BITS)}] "
                f"warn=[{flag_names((st >> 8) & 0xFF, WARN_BITS)}] "
                f"state={STATES[(st >> 16) & 0xF]} mode={MODES[min((st >> 20) & 0xF, 5)]}")
    if op == OP["motor_status"]:
        types = {0: "none", 1: "stepper", 2: "bldc", 3: "error"}
        phases = {0: "none", 1: "ABC", 2: "CAB", 3: "ACB"}
        fl = struct.unpack("<I", d[4:8])[0]
        return (f"motor   mode={MODES[min(d[1] & 0xF, 5)]} type={types[(d[1] >> 4) & 3]} "
                f"phase={phases[(d[1] >> 6) & 3]} poles={d[2]} temp={d[3]}C "
                f"dir={'CCW' if fl & 1 else 'CW'} en={bool(fl & 2)} cal={bool(fl & 4)}")
    if op == OP["electric_status"]:
        eff = "n/a" if d[1] == 255 else f"{d[1] * 100 // 254}%"
        return (f"elec    eff={eff} P={bf16_to_float(struct.unpack('<H', d[2:4])[0]):.2f}W "
                f"Vbus={bf16_to_float(struct.unpack('<H', d[4:6])[0]):.2f}V "
                f"Iq={bf16_to_float(struct.unpack('<H', d[6:8])[0]):+.3f}A")
    if op == OP["motion1"]:
        return f"motion1 mode={d[1]} pos={i48_to_rev(d[2:8]):+.5f}rev"
    if op == OP["motion2"]:
        vel = f24_to_float(d[1] | (d[2] << 8) | (d[3] << 16))
        return (f"motion2 vel={vel:+.4f}RPS "
                f"acc={bf16_to_float(struct.unpack('<H', d[4:6])[0]):+.2f}RPS2 "
                f"Iq={bf16_to_float(struct.unpack('<H', d[6:8])[0]):+.3f}A")
    if op == OP["motion3"]:
        qs = struct.unpack("<H", d[6:8])[0]
        ts = signext(qs >> 4, 12) / 256.0
        return (f"motion3 perr={i24_to_rev(d[1:4]):+.5f}rev "
                f"verr={bf16_to_float(struct.unpack('<H', d[4:6])[0]):+.3f}RPS "
                f"queue={qs & 0xF} scale={ts:.3f}x")
    if op == OP["motor_props"]:
        return (f"props   Tlim={d[1]}C R={bf16_to_float(struct.unpack('<H', d[2:4])[0]):.4f}ohm "
                f"L={bf16_to_float(struct.unpack('<H', d[4:6])[0]) * 1e6:.1f}uH "
                f"Kt={bf16_to_float(struct.unpack('<H', d[6:8])[0]):.5f}Nm/A")
    if op == OP["encoder"]:
        return (f"encoder health=0x{d[1]:02X} angle={struct.unpack('<H', d[2:4])[0]} "
                f"turns={struct.unpack('<i', d[4:8])[0]}")
    if op == OP["driver_status"]:
        return (f"driver  state={STATES[d[1] & 0xF]} mode={MODES[min(d[1] >> 4, 4)]} "
                f"fault=[{flag_names(d[2], FAULT_BITS)}] warn=[{flag_names(d[3], WARN_BITS)}] "
                f"op={d[4]} motor={d[5]}")
    if op == OP["calib_status"]:
        return (f"calib   kind={d[1]} stage={d[2]} progress={d[3] * 100 // 255}% "
                f"flags=0x{d[4]:02X} cal_id={d[5]} poles={struct.unpack('<H', d[6:8])[0]}")
    if op == OP["comm_status"]:
        return (f"comm    bus=0x{d[1]:02X} tec={d[2]} rec={d[3]} "
                f"dropped={struct.unpack('<H', d[4:6])[0]} nack={struct.unpack('<H', d[6:8])[0]}")
    if op == OP["power_status"]:
        return (f"power   load={struct.unpack('b', d[1:2])[0]} "
                f"Ibus={bf16_to_float(struct.unpack('<H', d[2:4])[0]):.3f}A "
                f"Pbus={bf16_to_float(struct.unpack('<H', d[4:6])[0]):.2f}W headroom={d[6]}C")
    if op == OP["heartbeat"]:
        return (f"HB      state={STATES[d[1] & 0xF]} mode={MODES[min(d[1] >> 4, 4)]} "
                f"fault=[{flag_names(d[2], FAULT_BITS)}] warn=[{flag_names(d[3], WARN_BITS)}] "
                f"{d[4] * 0.25:.2f}V {struct.unpack('b', d[5:6])[0]}C "
                f"load={struct.unpack('b', d[6:7])[0]} seq={d[7] & 0xF} q={d[7] >> 4}")
    if op == OP["queue_stat"]:
        return (f"queue   state=0x{d[1]:02X} queued={d[2]} free={d[3]} "
                f"scale={struct.unpack('<h', d[4:6])[0] / 1024:.3f}x "
                f"dropped={struct.unpack('<H', d[6:8])[0]}")
    if op == OP["param_status"]:
        return (f"param   state=0x{d[1]:02X} cfg_ver={d[2]} cal_id={d[3]} "
                f"cfg_free={struct.unpack('<H', d[4:6])[0]}B")
    if op == OP["calib_result"]:
        return (f"result  flags=0x{d[1]:02X} R={bf16_to_float(struct.unpack('<H', d[2:4])[0]):.4f}ohm "
                f"Ld={bf16_to_float(struct.unpack('<H', d[4:6])[0]) * 1e6:.1f}uH "
                f"Lq={bf16_to_float(struct.unpack('<H', d[6:8])[0]) * 1e6:.1f}uH")
    if op in (OP["reg_read"], OP["reg_write"], OP["reg_info"]):
        rid = struct.unpack("<H", d[2:4])[0]
        rname, _ = REG_BY_ID.get(rid, (f"0x{rid:04X}", "u32"))
        rtype = TYPE_NAMES[d[1]] if d[1] < len(TYPE_NAMES) else "u32"
        val = decode_reg(rtype, struct.unpack("<I", d[4:8])[0])
        shown = f"{val:.6g}" if isinstance(val, float) else val
        return f"{name:7s} {rname}({rtype}) = {shown}"
    if op == OP["node_id"]:
        return f"nodeid  id={d[1]} flags=0x{d[2]:02X} state=0x{d[3]:02X} guard=0x{struct.unpack('<I', d[4:8])[0]:08X}"
    if op == OP["ack"]:
        return f"ACK     {OP_NAME.get(d[1], hex(d[1]))} detail={d[3]}"
    if op == OP["nack"]:
        return (f"NACK    {OP_NAME.get(d[1], hex(d[1]))}: "
                f"{ERRORS.get(d[2], hex(d[2]))} (detail={d[3]})")
    return f"{name:7s} {d.hex(' ')}"


def expect_ok(reply, what):
    """Print the outcome of a command that answers with ACK or NACK."""
    if reply is None:
        print(f"{what}: NO RESPONSE")
        return False
    print(f"{what}: {decode_chunk(reply)}")
    return reply[0] != OP["nack"]

# ============================================================================
# Commands
# ============================================================================


def cmd_scan(link, args):
    """Run a UID discovery sweep and hand out node addresses."""
    print("discovery: entering addressing mode")
    link.send(NMT_DISCOVER_BEGIN, b"\x00" * 8)
    time.sleep(0.05)

    next_id = args.first_id
    found = []
    while next_id <= 254:
        uid = bytearray()
        for step in range(UID_BYTES):
            # Broadcast the prefix resolved so far. Every node whose UID starts
            # with it answers with a data-less frame whose identifier carries
            # its next UID byte; CAN arbitration delivers the lowest one.
            link.send(NMT_DISCOVER_PREFIX, bytes([step]) + bytes(uid) + b"\x00" * (7 - step))
            replies = [m.arbitration_id & 0xFF for m in link.drain(args.window)
                       if (m.arbitration_id & 0x700) == ID_DISCOVERY]
            if not replies:
                break
            uid.append(min(replies))
        if len(uid) != UID_BYTES:
            break

        print(f"discovery: found UID {bytes(uid).hex()} -> assigning node {next_id}")
        link.send(NMT_DISCOVER_ASSIGN, bytes(uid) + bytes([next_id]))
        deadline = time.time() + 1.0
        acked = False
        while time.time() < deadline:
            msg = link.recv(deadline - time.time())
            if msg and msg.arbitration_id == ID_RESPONSE + next_id:
                print(f"           {decode_chunk(bytes(msg.data).ljust(8, b'0'))}")
                acked = True
                break
        if not acked:
            print("           WARNING: no acknowledgement - address may not have been adopted")
        found.append((bytes(uid), next_id))
        next_id += 1

    link.send(NMT_DISCOVER_END, b"\x00" * 8)
    print(f"discovery: complete, {len(found)} node(s)")
    if found and args.save:
        for _, nid in found:
            expect_ok(link.request(nid, bytes([OP["param_save"], 1, 0, 0, 0, 0, 0, 0])),
                      f"node {nid} save")
    return 0


def cmd_info(link, args):
    """Read the full status set of one node."""
    for key in ("device_info", "device_uid", "device_status", "motor_status",
                "electric_status", "motion1", "motion3", "motor_props",
                "encoder", "driver_status", "calib_status", "comm_status",
                "power_status", "queue_stat", "param_status", "calib_result"):
        reply = link.request(args.node, bytes([OP[key], 0, 0, 0, 0, 0, 0, 0]))
        print(f"  {decode_chunk(reply) if reply else key + ': NO RESPONSE'}")
    return 0


def cmd_get(link, args):
    """Read one register."""
    if args.reg not in REGS:
        sys.exit(f"unknown register '{args.reg}'. Try: {', '.join(sorted(REGS))}")
    rid, _ = REGS[args.reg]
    payload = bytes([OP["reg_read"], 0]) + struct.pack("<H", rid) + b"\x00" * 4
    reply = link.request(args.node, payload)
    print(f"  {decode_chunk(reply) if reply else 'NO RESPONSE'}")
    return 0


def cmd_set(link, args):
    """Write one register and show the value actually in force."""
    if args.reg not in REGS:
        sys.exit(f"unknown register '{args.reg}'. Try: {', '.join(sorted(REGS))}")
    rid, rtype = REGS[args.reg]
    raw = encode_reg(rtype, args.value)
    payload = bytes([OP["reg_write"], 0]) + struct.pack("<HI", rid, raw)
    reply = link.request(args.node, payload)
    print(f"  {decode_chunk(reply) if reply else 'NO RESPONSE'}")
    return 0


def cmd_enable(link, args):
    """Enable or disable the output stage."""
    on = 1 if args.state == "on" else 0
    return 0 if expect_ok(link.request(args.node, bytes([OP["sys_enable"], on, 0, 0, 0, 0, 0, 0])),
                          "enable") else 1


def cmd_mode(link, args):
    """Select the control mode, optionally energizing at the same time."""
    mode = MODES.index(args.mode)
    payload = bytes([OP["control_mode"], mode, 1 if args.enable else 0, 0, 0, 0, 0, 0])
    return 0 if expect_ok(link.request(args.node, payload), "mode") else 1


def wait_settled(link, node, tol_rev=0.01, timeout=60.0):
    """Poll until the drive has actually arrived, then report where it landed.

    A setpoint command returns as soon as the drive ACCEPTS the target, not
    when the rotor gets there - that is what makes streaming setpoints possible.
    Anything that needs one move to finish before the next begins has to wait
    for it, and the drive already publishes everything needed: Motion Status 3
    carries the following error and the velocity error.
    """
    start = time.time()
    last = None
    last_queued = 0
    settled_polls = 0
    while time.time() - start < timeout:
        reply = link.request(node, bytes([OP["motion3"], 0, 0, 0, 0, 0, 0, 0]), timeout=0.3)
        if reply is None:
            time.sleep(0.05)
            continue
        perr = i24_to_rev(reply[1:4])
        queued = struct.unpack("<H", reply[6:8])[0] & 0x0F
        last = perr
        # Arrival is judged on the following error alone, held over several
        # consecutive polls. The velocity error would be the obvious second
        # condition, but it is frozen whenever the position loop is not
        # running, so keying on it would hang against a drive that has already
        # stopped - and against any firmware predating that fix.
        if abs(perr) <= tol_rev and queued == 0:
            settled_polls += 1
            if settled_polls >= 3:
                print(f"  settled after {time.time() - start:.2f}s (perr={perr:+.5f}rev)")
                return True
        else:
            settled_polls = 0
        last_queued = queued
        time.sleep(0.02)

    # The two ways this fails look nothing alike, and saying the wrong one
    # sends you chasing a current limit that was never the problem.
    if last is not None and last_queued and abs(last) <= tol_rev:
        print(f"  QUEUE NOT DRAINING: {last_queued} segment(s) still waiting, but the rotor is "
              f"on target (perr={last:+.5f}rev).")
        print("  The trajectory queue only advances in MOTION mode. Check with:")
        print("      tpcp_host.py get <node> control_mode      (4 = motion)")
        print("      tpcp_host.py get <node> enable            (1 = output stage armed)")
        print("  Clear the stale segments with:  tpcp_host.py queue <node> clear")
    elif last is not None:
        print(f"  DID NOT SETTLE in {timeout:.0f}s: perr={last:+.5f}rev, {last_queued} queued")
        print("  the rotor is not reaching the target - most likely the current limit is too low "
              "for the load, or the move is too fast for the available torque")
    return False


def cmd_move(link, args):
    """Command a position target."""
    flags = 0x01 if args.relative else 0x00
    if args.set_mode:
        flags |= 0x02
    payload = bytes([OP["target_position"], flags]) + rev_to_i48(args.revolutions)
    if not expect_ok(link.request(args.node, payload), "move"):
        return 1
    if args.wait:
        return 0 if wait_settled(link, args.node, args.tolerance, args.timeout) else 1
    return 0


def cmd_jog(link, args):
    """Command a velocity target."""
    v = float_to_f24(args.rps)
    payload = (bytes([OP["target_velocity"], v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF]) +
               struct.pack("<H", 0) + bytes([0x02 if args.set_mode else 0x00, 0]))
    return 0 if expect_ok(link.request(args.node, payload), "jog") else 1


def cmd_stop(link, args):
    """Stop with a selectable end state."""
    style = {"hold": 0, "freewheel": 1, "brake": 2, "quick": 3}[args.style]
    return 0 if expect_ok(link.request(args.node, bytes([OP["stop"], style, 0, 0, 0, 0, 0, 0])),
                          "stop") else 1


# Mirrors DRIVE_MOTION_QUEUE_DEPTH in src/lib/motor_control/drive_control.h.
# Only used to size the first burst; every burst after that is sized by the
# free-slot count the drive itself reports.
MOTION_QUEUE_DEPTH = 32


def query_queue(link, node, timeout=0.3):
    """Read the executor queue state: (queued, free_slots, dropped), or None."""
    reply = link.request(node, bytes([OP["queue_stat"], 0, 0, 0, 0, 0, 0, 0]), timeout=timeout)
    if reply is None or reply[0] != OP["queue_stat"]:
        return None
    return reply[2], reply[3], struct.unpack("<H", reply[6:8])[0]


def stream_pvt_leg(link, node, revolutions, seconds, segments):
    """Stream one leg of a move into the drive's trajectory queue.

    The leg is cut into equal segments, each a bit-packed PVT frame carrying a
    displacement, an end velocity and a duration. They go out on the setpoint
    channel, which is fire-and-forget: the drive queues them and interpolates
    between them, so the shape of the motion is set by the numbers here rather
    than by how fast this loop happens to run.

    Delivery is paced by the drive's queue, not by the transport. The queue
    holds MOTION_QUEUE_DEPTH segments and silently rejects anything that does
    not fit, so a leg longer than that has to be fed as the executor consumes
    it. This used to be hidden by how slow the CDC transport was - streaming
    four frames per write with a sleep between batches took long enough that
    the queue drained as it filled - but HID delivers a whole burst in under a
    millisecond, which overruns the queue and silently truncates the move.

    Polling for free slots is safe against the frames still being in flight:
    both transports deliver in order and the engine dispatches frames in
    arrival order, so a status reply necessarily reflects every setpoint sent
    before the request that produced it.
    """
    per = revolutions / segments                            # Displacement per segment [rev]
    dur = int(round(seconds / segments * 10000))            # Segment duration [100 us units]
    vel = revolutions / seconds                             # Cruise velocity [RPS]
    print(f"  leg {revolutions:+.3f}rev over {seconds:.2f}s ({vel:+.3f} RPS), "
          f"{segments} x {dur / 10.0:.1f}ms segments")
    d = int(round(per * 65536.0)) & 0x3FFFFF                # Q16.16 displacement, 22 bits
    v = float_to_f24(vel)                                   # float24 end velocity
    frame = bytes([0x80 | ((d >> 16) & 0x3F), (d >> 8) & 0xFF, d & 0xFF,
                   (v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF,
                   (dur >> 8) & 0xFF, dur & 0xFF])

    seg_period = seconds / segments                         # How fast one slot frees up [s]
    sent = 0                                                # Segments the drive has accepted
    dropped0 = None                                         # Drive's reject counter when we started
    # Generous: the whole leg plus room for a drive that is slow to start.
    deadline = time.time() + seconds + 30.0

    while sent < segments:
        stat = query_queue(link, node)
        if stat is None:
            # No status to pace against. Send what is left in one burst, which
            # is what the tool always did, and say so - on a fast transport
            # this is exactly the case that truncates a long leg.
            print("  note: no queue status; streaming without flow control")
            link.send_many(ID_SETPOINT + node, [frame] * (segments - sent))
            return
        _, free, dropped = stat
        if dropped0 is None:
            dropped0 = dropped
        elif dropped > dropped0:
            # The drive rejected something we sent, which means the free-slot
            # count it reported was wrong. Pacing cannot recover from that, and
            # silently losing segments truncates the move - so say so loudly
            # rather than letting the rotor come up short with no explanation.
            print(f"  WARNING: drive rejected {dropped - dropped0} segment(s) as queue-full "
                  f"despite reporting free slots. The move will be short.")
            print(f"           Firmware older than the free_slots fix over-reports by one; "
                  f"reflash, or reduce --segments below {MOTION_QUEUE_DEPTH}.")
            dropped0 = dropped
        room = min(free, segments - sent)                   # Never offer more than fits
        if room > 0:
            link.send_many(ID_SETPOINT + node, [frame] * room)
            sent += room
        else:
            # Queue full: wait about one segment's worth of playback for a slot
            # rather than spinning on the status request.
            time.sleep(min(seg_period, 0.05))
        if time.time() > deadline:
            print(f"  WARNING: queue never drained, {segments - sent} of {segments} segments "
                  f"not sent. Is the drive in motion mode and enabled?")
            return


def read_reg(link, node, name):
    """Read one register by name, or None when the drive does not answer."""
    rid, rtype = REGS[name]
    payload = bytes([OP["reg_read"], 0]) + struct.pack("<H", rid) + b"\x00" * 4
    reply = link.request(node, payload, timeout=0.4)
    if reply is None or reply[0] == OP["nack"]:
        return None
    return decode_reg(rtype, struct.unpack("<I", reply[4:8])[0])


# ============================================================================
# Motor speaker (DriveMode::Tone)
# ============================================================================
#
# The drive can drive its coils as a loudspeaker: in tone mode the control ISR
# synthesizes a sine at TP_REG_TONE_FREQ straight onto the alpha-beta vector,
# bypassing the current loop. A motor is monophonic, so a piano arrangement has
# to be reduced to one voice - the highest note sounding at each instant, which
# is where the melody lives in nearly all keyboard writing.

TONE_MIN_HZ = 80        # Below this a winding is close to a short and you feel it, not hear it
TONE_MAX_HZ = 2000      # Above this the inductance swallows the note
MIN_EVENT_S = 0.02      # Changes closer together than this are not separately audible


def emit(schedule, t, freq):
    """Append a schedule entry, collapsing anything too brief to be heard.

    Every entry costs a register write, and a change lasting a couple of
    milliseconds is a click rather than a note, so an entry that lands on top of
    the previous one replaces it instead of queueing another write.
    """
    if schedule and (t - schedule[-1][0]) < MIN_EVENT_S:
        schedule[-1] = (schedule[-1][0], freq)
        return
    schedule.append((t, freq))


def _vlq(b, i):
    """Read one MIDI variable-length quantity. Returns (value, next index)."""
    v = 0
    while True:
        c = b[i]
        i += 1
        v = (v << 7) | (c & 0x7F)
        if not c & 0x80:
            return v, i


def parse_midi_melody(path, start=0.0, end=30.0, channel=None):
    """Reduce a standard MIDI file to a monophonic schedule of (time, freq).

    Plays the window [start, end) of the piece. Returns a list of
    (seconds_from_the_START_OF_THE_WINDOW, frequency_hz) where 0 Hz is a rest,
    so playback always begins immediately regardless of where the window sits.

    Events before the window are still tracked rather than skipped: a note
    struck before `start` and still held at `start` is part of what the piece
    sounds like there, so it is emitted at t=0 instead of being lost.

    Only note on/off and tempo are interpreted - everything else is skipped,
    which is enough for any type 0 or 1 file.
    """
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
        # NB: named track_end, not end - `end` is the caller's window bound.
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
            if st == 0xFF:                                  # Meta event
                mt = data[i]
                i += 1
                l, i = _vlq(data, i)
                if mt == 0x51:                              # Tempo, microseconds per quarter
                    tempos.append((tick, int.from_bytes(data[i:i + l], "big")))
                i += l
            elif st in (0xF0, 0xF7):                        # SysEx: skip
                l, i = _vlq(data, i)
                i += l
            else:
                hi = st & 0xF0
                n = 2 if hi in (0x80, 0x90, 0xA0, 0xB0, 0xE0) else 1
                d1 = data[i]
                d2 = data[i + 1] if n == 2 else 0
                i += n
                if hi == 0x90 and d2 > 0:
                    events.append((tick, 1, d1, st & 0x0F))  # Note on
                elif hi == 0x80 or (hi == 0x90 and d2 == 0):
                    events.append((tick, 0, d1, st & 0x0F))  # Note off
        pos = track_end

    tempos.sort()
    if not tempos:
        tempos = [(0, 500000)]                              # MIDI default: 120 BPM

    def to_seconds(tk):
        """Convert absolute ticks to seconds, honouring every tempo change."""
        total, last, cur = 0.0, 0, tempos[0][1]
        for tt, us in tempos:
            if tt >= tk:
                break
            total += (tt - last) * cur / 1e6 / div
            last, cur = tt, us
        return total + (tk - last) * cur / 1e6 / div

    # Walk the events in time order keeping the set of sounding notes, and emit
    # a schedule entry whenever the highest one changes.
    events.sort(key=lambda e: (e[0], e[1]))
    voice = channel if channel is not None else choose_melody_channel(events)
    if voice is not None:
        events = [e for e in events if e[3] == voice]       # One voice: a motor is monophonic
    sounding, schedule, current, opened = set(), [], None, False
    i, n = 0, len(events)
    while i < n:
        tick = events[i][0]
        t = to_seconds(tick)
        if t > end:
            break
        # Apply EVERY event sharing this tick before looking at the result. A
        # chord is several note-ons at one instant, and the note-offs ending the
        # previous chord share that instant too. Reacting to them one at a time
        # makes the top note flicker through intermediate pitches and dip to
        # silence between chords - which is heard as a stutter, and costs one
        # register write per event.
        while i < n and events[i][0] == tick:
            _, on, pitch, _ch = events[i]
            if on:
                sounding.add(pitch)
            else:
                sounding.discard(pitch)
            i += 1
        top = max(sounding) if sounding else None
        if t < start:
            current = top                                   # Track silently: not in the window yet
            continue
        if not opened:
            # First tick inside the window: `current` is still whatever was
            # sounding when the window opened, so emit that at t=0.
            schedule.append((0.0, 0 if current is None else midi_to_hz(current)))
            opened = True
        if top != current:
            current = top
            emit(schedule, t - start, 0 if top is None else midi_to_hz(top))
    schedule.append((end - start, 0))                       # Always end in silence
    return schedule, voice


def choose_melody_channel(events):
    """Pick the MIDI channel most likely to carry the melody.

    In a keyboard arrangement the melody is the voice that plays one note at a
    time while the accompaniment moves in blocks, so the channel with the
    highest fraction of single-note onsets wins, and note count breaks ties.
    Picking by pitch instead does not work: the hands overlap, and a held
    accompaniment chord regularly sits above the melody and masks it.
    """
    onsets = {}                                             # channel -> {tick: note count}
    for tick, on, _pitch, ch in events:
        if on:
            onsets.setdefault(ch, {}).setdefault(tick, 0)
            onsets[ch][tick] += 1
    best, best_score = None, None
    for ch, ticks in onsets.items():
        mono = sum(1 for n in ticks.values() if n == 1) / len(ticks)
        score = (mono, len(ticks))                          # Monophonic first, then busiest
        if best_score is None or score > best_score:
            best, best_score = ch, score
    return best


def midi_to_hz(pitch):
    """Equal-tempered frequency of a MIDI note, octave-shifted into the motor's range."""
    f = 440.0 * (2.0 ** ((pitch - 69) / 12.0))
    while f < TONE_MIN_HZ:
        f *= 2.0                                            # Too low to radiate: raise an octave
    while f > TONE_MAX_HZ:
        f /= 2.0                                            # Too high to hear: drop an octave
    return int(round(f))


def silence(link, node):
    """Stop the tone and put the output stage back in its safe state."""
    write_reg(link, node, "tone_freq", 0)
    write_reg(link, node, "enable", 0)
    write_reg(link, node, "control_mode", 0)


def cmd_music(link, args):
    """Play the opening of a MIDI file through the motor coils."""
    end = args.end if args.end is not None else args.start + args.seconds
    if end <= args.start:
        print(f"--end ({end:g}s) must be after --start ({args.start:g}s)")
        return 1
    schedule, voice = parse_midi_melody(args.file, args.start, end, args.voice)
    notes = sum(1 for _, f in schedule if f)
    print(f"{os.path.basename(args.file)}: {notes} notes from {args.start:g}s to {end:g}s "
          f"({end - args.start:g}s, melody from channel {voice}), "
          f"amplitude {args.amp}/32768 of the bus")

    if not write_reg(link, args.node, "tone_amp", args.amp):
        print("drive rejected the amplitude (max 8192)")
        return 1
    if not write_reg(link, args.node, "control_mode", MODES.index("tone")):
        print("drive rejected tone mode - is the firmware new enough?")
        return 1
    write_reg(link, args.node, "enable", 1)

    t0 = time.time()
    try:
        for t, freq in schedule:
            delay = t0 + t - time.time()
            if delay > 0:
                time.sleep(delay)
            write_reg(link, args.node, "tone_freq", freq)
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        # Whatever happened, the coils must not be left singing.
        silence(link, args.node)
    print("done")
    return 0


def cmd_music_stop(link, args):
    """Silence the motor and disable the output stage."""
    silence(link, args.node)
    print("stopped")
    return 0


def write_reg(link, node, name, value):
    """Write one register by name. Returns True when the drive acknowledged."""
    rid, rtype = REGS[name]
    raw = encode_reg(rtype, str(value)) if isinstance(value, str) else int(value)
    payload = bytes([OP["reg_write"], 0]) + struct.pack("<H", rid) + struct.pack("<I", raw & 0xFFFFFFFF)
    reply = link.request(node, payload, timeout=0.4)
    return reply is not None and reply[0] != OP["nack"]


def require_motion_ready(link, args):
    """Check the drive can actually execute a trajectory before streaming one.

    Segments pushed at a drive in any other mode are accepted and queued, and
    then sit there: the executor only advances in MOTION mode. Without this
    check that looks exactly like a mechanical failure.
    """
    mode = read_reg(link, args.node, "control_mode")
    enabled = read_reg(link, args.node, "enable")
    if mode is None:
        print(f"node {args.node} is not answering")
        return False
    if (mode == 4) and enabled:
        return True
    if args.set_mode:
        print("  putting the drive into motion mode and enabling the output stage")
        payload = bytes([OP["control_mode"], 4, 1, 0, 0, 0, 0, 0])
        return expect_ok(link.request(args.node, payload), "  mode")
    name = MODES[mode] if mode < len(MODES) else str(mode)
    print(f"drive is in '{name}' mode with enable={enabled}: queued segments would never run.")
    print("Fix it with either:")
    print(f"    tpcp_host.py mode {args.node} motion --enable")
    print(f"    tpcp_host.py pvt {args.node} ... --set-mode")
    return False


def cmd_pvt(link, args):
    """Run a profiled move through the trajectory queue, optionally back and forth."""
    if not require_motion_ready(link, args):
        return 1
    vel = args.revolutions / args.seconds
    # Queue depth is no longer the limit - stream_pvt_leg paces against the
    # drive's free slots - but keeping up with consumption still is. The
    # executor takes a segment every `seconds/segments`, and the host has to
    # top the queue up at least that fast, which costs one request/response
    # round trip per refill. HID turns those round trips around in a
    # millisecond or two; CDC needs an order of magnitude longer, so only the
    # slow transport gets the warning.
    seg_ms = args.seconds / args.segments * 1000.0
    if seg_ms < 40.0 and not isinstance(link, HidLink):
        print(f"WARNING: {seg_ms:.1f}ms segments are finer than this transport can refill.")
        print(f"         Use about --segments {max(1, int(args.seconds / 0.05))} for a "
              f"{args.seconds:.2f}s leg, lengthen the leg, or use --interface hid.")
    two_way = args.back or (args.cycles > 1)
    legs = (1.0, -1.0) if two_way else (1.0,)
    print(f"{args.cycles} cycle(s), {args.revolutions:+.3f}rev per leg over "
          f"{args.seconds:.2f}s ({vel:.3f} RPS cruise), {args.segments} segments per leg")
    for _ in range(args.cycles):
        for direction in legs:
            stream_pvt_leg(link, args.node, args.revolutions * direction,
                           args.seconds, args.segments)
            if args.wait:
                if not wait_settled(link, args.node, args.tolerance, args.seconds + 15.0):
                    return 1
            else:
                # Without a wait the legs would all be queued at once and run
                # as one continuous move, so pace them by their own duration.
                time.sleep(args.seconds + 0.2)
    return 0


def cmd_queue(link, args):
    """Inspect or control the trajectory queue."""
    actions = {"status": None, "clear": 1, "pause": 2, "resume": 3, "scale": 4}
    if args.action == "status":
        reply = link.request(args.node, bytes([OP["queue_stat"], 0, 0, 0, 0, 0, 0, 0]))
        print(f"  {decode_chunk(reply) if reply else 'NO RESPONSE'}")
        return 0
    scale = int(round(args.scale * 1024)) if args.action == "scale" else 0
    payload = bytes([OP["queue_ctl"], actions[args.action]]) + struct.pack("<hI", scale, 0)[:6]
    return 0 if expect_ok(link.request(args.node, payload), f"queue {args.action}") else 1


def cmd_telemetry(link, args):
    """Configure one cyclic telemetry slot (period 0 disables it)."""
    op = OP.get(args.chunk)
    if op is None or op > OP["heartbeat"]:
        sys.exit(f"'{args.chunk}' is not a telemetry chunk")
    # Bit 7 of the flags byte is the apply bit; bit 0 enables the slot.
    flags = 0x80 | (0x01 if args.period else 0x00)
    payload = bytes([OP["telemetry"], args.slot, op if args.period else 0, flags]) + \
        struct.pack("<HH", args.period, 0)
    reply = link.request(args.node, payload)
    print(f"  {decode_chunk(reply) if reply else 'NO RESPONSE'}")
    return 0


def cmd_watchdog(link, args):
    """Configure the master-liveness watchdog (timeout 0 disables it)."""
    action = {"freewheel": 0, "brake": 1, "hold": 2}[args.action]
    flags = 0x80 | (0x01 if args.timeout else 0x00)
    payload = bytes([OP["watchdog"], action, flags, 0]) + struct.pack("<HH", args.timeout, 0)
    reply = link.request(args.node, payload)
    print(f"  {decode_chunk(reply) if reply else 'NO RESPONSE'}")
    return 0


def cmd_save(link, args):
    """Persist the node configuration (the drive must be disabled)."""
    target = {"all": 0, "config": 1, "motor": 2}[args.target]
    return 0 if expect_ok(link.request(args.node, bytes([OP["param_save"], target, 0, 0, 0, 0, 0, 0])),
                          "save") else 1


def cmd_defaults(link, args):
    """Restore defaults; 'all' also erases the stored record (loses the address)."""
    target = {"all": 0, "config": 1}[args.target]
    if target == 0:
        print("WARNING: this erases the stored configuration. The node will come "
              "back UNADDRESSED and will need a discovery sweep.")
    payload = bytes([OP["param_defaults"], target, 0, 0]) + struct.pack("<I", DEFAULTS_KEY)
    return 0 if expect_ok(link.request(args.node, payload), "defaults") else 1


def cmd_calibrate(link, args):
    """Start a calibration routine."""
    kind = {"offset": 0, "electrical": 1, "angle": 2}[args.kind]
    # Byte 2 is the flags byte; bit 1 forces a physical sweep. Without it a
    # drive whose motor profile is loaded reloads its stored angle table and
    # the shaft never turns, which reads as a request that did nothing.
    flags = 0x02 if args.force else 0x00
    ok = expect_ok(link.request(args.node, bytes([OP["calib_start"], kind, flags, 0, 0, 0, 0, 0])),
                   "calibrate")
    if not ok:
        return 1
    print("polling calibration status (Ctrl-C to stop)...")
    try:
        while True:
            reply = link.request(args.node, bytes([OP["calib_status"], 0, 0, 0, 0, 0, 0, 0]))
            if reply:
                print(f"  {decode_chunk(reply)}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        return 0


def cmd_reset(link, args):
    """Clear latched faults."""
    return 0 if expect_ok(link.request(args.node, bytes([OP["fault_reset"], 0, 0, 0, 0, 0, 0, 0])),
                          "fault reset") else 1


def cmd_reboot(link, args):
    """Reboot, optionally into the USB DFU bootloader."""
    payload = bytes([OP["reboot"], 1 if args.dfu else 0, 0, 0]) + struct.pack("<I", REBOOT_KEY)
    return 0 if expect_ok(link.request(args.node, payload), "reboot") else 1


def cmd_estop(link, args):
    """Broadcast an emergency stop to every node."""
    link.send(NMT_GLOBAL_CONTROL, bytes([0, 0, 0, 0, 0, 0, 0, 0]))
    print("emergency stop broadcast")
    return 0


def cmd_monitor(link, args):
    """Decode and print every protocol frame on the bus."""
    print("monitoring (Ctrl-C to stop)")
    t0 = time.time()
    try:
        while True:
            msg = link.recv(1.0)
            if msg is None:
                continue
            cid = msg.arbitration_id
            data = bytes(msg.data).ljust(8, b"\x00")
            if cid < 0x100:
                tag, node = "EMCY", cid
            elif cid < 0x200:
                tag, node = "NMT ", cid - ID_NMT
            elif cid < 0x300:
                tag, node = "SETP", cid - ID_SETPOINT
            elif cid < 0x400:
                tag, node = "REQ ", cid - ID_REQUEST
            elif cid < 0x500:
                tag, node = "RESP", cid - ID_RESPONSE
            elif cid < 0x600:
                print(f"{time.time() - t0:8.3f} DISC  uid_byte=0x{cid & 0xFF:02X}")
                continue
            else:
                tag, node = "TLM ", cid - ID_TELEMETRY
            if args.node is not None and node != args.node and tag != "NMT ":
                continue
            print(f"{time.time() - t0:8.3f} {tag} {node:3d}  {decode_chunk(data)}")
    except KeyboardInterrupt:
        return 0


def cmd_raw(link, args):
    """Send an arbitrary request frame, for poking at the protocol by hand."""
    payload = bytes.fromhex(args.hex.replace(" ", "")).ljust(8, b"\x00")[:8]
    reply = link.request(args.node, payload)
    print(f"  {decode_chunk(reply) if reply else 'NO RESPONSE'}")
    return 0

# ============================================================================
# Entry point
# ============================================================================


def main():
    p = argparse.ArgumentParser(
        description="Host-side master for the TunePulse CAN Protocol v1.",
        epilog="Register names for get/set: " + ", ".join(sorted(REGS)))
    p.add_argument("--interface", default="auto",
                   help="'auto' (default) uses the drive's HID control interface when one is "
                        "present, otherwise detects whether the channel is a TunePulse console "
                        "or a CAN adapter; 'hid' forces the USB HID transport; 'serial' forces "
                        "the USB CDC transport; anything else is a python-can interface name "
                        "such as slcan, pcan or socketcan")
    p.add_argument("--channel", default=None,
                   help="adapter channel or serial device. Omit it and the drive's console is "
                        "auto-detected; the TPCP_CHANNEL environment variable overrides that.")
    p.add_argument("--bitrate", type=int, default=500000, help="arbitration bit rate (default: 500000)")
    p.add_argument("--data-bitrate", type=int, default=2000000, help="CAN-FD data-phase bit rate")
    p.add_argument("--fd", action="store_true", help="use CAN-FD frames with bit-rate switching")
    p.add_argument("-v", "--verbose", action="store_true", help="dump every frame sent and received")
    p.add_argument("--log", action="store_true",
                   help="with --interface serial, also print the board's console output")
    sub = p.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("scan", help="discover unaddressed nodes and assign addresses")
    s.add_argument("--first-id", type=int, default=1, help="first address to hand out")
    s.add_argument("--window", type=float, default=0.05, help="arbitration collection window [s]")
    s.add_argument("--save", action="store_true", help="persist each assigned address")
    s.set_defaults(fn=cmd_scan)

    s = sub.add_parser("info", help="dump every status chunk of a node")
    s.add_argument("node", type=int)
    s.set_defaults(fn=cmd_info)

    s = sub.add_parser("get", help="read one register")
    s.add_argument("node", type=int)
    s.add_argument("reg")
    s.set_defaults(fn=cmd_get)

    s = sub.add_parser("set", help="write one register")
    s.add_argument("node", type=int)
    s.add_argument("reg")
    s.add_argument("value")
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

    s = sub.add_parser("move", help="command a position target")
    s.add_argument("node", type=int)
    s.add_argument("revolutions", type=float)
    s.add_argument("--relative", action="store_true")
    s.add_argument("--set-mode", action="store_true", help="switch to position mode too")
    s.add_argument("--wait", action="store_true",
                   help="block until the rotor actually arrives (setpoints are asynchronous: "
                        "without this the command returns as soon as the target is accepted)")
    s.add_argument("--tolerance", type=float, default=0.01, help="arrival window in revolutions")
    s.add_argument("--timeout", type=float, default=60.0, help="give up after this many seconds")
    s.set_defaults(fn=cmd_move)

    s = sub.add_parser("jog", help="command a velocity target")
    s.add_argument("node", type=int)
    s.add_argument("rps", type=float)
    s.add_argument("--set-mode", action="store_true", help="switch to velocity mode too")
    s.set_defaults(fn=cmd_jog)

    s = sub.add_parser("stop", help="stop with a selectable end state")
    s.add_argument("node", type=int)
    s.add_argument("style", nargs="?", default="hold",
                   choices=["hold", "freewheel", "brake", "quick"])
    s.set_defaults(fn=cmd_stop)

    s = sub.add_parser("pvt", help="stream PVT segments into the trajectory queue")
    s.add_argument("node", type=int)
    s.add_argument("revolutions", type=float)
    s.add_argument("--seconds", type=float, default=2.0, help="duration of one leg")
    s.add_argument("--back", action="store_true", help="also return to the start")
    s.add_argument("--segments", type=int, default=20)
    s.add_argument("--wait", action="store_true", help="block until the queue drains and the rotor arrives")
    s.add_argument("--tolerance", type=float, default=0.01, help="arrival window in revolutions")
    s.add_argument("--cycles", type=int, default=1, help="repeat the move forwards and back this many times")
    s.add_argument("--set-mode", action="store_true",
                   help="put the drive into motion mode and enable it first")
    s.set_defaults(fn=cmd_pvt)

    s = sub.add_parser("music", help="play the opening of a MIDI file through the motor coils")
    s.add_argument("node", type=int)
    s.add_argument("file", help="standard MIDI file (.mid)")
    s.add_argument("--start", type=float, default=0.0, help="first second of the piece to play")
    s.add_argument("--end", type=float, default=None,
                   help="last second to play; overrides --seconds")
    s.add_argument("--seconds", type=float, default=30.0,
                   help="how long to play for when --end is not given (default: 30)")
    s.add_argument("--amp", type=int, default=2048,
                   help="tone amplitude, Q15 fraction of the bus voltage (max 8192)")
    # NOT --channel: that is the global transport option (serial port or HID
    # device path), and a subcommand redefining it silently receives the
    # transport's value instead of the user's.
    s.add_argument("--voice", type=int, default=None, choices=range(16), metavar="0-15",
                   help="MIDI channel to play; default picks the most monophonic voice")
    s.set_defaults(fn=cmd_music)

    s = sub.add_parser("music-stop", help="silence the motor and disable the output stage")
    s.add_argument("node", type=int)
    s.set_defaults(fn=cmd_music_stop)

    s = sub.add_parser("queue", help="inspect or control the trajectory queue")
    s.add_argument("node", type=int)
    s.add_argument("action", nargs="?", default="status",
                   choices=["status", "clear", "pause", "resume", "scale"])
    s.add_argument("scale", nargs="?", type=float, default=1.0,
                   help="playback rate for 'scale' (1.0 = normal, 0.5 = half speed)")
    s.set_defaults(fn=cmd_queue)

    s = sub.add_parser("telemetry", help="configure a cyclic telemetry slot")
    s.add_argument("node", type=int)
    s.add_argument("slot", type=int, choices=[0, 1, 2, 3])
    s.add_argument("chunk", help="chunk name, e.g. motion1 / heartbeat / electric_status")
    s.add_argument("period", type=int, help="period in ms (0 disables the slot)")
    s.set_defaults(fn=cmd_telemetry)

    s = sub.add_parser("watchdog", help="configure the master-liveness watchdog")
    s.add_argument("node", type=int)
    s.add_argument("timeout", type=int, help="timeout in ms (0 disables it)")
    s.add_argument("action", nargs="?", default="freewheel",
                   choices=["freewheel", "brake", "hold"])
    s.set_defaults(fn=cmd_watchdog)

    s = sub.add_parser("save", help="persist configuration (drive must be disabled)")
    s.add_argument("node", type=int)
    s.add_argument("target", nargs="?", default="config", choices=["all", "config", "motor"])
    s.set_defaults(fn=cmd_save)

    s = sub.add_parser("defaults", help="restore default configuration")
    s.add_argument("node", type=int)
    s.add_argument("target", nargs="?", default="config", choices=["all", "config"])
    s.set_defaults(fn=cmd_defaults)

    s = sub.add_parser("calibrate", help="start a calibration routine and poll its progress")
    s.add_argument("node", type=int)
    s.add_argument("kind", choices=["offset", "electrical", "angle"])
    s.add_argument("--force", action="store_true",
                   help="ignore the stored angle table and sweep the shaft for real")
    s.set_defaults(fn=cmd_calibrate)

    s = sub.add_parser("reset", help="clear latched faults")
    s.add_argument("node", type=int)
    s.set_defaults(fn=cmd_reset)

    s = sub.add_parser("reboot", help="reboot a node")
    s.add_argument("node", type=int)
    s.add_argument("--dfu", action="store_true", help="jump into the USB DFU bootloader")
    s.set_defaults(fn=cmd_reboot)

    s = sub.add_parser("estop", help="broadcast an emergency stop to every node")
    s.set_defaults(fn=cmd_estop)

    s = sub.add_parser("monitor", help="decode every protocol frame on the bus")
    s.add_argument("node", type=int, nargs="?", default=None)
    s.set_defaults(fn=cmd_monitor)

    s = sub.add_parser("raw", help="send an arbitrary request frame")
    s.add_argument("node", type=int)
    s.add_argument("hex", help="payload bytes, e.g. '01 00 00 00 00 00 00 00'")
    s.set_defaults(fn=cmd_raw)

    args = p.parse_args()

    # ---- Resolve the channel and the transport in ONE pass ----------------
    # Every step here costs wall-clock time on every command, so the order is
    # cheapest-first: an explicit flag, then the environment, then the port that
    # answered last time, and only then a scan of the whole machine.
    import os
    explicit = args.channel is not None
    if args.channel is None:
        args.channel = os.environ.get("TPCP_CHANNEL")
        explicit = args.channel is not None

    node = None
    is_console = False

    # The HID control interface is preferred whenever the board exposes one:
    # it needs no port guessing, no baud rate and no framing, and it is the
    # only transport that streams a trajectory burst at full speed. Serial
    # detection below is skipped entirely once it is found.
    if args.interface in ("auto", "hid"):
        hid_path = find_hid_path(args.channel if explicit else None)
        if hid_path is not None:
            if args.interface == "auto":
                print("detected: TunePulse HID control interface")
            args.interface = "hid"
            args.channel = hid_path
            link = HidLink(args)
            try:
                return args.fn(link, args)
            finally:
                link.close()
        if args.interface == "hid":
            sys.exit("no TunePulse HID interface found.\n"
                     "  - check the board runs firmware with the composite USB device\n"
                     "  - hidapi must be installed:  pip install hidapi\n"
                     "  - on Linux a udev rule may be needed to open it without root")

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

    # macOS: /dev/tty.* can block on open; /dev/cu.* is the right device.
    args.channel, note = normalize_port(args.channel)
    if note:
        print(f"note: {note}")

    # Decide the transport, reusing whatever the steps above already learned so
    # a port is never interrogated twice for the same answer.
    if not is_console:
        found, node = probe_drive(args.channel)
        is_console = found
        if not is_console:
            # Unaddressed drives answer nothing, so listen for the console.
            is_console, node, _ = sniff_port(args.channel)

    if args.interface == "auto":
        args.interface = "serial" if is_console else "slcan"
        if not is_console:
            print("detected: no TunePulse console on this port -> assuming a CAN adapter (slcan)")

    if args.interface == "serial":
        if is_console:
            cache_write(args.channel)
        else:
            print(f"WARNING: nothing that looks like a TunePulse console is talking on "
                  f"{args.channel}.\n"
                  f"         Check the board is powered and running, and that this is its own USB\n"
                  f"         port rather than the ST-Link's virtual COM port.")

    # An unaddressed drive answers network management only, so every request
    # would time out. Say so once, with the fix, instead of sixteen times.
    target = getattr(args, "node", None)
    if (node == 0) and (target is not None) and (args.cmd not in ("scan", "monitor", "estop")):
        print(f"\nThe drive reports node=0 (UNASSIGNED). It answers network management only,\n"
              f"so every request to node {target} will time out. Give it an address first:\n\n"
              f"    {sys.argv[0]} --channel {args.channel} scan\n")
        return 1

    link = SerialLink(args) if args.interface == "serial" else Link(args)
    try:
        return args.fn(link, args)
    finally:
        link.close()


if __name__ == "__main__":
    sys.exit(main())
