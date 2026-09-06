#!/usr/bin/env python3
"""Model Context Protocol server for the TunePulse drive.

This exposes the same drive that tools/tpcp_host.py drives from a shell, but
as a set of MCP tools an assistant can call directly: connect, read status,
read and write registers, select a control mode, move the shaft, stream a
profiled trajectory, calibrate, persist, and play a MIDI file through the
motor coils.

Why a second front end rather than shelling out to tpcp_host.py
---------------------------------------------------------------
The CLI is built for a human: it prints prose and returns an exit code. A
model needs the numbers. Every tool here answers with JSON - decoded chunk
fields, register values, queue depth, calibration flags - and keeps the CLI's
prose alongside it under "log" when the underlying helper produces some.

It also keeps the link open. Shelling out re-enumerates USB, re-opens the HID
device and re-probes the port on every single command, which costs the better
part of a second each time and makes a poll loop useless. One long-lived
connection turns a status read into a millisecond.

Transport
---------
Stdio JSON-RPC 2.0, newline-delimited, implemented here against the MCP
specification rather than through the reference SDK. That is deliberate: the
drive's hidapi bindings live in whichever interpreter the bench already uses
(here, PlatformIO's own virtualenv), and requiring an SDK install into that
environment is a far bigger imposition than a few hundred lines of protocol.
The only imports are the standard library plus tpcp_host itself.

    STDOUT CARRIES JSON-RPC AND NOTHING ELSE. tpcp_host prints freely, so
    sys.stdout is rebound to stderr at startup and the real stream is kept
    private to the transport. Any stray print therefore lands in the client's
    log instead of corrupting a frame.

Registering it
--------------
    claude mcp add tunepulse -- /path/to/python3 /path/to/tools/tpcp_mcp.py

    # Read-only: every tool that could move the shaft or write flash is
    # withheld, not merely refused.
    claude mcp add tunepulse -- /path/to/python3 /path/to/tools/tpcp_mcp.py --read-only

Safety
------
Tools that turn the rotor say so in their description, and the ones that
cannot be undone by another command - erasing the stored configuration,
rebooting into DFU - refuse to act without confirm=true. --read-only removes
the whole mutating half of the surface for a session that should only look.

The drive's own protections are unchanged and still apply: the output stage
must be enabled, the trajectory queue only advances in motion mode, and the
watchdog still stops a drive whose master goes quiet.

Copyright 2026, creapunk
Licensed under the Apache License, Version 2.0
"""

import contextlib
import io
import json
import os
import struct
import sys
import threading
import time
import types

# tpcp_host.py sits next to this file and is imported, not executed: it holds
# the wire format, the codecs, the register map and both USB transports, and
# duplicating any of that here would guarantee the two drift apart.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    import can  # noqa: F401                       # Present on a bench with a CAN adapter
except ImportError:
    # tpcp_host imports python-can unconditionally and exits if it is missing,
    # but only the CAN transport ever touches it. A bench talking to the drive
    # over USB should not have to install a CAN stack, so stand in a module
    # whose every attribute raises - the failure then happens if and only if a
    # CAN interface is actually selected, with a message that says so.
    class _MissingCan(types.ModuleType):
        def __getattr__(self, name):
            raise RuntimeError("python-can is not installed: only the 'hid' and 'serial' "
                               "transports are available (pip install python-can for CAN)")
    sys.modules["can"] = _MissingCan("can")

import tpcp_host as tp                             # noqa: E402  (path set up above)

SERVER_NAME = "tunepulse"
SERVER_VERSION = "1.0.0"

# Protocol revisions whose tool and resource surface is identical to what this
# server implements. The client's own revision is echoed back when it is one of
# these, which is what the specification asks for; anything else negotiates
# down to the oldest one everybody understands.
SUPPORTED_PROTOCOLS = ("2025-06-18", "2025-03-26", "2024-11-05")
FALLBACK_PROTOCOL = "2024-11-05"

# The real stdout, captured before anything can print to it. Every JSON-RPC
# message goes here and nothing else ever does.
_TRANSPORT_OUT = sys.stdout
sys.stdout = sys.stderr


# ============================================================================
# Minimal MCP server over stdio
# ============================================================================


class McpError(Exception):
    """A tool failure that should be reported to the model, not crash the server."""


class Tool:
    """One callable exposed to the client, with the schema that describes it."""

    def __init__(self, name, description, schema, fn, mutating):
        self.name = name                  # Identifier the client calls
        self.description = description    # What it does, in the model's terms
        self.schema = schema              # JSON Schema for the arguments
        self.fn = fn                      # Python implementation
        self.mutating = mutating          # Withheld entirely in --read-only mode

    def spec(self):
        """The tools/list entry for this tool."""
        return {"name": self.name, "description": self.description, "inputSchema": self.schema}


TOOLS = {}          # name -> Tool
RESOURCES = {}      # uri  -> (name, description, mime, callable returning text)


def tool(name, description, properties=None, required=(), mutating=True):
    """Register a function as an MCP tool.

    The schema is assembled from the property map rather than written out in
    full, because every tool here takes a flat object and the boilerplate would
    otherwise be longer than the implementations.
    """
    schema = {
        "type": "object",
        "properties": properties or {},
        "required": list(required),
        # Typos in argument names are a real failure mode for a generated call,
        # and silently ignoring an unknown key turns one into a wrong action.
        "additionalProperties": False,
    }

    def wrap(fn):
        TOOLS[name] = Tool(name, description, schema, fn, mutating)
        return fn
    return wrap


def resource(uri, name, description, mime="text/plain"):
    """Register a read-only document the client can pull in for context."""
    def wrap(fn):
        RESOURCES[uri] = (name, description, mime, fn)
        return fn
    return wrap


class Server:
    """JSON-RPC 2.0 dispatch loop speaking the MCP stdio transport."""

    def __init__(self, read_only=False):
        self.read_only = read_only
        self.out = _TRANSPORT_OUT

    # -- framing ------------------------------------------------------------

    def _write(self, message):
        """Emit one JSON-RPC message as a single line on the real stdout."""
        self.out.write(json.dumps(message, separators=(",", ":")) + "\n")
        self.out.flush()

    def _result(self, req_id, result):
        self._write({"jsonrpc": "2.0", "id": req_id, "result": result})

    def _error(self, req_id, code, message):
        self._write({"jsonrpc": "2.0", "id": req_id, "error": {"code": code, "message": message}})

    # -- surface ------------------------------------------------------------

    def _visible_tools(self):
        """Tools this session is allowed to offer, in registration order."""
        return [t for t in TOOLS.values() if not (self.read_only and t.mutating)]

    def _call_tool(self, name, arguments):
        """Run one tool and package its answer as MCP tool content.

        Failures come back as content with isError set rather than as a
        JSON-RPC error: the distinction the specification draws is that a
        protocol error means the call could not be made at all, while a drive
        that answered NACK is a result the model needs to read and act on.
        """
        tool_obj = TOOLS.get(name)
        if tool_obj is None:
            raise McpError(f"unknown tool '{name}'")
        if self.read_only and tool_obj.mutating:
            raise McpError(f"'{name}' is withheld: this server was started with --read-only")

        # tpcp_host writes its human-readable narration to stdout. Capture it
        # so it can travel with the result instead of vanishing into the log.
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            payload = tool_obj.fn(**(arguments or {}))
        log = buf.getvalue().strip()
        if log and isinstance(payload, dict) and "log" not in payload:
            payload["log"] = log
        return {"content": [{"type": "text", "text": json.dumps(payload, indent=2, default=str)}],
                "isError": False}

    # -- dispatch -----------------------------------------------------------

    def handle(self, msg):
        """Process one decoded message; returns nothing, replies are written."""
        method = msg.get("method")
        req_id = msg.get("id")
        params = msg.get("params") or {}

        # A message with no id is a notification: the specification forbids
        # answering it, including the cancellations a client sends freely.
        if req_id is None:
            return

        if method == "initialize":
            asked = params.get("protocolVersion")
            version = asked if asked in SUPPORTED_PROTOCOLS else FALLBACK_PROTOCOL
            self._result(req_id, {
                "protocolVersion": version,
                "capabilities": {"tools": {"listChanged": False},
                                 "resources": {"listChanged": False, "subscribe": False}},
                "serverInfo": {"name": SERVER_NAME, "version": SERVER_VERSION},
                "instructions": INSTRUCTIONS,
            })
            return

        if method == "ping":
            self._result(req_id, {})
            return

        if method == "tools/list":
            self._result(req_id, {"tools": [t.spec() for t in self._visible_tools()]})
            return

        if method == "tools/call":
            try:
                self._result(req_id, self._call_tool(params.get("name"), params.get("arguments")))
            except McpError as exc:
                self._result(req_id, {"content": [{"type": "text", "text": str(exc)}],
                                      "isError": True})
            except Exception as exc:  # noqa: BLE001 - a bad call must not kill the link
                import traceback
                traceback.print_exc(file=sys.stderr)
                self._result(req_id, {"content": [{"type": "text",
                                                   "text": f"{type(exc).__name__}: {exc}"}],
                                      "isError": True})
            return

        if method == "resources/list":
            self._result(req_id, {"resources": [
                {"uri": uri, "name": n, "description": d, "mimeType": m}
                for uri, (n, d, m, _) in RESOURCES.items()]})
            return

        if method == "resources/read":
            uri = params.get("uri")
            entry = RESOURCES.get(uri)
            if entry is None:
                self._error(req_id, -32602, f"unknown resource '{uri}'")
                return
            _, _, mime, fn = entry
            self._result(req_id, {"contents": [{"uri": uri, "mimeType": mime, "text": fn()}]})
            return

        if method in ("prompts/list",):
            self._result(req_id, {"prompts": []})
            return

        self._error(req_id, -32601, f"method '{method}' is not implemented")

    def run(self):
        """Read newline-delimited messages until the client closes the pipe."""
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            try:
                msg = json.loads(line)
            except json.JSONDecodeError as exc:
                self._error(None, -32700, f"parse error: {exc}")
                continue
            try:
                self.handle(msg)
            except Exception:  # noqa: BLE001 - never let one message end the session
                import traceback
                traceback.print_exc(file=sys.stderr)
        DRIVE.close()


# ============================================================================
# Drive session
# ============================================================================


class _Args:
    """Stand-in for the argparse namespace tpcp_host's helpers expect.

    Those helpers were written against a parsed command line. Rebuilding one
    here is what lets the tricky parts - queue-paced trajectory streaming,
    settle detection, the discovery sweep - be reused verbatim instead of
    reimplemented and then drift.
    """

    def __init__(self, **kw):
        self.interface = "auto"
        self.channel = None
        self.bitrate = 500000
        self.data_bitrate = 2000000
        self.fd = False
        self.verbose = False
        self.log = False
        self.__dict__.update(kw)


class Drive:
    """The one open connection to the drive, shared by every tool.

    Holding the link open is the point: re-enumerating USB per command costs
    most of a second, which makes any kind of polling pointless. The lock is
    what makes that safe - a MIDI playback thread writes the tone register
    while tool calls arrive, and the request/response correlation in both
    transports assumes one conversation at a time.
    """

    def __init__(self):
        self.link = None            # HidLink, SerialLink or Link
        self.lock = threading.RLock()
        self.interface = None       # Transport actually in use
        self.channel = None         # Device path or adapter channel
        self.node = 1               # Default node when a tool does not name one
        self.opened_at = None       # When the link came up
        self.player = None          # Background MIDI playback thread
        self.stop_playback = threading.Event()
        self.pref_interface = "auto"   # Transport a lazy open should try first
        self.pref_channel = None       # Port a lazy open should use, if pinned

    # -- lifecycle ----------------------------------------------------------

    def open(self, interface="auto", channel=None, node=None, verbose=False, log=False):
        """Open a link, replacing any that is already up.

        Mirrors the transport selection in tpcp_host.main(): the HID control
        interface wins whenever the board exposes one, because it needs no port
        guessing, no baud rate and no framing, and it is the only transport
        that streams a trajectory burst at full speed.
        """
        self.close()
        args = _Args(interface=interface, channel=channel, verbose=verbose, log=log)
        detail = {}

        if interface in ("auto", "hid"):
            path = tp.find_hid_path(channel if channel else None)
            if path is not None:
                args.interface, args.channel = "hid", path
                self.link = tp.HidLink(args)
                self.interface = "hid"
                # hidapi hands back the OS path as bytes; it is only ever shown.
                self.channel = path.decode(errors="replace") if isinstance(path, bytes) else path
                detail["note"] = "using the drive's HID control interface"
            elif interface == "hid":
                raise McpError(
                    "no TunePulse HID interface found. Check the board runs firmware with the "
                    "composite USB device, that hidapi is installed, and on Linux that a udev "
                    "rule lets it be opened without root.")

        if self.link is None:
            # No HID interface: fall back to the CDC console, auto-detecting
            # the port the same way the CLI does when none was named.
            if args.channel is None and interface in ("auto", "serial"):
                found, why = tp.autodetect_channel()
                if found is None:
                    raise McpError(f"could not find a drive: {why}")
                args.channel, detail["note"] = found, why
            if args.channel is None:
                raise McpError(f"interface '{interface}' needs an explicit channel")
            args.channel, note = tp.normalize_port(args.channel)
            if note:
                detail["port_note"] = note
            if interface == "auto":
                is_console, probed, _ = tp.sniff_port(args.channel, 1.2)
                args.interface = "serial" if is_console else "slcan"
                if probed is not None:
                    detail["console_reports_node"] = probed
            self.link = tp.SerialLink(args) if args.interface == "serial" else tp.Link(args)
            self.interface = args.interface
            self.channel = args.channel

        self.opened_at = time.time()
        if node is not None:
            self.node = int(node)
        return detail

    def close(self):
        """Drop the link and release the USB device back to other tools."""
        self.stop_playback.set()
        player = self.player
        if player is not None and player.is_alive():
            player.join(timeout=2.0)
        self.player = None
        with self.lock:
            if self.link is not None:
                try:
                    self.link.close()
                except Exception:  # noqa: BLE001 - a closing error is not actionable
                    pass
            self.link = None
            self.interface = None
            self.channel = None
            self.opened_at = None

    def ensure(self):
        """Return the open link, opening one with defaults if there is none."""
        if self.link is None:
            self.open(self.pref_interface, self.pref_channel)
        return self.link

    def target(self, node):
        """Resolve the node a tool should address."""
        return self.node if node is None else int(node)

    # -- protocol -----------------------------------------------------------

    def request(self, node, payload, timeout=0.5):
        """Send one request and return the eight-byte answer, or None."""
        with self.lock:
            return self.ensure().request(node, payload, timeout)

    def chunk(self, node, key, timeout=0.5):
        """Request one status chunk by opcode name and decode it."""
        reply = self.request(node, bytes([tp.OP[key], 0, 0, 0, 0, 0, 0, 0]), timeout)
        if reply is None:
            return None
        return decode_frame(reply)

    def command(self, node, payload, what, timeout=0.5):
        """Send a request that answers ACK or NACK and report which.

        The NACK layout is the one place this protocol is easy to misread: byte
        1 echoes the opcode that was rejected and the error code is in byte 2.
        Reading byte 1 as the error turns every rejection into a nonsense
        diagnosis, so it is decoded once, here.
        """
        reply = self.request(node, payload, timeout)
        if reply is None:
            return {"ok": False, "command": what, "error": "no response",
                    "hint": "the drive did not answer - check the node address and the link"}
        if reply[0] == tp.OP["nack"]:
            return {"ok": False, "command": what,
                    "rejected": tp.OP_NAME.get(reply[1], f"0x{reply[1]:02X}"),
                    "error": tp.ERRORS.get(reply[2], f"0x{reply[2]:02X}"),
                    "detail": reply[3]}
        return {"ok": True, "command": what, "detail": reply[3] if reply[0] == tp.OP["ack"] else None,
                "reply": decode_frame(reply)}


DRIVE = Drive()


# ============================================================================
# Structured chunk decoding
# ============================================================================
#
# tpcp_host.decode_chunk renders a chunk as a line of text for a human. A model
# needs the fields themselves, so each layout is decoded once more into a dict.
# The text form is carried alongside because it is compact and unambiguous, and
# because it is what the CLI prints - the two views of one frame always agree.

MOTOR_TYPES = {0: "none", 1: "stepper", 2: "bldc", 3: "error"}
PHASE_PATTERNS = {0: "none", 1: "ABC", 2: "CAB", 3: "ACB"}

# Calibration stage names, from MotorStatus in the firmware. Stage and progress
# describe the ELECTRICAL identification machine only: during an angle sweep
# they keep reporting whatever the last electrical run left behind, so an angle
# sweep has to be followed through the flag bits instead.
CALIB_STAGES = {0: "success", 1: "not-calibrated", 2: "calibrating", 3: "hardware-error",
                4: "motor-error", 5: "config-error", 6: "calibration-error"}
CALIB_FLAGS = {0x01: "calibrated", 0x02: "settling", 0x04: "persisting", 0x08: "failed"}


def _bf16(d, off):
    """Read a bfloat16 field."""
    return tp.bf16_to_float(struct.unpack("<H", d[off:off + 2])[0])


def decode_frame(d):
    """Decode one eight-byte frame into {"chunk", "text", ...fields}."""
    d = bytes(d).ljust(8, b"\x00")
    op = d[0]
    out = {"chunk": tp.OP_NAME.get(op, f"0x{op:02X}"), "text": tp.decode_chunk(d),
           "raw": d.hex(" ")}

    if op & 0x80:                                    # Bit-packed motion burst
        out.update(chunk="burst",
                   displacement_rev=tp.signext(((d[0] & 0x3F) << 16) | (d[1] << 8) | d[2], 22) / 65536.0,
                   velocity_rps=tp.f24_to_float((d[3] << 16) | (d[4] << 8) | d[5]),
                   tail=(d[6] << 8) | d[7])
        return out

    if op == tp.OP["device_info"]:
        fw = struct.unpack("<H", d[2:4])[0]
        out.update(hw_version=d[1], firmware=f"{fw >> 8}.{fw & 0xFF}",
                   device_code=f"0x{struct.unpack('<I', d[4:8])[0]:08X}")
    elif op == tp.OP["device_uid"]:
        out.update(uid=d[1:8].hex())
    elif op == tp.OP["device_status"]:
        st = struct.unpack("<I", d[4:8])[0]
        out.update(uptime_s=int.from_bytes(d[1:4], "little"),
                   faults=_bits(st & 0xFF, tp.FAULT_BITS),
                   warnings=_bits((st >> 8) & 0xFF, tp.WARN_BITS),
                   state=tp.STATES[(st >> 16) & 0xF],
                   mode=tp.MODES[min((st >> 20) & 0xF, len(tp.MODES) - 1)])
    elif op == tp.OP["motor_status"]:
        fl = struct.unpack("<I", d[4:8])[0]
        out.update(mode=tp.MODES[min(d[1] & 0xF, len(tp.MODES) - 1)],
                   motor_type=MOTOR_TYPES[(d[1] >> 4) & 3],
                   phase_pattern=PHASE_PATTERNS[(d[1] >> 6) & 3],
                   pole_pairs=d[2], temperature_c=d[3],
                   direction="CCW" if fl & 1 else "CW",
                   enabled=bool(fl & 2), calibrated=bool(fl & 4))
    elif op == tp.OP["electric_status"]:
        out.update(efficiency_pct=None if d[1] == 255 else d[1] * 100 // 254,
                   power_w=_bf16(d, 2), supply_v=_bf16(d, 4), current_q_a=_bf16(d, 6))
    elif op == tp.OP["motion1"]:
        out.update(mode=d[1], position_rev=tp.i48_to_rev(d[2:8]))
    elif op == tp.OP["motion2"]:
        out.update(velocity_rps=tp.f24_to_float(d[1] | (d[2] << 8) | (d[3] << 16)),
                   accel_rps2=_bf16(d, 4), current_q_a=_bf16(d, 6))
    elif op == tp.OP["motion3"]:
        qs = struct.unpack("<H", d[6:8])[0]
        out.update(position_error_rev=tp.i24_to_rev(d[1:4]), velocity_error_rps=_bf16(d, 4),
                   queued=qs & 0xF, time_scale=tp.signext(qs >> 4, 12) / 256.0)
    elif op == tp.OP["motor_props"]:
        out.update(temp_limit_c=d[1], resistance_ohm=_bf16(d, 2),
                   inductance_uh=_bf16(d, 4) * 1e6, torque_constant_nm_a=_bf16(d, 6))
    elif op == tp.OP["encoder"]:
        out.update(health=f"0x{d[1]:02X}", angle=struct.unpack("<H", d[2:4])[0],
                   turns=struct.unpack("<i", d[4:8])[0])
    elif op == tp.OP["driver_status"]:
        out.update(state=tp.STATES[d[1] & 0xF], mode=tp.MODES[min(d[1] >> 4, len(tp.MODES) - 1)],
                   faults=_bits(d[2], tp.FAULT_BITS), warnings=_bits(d[3], tp.WARN_BITS),
                   op_status=d[4], motor_status=d[5])
    elif op == tp.OP["calib_status"]:
        flags = d[4]
        out.update(kind=d[1], kind_name={0: "current-offset", 1: "electrical", 2: "angle"}.get(d[1], "idle" if d[1] == 0xFF else str(d[1])),
                   stage=d[2], stage_name=CALIB_STAGES.get(d[2], f"stage {d[2]}"),
                   progress_pct=d[3] * 100 // 255, flags=f"0x{flags:02X}",
                   calibrated=bool(flags & 0x01), settling=bool(flags & 0x02),
                   persisting=bool(flags & 0x04),
                   failed=bool(flags & 0x08) or d[2] == 6,
                   cal_id=d[5], pole_pairs=struct.unpack("<H", d[6:8])[0],
                   note="stage/progress track the ELECTRICAL identification only; an angle "
                        "sweep is in progress while calibrated=false or settling/persisting")
    elif op == tp.OP["comm_status"]:
        out.update(bus_state=f"0x{d[1]:02X}", tx_errors=d[2], rx_errors=d[3],
                   dropped=struct.unpack("<H", d[4:6])[0],
                   nacks=struct.unpack("<H", d[6:8])[0])
    elif op == tp.OP["power_status"]:
        out.update(load_pct=struct.unpack("b", d[1:2])[0], bus_current_a=_bf16(d, 2),
                   bus_power_w=_bf16(d, 4), thermal_headroom_c=d[6])
    elif op == tp.OP["heartbeat"]:
        out.update(state=tp.STATES[d[1] & 0xF], mode=tp.MODES[min(d[1] >> 4, len(tp.MODES) - 1)],
                   faults=_bits(d[2], tp.FAULT_BITS), warnings=_bits(d[3], tp.WARN_BITS),
                   supply_v=d[4] * 0.25, temperature_c=struct.unpack("b", d[5:6])[0],
                   load_pct=struct.unpack("b", d[6:7])[0],
                   seq=d[7] & 0xF, queued=d[7] >> 4)
    elif op == tp.OP["queue_stat"]:
        out.update(state=f"0x{d[1]:02X}", queued=d[2], free=d[3],
                   time_scale=struct.unpack("<h", d[4:6])[0] / 1024.0,
                   dropped=struct.unpack("<H", d[6:8])[0])
    elif op == tp.OP["param_status"]:
        out.update(state=f"0x{d[1]:02X}", config_version=d[2], cal_id=d[3],
                   config_free_bytes=struct.unpack("<H", d[4:6])[0])
    elif op == tp.OP["calib_result"]:
        out.update(flags=f"0x{d[1]:02X}", resistance_ohm=_bf16(d, 2),
                   inductance_d_uh=_bf16(d, 4) * 1e6, inductance_q_uh=_bf16(d, 6) * 1e6)
    elif op in (tp.OP["reg_read"], tp.OP["reg_write"], tp.OP["reg_info"]):
        rid = struct.unpack("<H", d[2:4])[0]
        rname, _ = tp.REG_BY_ID.get(rid, (f"0x{rid:04X}", "u32"))
        rtype = tp.TYPE_NAMES[d[1]] if d[1] < len(tp.TYPE_NAMES) else "u32"
        out.update(register=rname, id=f"0x{rid:04X}", type=rtype,
                   value=tp.decode_reg(rtype, struct.unpack("<I", d[4:8])[0]))
    elif op == tp.OP["node_id"]:
        out.update(node=d[1], flags=f"0x{d[2]:02X}", state=f"0x{d[3]:02X}")
    elif op == tp.OP["ack"]:
        out.update(acked=tp.OP_NAME.get(d[1], f"0x{d[1]:02X}"), detail=d[3])
    elif op == tp.OP["nack"]:
        out.update(rejected=tp.OP_NAME.get(d[1], f"0x{d[1]:02X}"),
                   error=tp.ERRORS.get(d[2], f"0x{d[2]:02X}"), detail=d[3])
    return out


def _bits(value, names):
    """Return the names of the set bits, as a list."""
    return [n for i, n in enumerate(names) if value & (1 << i)]


# ============================================================================
# Tools: link
# ============================================================================

NODE_ARG = {"type": "integer", "minimum": 0, "maximum": 255,
            "description": "Node address. Omitted, the session default (set by connect) is used."}


@tool("connect",
      "Open a link to the drive and report what answered. Prefers the USB HID control interface, "
      "falling back to auto-detecting the CDC console. Safe: it only reads. Calling it again "
      "reconnects. Other tools connect automatically, so this is only needed to pin a transport, "
      "a port or the default node.",
      {"interface": {"type": "string",
                     "enum": ["auto", "hid", "serial", "slcan", "pcan", "socketcan"],
                     "description": "Transport. 'auto' (default) picks HID when the board exposes "
                                    "it, otherwise the console."},
       "channel": {"type": "string",
                   "description": "Serial device or adapter channel. Omit to auto-detect."},
       "node": NODE_ARG},
      mutating=False)
def t_connect(interface="auto", channel=None, node=None):
    detail = DRIVE.open(interface, channel, node)
    info = DRIVE.chunk(DRIVE.node, "device_info")
    status = DRIVE.chunk(DRIVE.node, "device_status")
    out = {"connected": True, "interface": DRIVE.interface, "channel": DRIVE.channel,
           "default_node": DRIVE.node, "device": info, "status": status}
    out.update(detail)
    if info is None:
        out["warning"] = (f"the link is open but node {DRIVE.node} did not answer. Run 'scan' if "
                          f"the drive is unaddressed, or set a different node.")
    return out


@tool("disconnect",
      "Close the link and release the USB device. A HID device can only be held by one process, "
      "so this is what frees it for tpcp_host.py or the desktop app.",
      mutating=False)
def t_disconnect():
    was = DRIVE.interface
    DRIVE.close()
    return {"connected": False, "released": was}


@tool("link_status", "Report whether a link is open, over what, and to which default node.",
      mutating=False)
def t_link_status():
    if DRIVE.link is None:
        return {"connected": False,
                "hint": "any tool will connect on its own; use 'connect' to pin a transport"}
    counters = {}
    for name in ("rx_count", "tx_count", "dropped", "errors"):
        fn = getattr(DRIVE.link, name, None)
        if callable(fn):
            counters[name] = fn()
    out = {"connected": True, "interface": DRIVE.interface, "channel": DRIVE.channel,
           "default_node": DRIVE.node, "open_for_s": round(time.time() - DRIVE.opened_at, 1),
           "playing_midi": bool(DRIVE.player and DRIVE.player.is_alive())}
    if counters:
        out["transport_counters"] = counters      # Only the CDC transport keeps any
    return out


@tool("scan",
      "Run a UID discovery sweep and hand out node addresses to unaddressed drives. Needed only "
      "when a drive reports node 0, which answers network management and nothing else.",
      {"first_id": {"type": "integer", "minimum": 1, "maximum": 254, "default": 1},
       "window": {"type": "number", "default": 0.05,
                  "description": "Arbitration collection window per UID byte [s]."},
       "save": {"type": "boolean", "default": False,
                "description": "Persist each assigned address to flash."}})
def t_scan(first_id=1, window=0.05, save=False):
    args = _Args(first_id=first_id, window=window, save=save)
    with DRIVE.lock:
        tp.cmd_scan(DRIVE.ensure(), args)
    return {"ok": True}


# ============================================================================
# Tools: status and registers
# ============================================================================

# The chunks worth reading on every poll: what the drive is doing, whether it
# is faulted, where the shaft is, and whether a trajectory is draining.
DEFAULT_CHUNKS = ["device_status", "motor_status", "electric_status", "motion1",
                  "motion3", "queue_stat"]

# Everything the drive will report, in the order tpcp_host's `info` asks for it.
ALL_CHUNKS = ["device_info", "device_uid", "device_status", "motor_status", "electric_status",
              "motion1", "motion2", "motion3", "motor_props", "encoder", "driver_status",
              "calib_status", "comm_status", "power_status", "queue_stat", "param_status",
              "calib_result"]


@tool("read_status",
      "Read decoded status chunks from the drive. The default set covers state, faults, mode, "
      "position, following error and the trajectory queue. Pass chunks=['all'] for everything, "
      "or name the ones you want. Read-only.",
      {"node": NODE_ARG,
       "chunks": {"type": "array", "items": {"type": "string"},
                  "description": "Chunk names, or ['all']. Available: " + ", ".join(ALL_CHUNKS)}},
      mutating=False)
def t_read_status(node=None, chunks=None):
    n = DRIVE.target(node)
    names = DEFAULT_CHUNKS if not chunks else (ALL_CHUNKS if "all" in chunks else list(chunks))
    bad = [c for c in names if c not in tp.OP]
    if bad:
        raise McpError(f"unknown chunk(s) {bad}. Available: {', '.join(ALL_CHUNKS)}")
    out = {"node": n}
    for name in names:
        out[name] = DRIVE.chunk(n, name) or {"error": "no response"}
    return out


@tool("list_registers",
      "List the drive's register map: name, id, type and which group it belongs to. Use it to "
      "find the right name for read_register / write_register. Read-only, no drive access.",
      {"filter": {"type": "string",
                  "description": "Substring match on the register name, e.g. 'limit' or 'tone'."}},
      mutating=False)
def t_list_registers(filter=None):  # noqa: A002 - the schema name is the useful one
    groups = {0x00: "device & link", 0x01: "motor", 0x02: "limits & protection",
              0x03: "control", 0x04: "motion", 0x05: "telemetry (mostly read-only)"}
    out = []
    for name, (rid, rtype) in sorted(tp.REGS.items(), key=lambda kv: kv[1][0]):
        if filter and filter.lower() not in name.lower():
            continue
        out.append({"name": name, "id": f"0x{rid:04X}", "type": rtype,
                    "group": groups.get(rid >> 8, "other")})
    return {"count": len(out), "registers": out}


@tool("read_register",
      "Read one or more registers by name. Read-only.",
      {"node": NODE_ARG,
       "names": {"type": "array", "items": {"type": "string"},
                 "description": "Register names. See list_registers."}},
      required=["names"], mutating=False)
def t_read_register(names, node=None):
    n = DRIVE.target(node)
    values, errors = {}, {}
    for name in names:
        if name not in tp.REGS:
            errors[name] = "unknown register"
            continue
        rid, rtype = tp.REGS[name]
        payload = bytes([tp.OP["reg_read"], 0]) + struct.pack("<H", rid) + b"\x00" * 4
        reply = DRIVE.request(n, payload, timeout=0.4)
        if reply is None:
            errors[name] = "no response"
        elif reply[0] == tp.OP["nack"]:
            errors[name] = tp.ERRORS.get(reply[2], f"0x{reply[2]:02X}")
        else:
            values[name] = tp.decode_reg(rtype, struct.unpack("<I", reply[4:8])[0])
    out = {"node": n, "values": values}
    if errors:
        out["errors"] = errors
    return out


@tool("write_register",
      "Write one register and report the value the drive holds afterwards. Changing a limit or a "
      "control gain takes effect immediately; it is NOT persisted until you call persist.",
      {"node": NODE_ARG,
       "name": {"type": "string", "description": "Register name. See list_registers."},
       "value": {"type": ["number", "string"],
                 "description": "Value in the register's own units. Integers accept 0x notation."}},
      required=["name", "value"])
def t_write_register(name, value, node=None):
    if name not in tp.REGS:
        raise McpError(f"unknown register '{name}'. Call list_registers to see them all.")
    n = DRIVE.target(node)
    rid, rtype = tp.REGS[name]
    # encode_reg parses a command-line string; give it one of the right shape
    # for the register's type so "1.0" into a u8 does not raise instead of
    # rounding, and "0x10" into a u8 still works.
    raw = _encode(rtype, value)
    payload = bytes([tp.OP["reg_write"], 0]) + struct.pack("<HI", rid, raw)
    reply = DRIVE.request(n, payload)
    if reply is None:
        return {"ok": False, "node": n, "register": name, "error": "no response"}
    if reply[0] == tp.OP["nack"]:
        return {"ok": False, "node": n, "register": name,
                "error": tp.ERRORS.get(reply[2], f"0x{reply[2]:02X}"), "detail": reply[3]}
    return {"ok": True, "node": n, "register": name, "result": decode_frame(reply)}


# ============================================================================
# Tools: control
# ============================================================================


def _encode(rtype, value):
    """Turn a JSON value into the 32-bit payload a register write carries.

    A model has no reason to know that a pole count is an integer and a current
    limit is a float, and JSON blurs the two anyway - 2 and 2.0 are the same
    number to it. So an integer register accepts anything that names a whole
    number, in decimal or 0x form, and only a genuinely fractional value for an
    integer field is refused. Negative values wrap to two's complement, which is
    exactly how the signed types are encoded on the wire.
    """
    if rtype in ("f32", "q16"):
        return tp.encode_reg(rtype, str(value))
    try:
        return int(str(value), 0) & 0xFFFFFFFF
    except ValueError:
        pass
    number = float(value)
    if number != int(number):
        raise McpError(f"a {rtype} register cannot hold {value!r}")
    return int(number) & 0xFFFFFFFF


@tool("set_mode",
      "Select the control mode, optionally arming the output stage at the same time. "
      "disabled/torque/velocity/position/motion/tone. Note the trajectory queue only advances in "
      "motion mode, and a queued move in any other mode simply sits there.",
      {"node": NODE_ARG,
       "mode": {"type": "string", "enum": tp.MODES},
       "enable": {"type": "boolean", "default": False,
                  "description": "Also energize the output stage."}},
      required=["mode"])
def t_set_mode(mode, node=None, enable=False):
    if mode not in tp.MODES:
        raise McpError(f"unknown mode '{mode}'. One of: {', '.join(tp.MODES)}")
    n = DRIVE.target(node)
    payload = bytes([tp.OP["control_mode"], tp.MODES.index(mode), 1 if enable else 0, 0, 0, 0, 0, 0])
    return DRIVE.command(n, payload, f"mode {mode}{' +enable' if enable else ''}")


@tool("enable",
      "Arm or disarm the output stage without changing the control mode. Disarming freewheels: "
      "the bridge goes high-impedance and the shaft is free.",
      {"node": NODE_ARG, "on": {"type": "boolean"}},
      required=["on"])
def t_enable(on, node=None):
    n = DRIVE.target(node)
    return DRIVE.command(n, bytes([tp.OP["sys_enable"], 1 if on else 0, 0, 0, 0, 0, 0, 0]),
                         "enable on" if on else "enable off")


@tool("stop",
      "Stop the motor with a chosen end state. hold keeps position, freewheel releases the bridge, "
      "brake shorts the windings, quick decelerates at the limit.",
      {"node": NODE_ARG,
       "style": {"type": "string", "enum": ["hold", "freewheel", "brake", "quick"],
                 "default": "hold"}})
def t_stop(node=None, style="hold"):
    n = DRIVE.target(node)
    code = {"hold": 0, "freewheel": 1, "brake": 2, "quick": 3}[style]
    return DRIVE.command(n, bytes([tp.OP["stop"], code, 0, 0, 0, 0, 0, 0]), f"stop {style}")


@tool("estop",
      "Broadcast an emergency stop to every node on the link. Use this first if anything is "
      "moving that should not be.")
def t_estop():
    with DRIVE.lock:
        DRIVE.ensure().send(tp.NMT_GLOBAL_CONTROL, b"\x00" * 8)
    return {"ok": True, "command": "emergency stop broadcast"}


@tool("move",
      "MOVES THE SHAFT. Command a position target in revolutions. Returns as soon as the drive "
      "accepts the target unless wait is set, because accepting a target is what makes streaming "
      "setpoints possible - arrival is a separate question.",
      {"node": NODE_ARG,
       "revolutions": {"type": "number", "description": "Target position [rev]."},
       "relative": {"type": "boolean", "default": False,
                    "description": "Treat the target as an offset from where the shaft is now."},
       "set_mode": {"type": "boolean", "default": False,
                    "description": "Switch to position mode and enable as part of the command."},
       "wait": {"type": "boolean", "default": False,
                "description": "Poll the following error until the shaft has arrived."},
       "tolerance": {"type": "number", "default": 0.01,
                     "description": "Arrival window on the following error [rev]."},
       "timeout": {"type": "number", "default": 30.0, "description": "Give up waiting after [s]."}},
      required=["revolutions"])
def t_move(revolutions, node=None, relative=False, set_mode=False, wait=False,
           tolerance=0.01, timeout=30.0):
    n = DRIVE.target(node)
    flags = (0x01 if relative else 0x00) | (0x02 if set_mode else 0x00)
    payload = bytes([tp.OP["target_position"], flags]) + tp.rev_to_i48(revolutions)
    result = DRIVE.command(n, payload, f"move {revolutions:+g}rev")
    if not result["ok"] or not wait:
        return result
    with DRIVE.lock:
        settled = tp.wait_settled(DRIVE.ensure(), n, tolerance, timeout)
    result["settled"] = settled
    result["motion3"] = DRIVE.chunk(n, "motion3")
    return result


@tool("jog",
      "MOVES THE SHAFT. Command a continuous velocity target in revolutions per second. The shaft "
      "keeps turning until stopped, so pair it with stop.",
      {"node": NODE_ARG,
       "rps": {"type": "number", "description": "Target velocity [rev/s]. 0 stops turning."},
       "set_mode": {"type": "boolean", "default": False,
                    "description": "Switch to velocity mode and enable as part of the command."}},
      required=["rps"])
def t_jog(rps, node=None, set_mode=False):
    n = DRIVE.target(node)
    v = tp.float_to_f24(rps)
    payload = (bytes([tp.OP["target_velocity"], v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF]) +
               struct.pack("<H", 0) + bytes([0x02 if set_mode else 0x00, 0]))
    return DRIVE.command(n, payload, f"jog {rps:+g} RPS")


@tool("pvt_move",
      "MOVES THE SHAFT. Run a profiled move through the trajectory queue: the leg is cut into "
      "segments streamed as position/velocity/time frames, so the shape of the motion comes from "
      "the numbers rather than from host timing. Set back or cycles>1 to return to the start. "
      "Delivery is paced against the drive's free queue slots.",
      {"node": NODE_ARG,
       "revolutions": {"type": "number", "description": "Displacement per leg [rev]."},
       "seconds": {"type": "number", "description": "Duration of one leg [s]."},
       "segments": {"type": "integer", "default": 20, "minimum": 1, "maximum": 2000,
                    "description": "Segments per leg. Finer is smoother but must be refilled "
                                   "faster than it plays."},
       "cycles": {"type": "integer", "default": 1, "minimum": 1, "maximum": 100},
       "back": {"type": "boolean", "default": False,
                "description": "Follow each leg with its mirror image."},
       "set_mode": {"type": "boolean", "default": True,
                    "description": "Put the drive in motion mode and enable it first. Without "
                                   "this the segments queue and never run."},
       "wait": {"type": "boolean", "default": False,
                "description": "Wait for each leg to settle before starting the next."},
       "tolerance": {"type": "number", "default": 0.01}},
      required=["revolutions", "seconds"])
def t_pvt_move(revolutions, seconds, node=None, segments=20, cycles=1, back=False,
               set_mode=True, wait=False, tolerance=0.01):
    n = DRIVE.target(node)
    args = _Args(node=n, revolutions=revolutions, seconds=seconds, segments=segments,
                 cycles=cycles, back=back, set_mode=set_mode, wait=wait, tolerance=tolerance)
    with DRIVE.lock:
        rc = tp.cmd_pvt(DRIVE.ensure(), args)
    return {"ok": rc == 0, "node": n, "queue": DRIVE.chunk(n, "queue_stat"),
            "motion1": DRIVE.chunk(n, "motion1")}


@tool("queue",
      "Inspect or control the trajectory queue: status, clear, pause, resume, or scale playback "
      "speed. Clearing discards queued segments; the shaft stops where it is.",
      {"node": NODE_ARG,
       "action": {"type": "string", "enum": ["status", "clear", "pause", "resume", "scale"],
                  "default": "status"},
       "scale": {"type": "number", "default": 1.0,
                 "description": "Playback rate for action='scale'. 1.0 is real time."}})
def t_queue(node=None, action="status", scale=1.0):
    n = DRIVE.target(node)
    if action == "status":
        return {"node": n, "queue": DRIVE.chunk(n, "queue_stat") or {"error": "no response"}}
    code = {"clear": 1, "pause": 2, "resume": 3, "scale": 4}[action]
    raw = int(round(scale * 1024)) if action == "scale" else 0
    payload = bytes([tp.OP["queue_ctl"], code]) + struct.pack("<hI", raw, 0)[:6]
    result = DRIVE.command(n, payload, f"queue {action}")
    result["queue"] = DRIVE.chunk(n, "queue_stat")
    return result


# ============================================================================
# Tools: calibration, persistence, faults
# ============================================================================

CALIB_KINDS = {"offset": 0, "electrical": 1, "angle": 2}


def _follow_calibration(node, kind, timeout):
    """Poll until a calibration routine finishes, and say how it went.

    The two routines have to be followed by different signals, and getting this
    wrong is what makes a working drive look broken.

    Electrical identification drives the stage/progress pair. An angle sweep
    does not: stage and progress keep reporting whatever the last electrical run
    left behind - typically "success, 100%" - for the whole time the shaft is
    turning. What an angle sweep does move is the flag byte, so it is followed
    there: the table is invalidated (calibrated clears), the shaft sweeps, the
    result settles and is persisted, and the routine is done when calibrated
    comes back with nothing being written.

    Both are watched for the routine STARTING before they are watched for it
    finishing. Without that the status left over from the previous run - idle,
    success, 100%, calibrated - satisfies the completion test on the very first
    poll and the tool reports success while the shaft is still turning.
    """
    deadline = time.time() + timeout
    started = False
    last = None
    while time.time() < deadline:
        time.sleep(0.25)
        status = DRIVE.chunk(node, "calib_status")
        if status is None:
            continue
        last = status
        if status["failed"]:
            return {"ok": False, "finished": True, "status": status,
                    "error": f"calibration failed in stage '{status['stage_name']}'",
                    "hint": "the reason is printed on the drive's CDC console; reconnect with "
                            "interface='serial' to read it"}
        if kind == CALIB_KINDS["angle"]:
            # The flag byte is the only thing an angle sweep moves.
            running = (not status["calibrated"]) or status["settling"] or status["persisting"]
        else:
            # kind returns to 0xFF (idle) when the routine is no longer loaded.
            running = (status["kind"] == kind) or (status["stage_name"] == "calibrating")
        if running:
            started = True
        elif started:
            return {"ok": status["stage_name"] == "success", "finished": True, "status": status}

    if not started:
        return {"ok": True, "finished": True, "status": last,
                "warning": "the routine never reported itself running and the shaft may never "
                           "have turned. A drive with a stored motor profile reloads it instead "
                           "of sweeping - retry with force=true."}
    return {"ok": False, "finished": False, "status": last,
            "error": f"still running after {timeout:g}s"}


@tool("calibrate",
      "TURNS THE SHAFT (kind 'angle' and 'electrical'). Start a calibration routine. "
      "'offset' measures current-sense offsets and does not move. 'electrical' identifies "
      "resistance and inductance by injecting current. 'angle' sweeps a full turn to build the "
      "encoder table. A drive with a stored motor profile reloads it instead of sweeping unless "
      "force is set.",
      {"node": NODE_ARG,
       "kind": {"type": "string", "enum": list(CALIB_KINDS), "default": "angle"},
       "force": {"type": "boolean", "default": False,
                 "description": "Force a physical sweep even when a table is already stored."},
       "wait": {"type": "boolean", "default": True,
                "description": "Poll to completion and report the outcome."},
       "timeout": {"type": "number", "default": 90.0}})
def t_calibrate(node=None, kind="angle", force=False, wait=True, timeout=90.0):
    if kind not in CALIB_KINDS:
        raise McpError(f"unknown calibration kind '{kind}'. One of: {', '.join(CALIB_KINDS)}")
    n = DRIVE.target(node)
    code = CALIB_KINDS[kind]
    # Byte 2 is the flags byte; bit 1 forces a physical sweep.
    payload = bytes([tp.OP["calib_start"], code, 0x02 if force else 0x00, 0, 0, 0, 0, 0])
    result = DRIVE.command(n, payload, f"calibrate {kind}{' --force' if force else ''}")
    if not result["ok"]:
        return result
    if not wait:
        result["status"] = DRIVE.chunk(n, "calib_status")
        result["note"] = "started; poll read_status(chunks=['calib_status']) for progress"
        return result
    result.update(_follow_calibration(n, code, timeout))
    result["result"] = DRIVE.chunk(n, "calib_result")
    return result


@tool("persist",
      "Save the configuration to flash, reload it, or restore defaults. Saving requires the drive "
      "to be disabled. target='all' with action='defaults' ERASES the stored record, which loses "
      "the node address and needs a discovery sweep afterwards, so it needs confirm=true.",
      {"node": NODE_ARG,
       "action": {"type": "string", "enum": ["save", "load", "defaults"], "default": "save"},
       "target": {"type": "string", "enum": ["all", "config", "motor"], "default": "config"},
       "confirm": {"type": "boolean", "default": False,
                   "description": "Required for action='defaults' with target='all'."}})
def t_persist(node=None, action="save", target="config", confirm=False):
    n = DRIVE.target(node)
    if action == "defaults":
        if target == "motor":
            raise McpError("action='defaults' accepts target 'all' or 'config'")
        code = {"all": 0, "config": 1}[target]
        if code == 0 and not confirm:
            raise McpError("restoring ALL defaults erases the stored configuration: the node comes "
                           "back unaddressed and needs a discovery sweep. Pass confirm=true if "
                           "that is what you want.")
        payload = bytes([tp.OP["param_defaults"], code, 0, 0]) + struct.pack("<I", tp.DEFAULTS_KEY)
        return DRIVE.command(n, payload, f"defaults {target}")
    code = {"all": 0, "config": 1, "motor": 2}[target]
    op = tp.OP["param_save"] if action == "save" else tp.OP["param_load"]
    result = DRIVE.command(n, bytes([op, code, 0, 0, 0, 0, 0, 0]), f"{action} {target}")
    result["param_status"] = DRIVE.chunk(n, "param_status")
    return result


@tool("fault_reset", "Clear latched faults. The condition that caused them must be gone first.",
      {"node": NODE_ARG})
def t_fault_reset(node=None):
    n = DRIVE.target(node)
    result = DRIVE.command(n, bytes([tp.OP["fault_reset"], 0, 0, 0, 0, 0, 0, 0]), "fault reset")
    result["status"] = DRIVE.chunk(n, "device_status")
    return result


@tool("reboot",
      "Reboot the drive, optionally into the USB DFU bootloader. Rebooting into DFU drops the "
      "protocol link until the board is reflashed or power-cycled, so it needs confirm=true.",
      {"node": NODE_ARG,
       "dfu": {"type": "boolean", "default": False},
       "confirm": {"type": "boolean", "default": False}})
def t_reboot(node=None, dfu=False, confirm=False):
    if dfu and not confirm:
        raise McpError("rebooting into DFU leaves no protocol link. Pass confirm=true to proceed.")
    n = DRIVE.target(node)
    payload = bytes([tp.OP["reboot"], 1 if dfu else 0, 0, 0]) + struct.pack("<I", tp.REBOOT_KEY)
    result = DRIVE.command(n, payload, "reboot into DFU" if dfu else "reboot")
    # The device re-enumerates, so the handle we hold is stale either way.
    DRIVE.close()
    result["link"] = "closed - the drive is re-enumerating; call connect again"
    return result


@tool("watchdog",
      "Configure the master-liveness watchdog. With it armed the drive stops on its own if the "
      "master goes quiet, which is what makes an unattended session safe. timeout_ms=0 disables it.",
      {"node": NODE_ARG,
       "timeout_ms": {"type": "integer", "minimum": 0, "maximum": 65535},
       "action": {"type": "string", "enum": ["freewheel", "brake", "hold"], "default": "hold"}},
      required=["timeout_ms"])
def t_watchdog(timeout_ms, node=None, action="hold"):
    n = DRIVE.target(node)
    code = {"freewheel": 0, "brake": 1, "hold": 2}[action]
    flags = 0x80 | (0x01 if timeout_ms else 0x00)
    payload = bytes([tp.OP["watchdog"], code, flags, 0]) + struct.pack("<HH", timeout_ms, 0)
    return DRIVE.command(n, payload, f"watchdog {timeout_ms}ms {action}")


# ============================================================================
# Tools: motor speaker
# ============================================================================


@tool("tone",
      "MAKES THE MOTOR SING. In tone mode the control ISR synthesizes a sine straight onto the "
      "alpha-beta voltage vector, bypassing the current loop, so the windings act as a "
      "loudspeaker. hz=0 silences the drive and returns it to disabled. The shaft holds rather "
      "than spins, but it is energized.",
      {"node": NODE_ARG,
       "hz": {"type": "integer", "minimum": 0, "maximum": 20000,
              "description": f"Frequency. Useful range is about {tp.TONE_MIN_HZ}-{tp.TONE_MAX_HZ} Hz; "
                             f"0 stops and disables."},
       "amp": {"type": "integer", "minimum": 0, "maximum": 8192, "default": 2000,
               "description": "Amplitude as a fraction of the bus voltage, out of 32768. "
                              "The drive rejects more than 8192."}},
      required=["hz"])
def t_tone(hz, node=None, amp=2000):
    n = DRIVE.target(node)
    DRIVE.stop_playback.set()                       # A manual tone overrides playback
    with DRIVE.lock:
        link = DRIVE.ensure()
        if hz == 0:
            tp.silence(link, n)
            return {"ok": True, "node": n, "tone": "off"}
        if not tp.write_reg(link, n, "tone_amp", amp):
            return {"ok": False, "node": n, "error": "the drive rejected the amplitude (max 8192)"}
        if not tp.write_reg(link, n, "control_mode", tp.MODES.index("tone")):
            return {"ok": False, "node": n,
                    "error": "the drive rejected tone mode - is the firmware new enough?"}
        tp.write_reg(link, n, "enable", 1)
        ok = tp.write_reg(link, n, "tone_freq", hz)
    return {"ok": ok, "node": n, "tone": "on", "hz": hz, "amp": amp}


def _play(node, schedule, amp):
    """Play a parsed schedule of (time, frequency) pairs on the motor coils.

    Runs on its own thread so a thirty-second clip does not hold a tool call
    open for thirty seconds. Each entry costs one register write, taken under
    the shared lock, so status reads still interleave while it plays.
    """
    with DRIVE.lock:
        link = DRIVE.ensure()
        tp.write_reg(link, node, "tone_amp", amp)
        tp.write_reg(link, node, "control_mode", tp.MODES.index("tone"))
        tp.write_reg(link, node, "enable", 1)
    t0 = time.time()
    try:
        for t, freq in schedule:
            delay = t0 + t - time.time()
            if delay > 0 and DRIVE.stop_playback.wait(delay):
                break                               # Asked to stop mid-clip
            if DRIVE.stop_playback.is_set():
                break
            with DRIVE.lock:
                tp.write_reg(DRIVE.ensure(), node, "tone_freq", freq)
    finally:
        # Whatever happened, the coils must not be left singing.
        with DRIVE.lock:
            if DRIVE.link is not None:
                tp.silence(DRIVE.link, node)


@tool("play_midi",
      "MAKES THE MOTOR SING. Play a window of a MIDI file through the motor coils. A motor is "
      "monophonic, so the melody is reduced to one voice: the most monophonic channel in the file, "
      "highest note sounding at each instant. Plays in the background and returns immediately; "
      "call stop_midi or tone(hz=0) to stop it early.",
      {"node": NODE_ARG,
       "file": {"type": "string", "description": "Path to a Standard MIDI File."},
       "start": {"type": "number", "default": 0.0, "description": "Window start [s]."},
       "end": {"type": "number", "description": "Window end [s]. Defaults to start + seconds."},
       "seconds": {"type": "number", "default": 30.0,
                   "description": "Window length when end is not given."},
       "amp": {"type": "integer", "minimum": 0, "maximum": 8192, "default": 2000},
       "voice": {"type": "integer", "minimum": 0, "maximum": 15,
                 "description": "Force a MIDI channel instead of choosing the most monophonic one."},
       "blocking": {"type": "boolean", "default": False,
                    "description": "Wait for the clip to finish instead of returning at once."}},
      required=["file"])
def t_play_midi(file, node=None, start=0.0, end=None, seconds=30.0, amp=2000,
                voice=None, blocking=False):
    path = os.path.expanduser(file)
    if not os.path.isfile(path):
        raise McpError(f"no such file: {path}")
    stop = end if end is not None else start + seconds
    if stop <= start:
        raise McpError(f"end ({stop:g}s) must be after start ({start:g}s)")
    if DRIVE.player is not None and DRIVE.player.is_alive():
        raise McpError("a clip is already playing; call stop_midi first")

    schedule, chosen = tp.parse_midi_melody(path, start, stop, voice)
    notes = sum(1 for _, f in schedule if f)
    if notes == 0:
        return {"ok": False, "file": os.path.basename(path),
                "error": f"no notes between {start:g}s and {stop:g}s on channel {chosen}",
                "hint": "try a different window, or name a channel with 'voice'"}

    DRIVE.stop_playback.clear()
    DRIVE.player = threading.Thread(target=_play, args=(DRIVE.target(node), schedule, amp),
                                    daemon=True)
    DRIVE.player.start()
    out = {"ok": True, "file": os.path.basename(path), "node": DRIVE.target(node),
           "notes": notes, "window_s": [start, stop], "channel": chosen, "amp": amp,
           "events": len(schedule)}
    if blocking:
        DRIVE.player.join(timeout=(stop - start) + 10.0)
        out["finished"] = not DRIVE.player.is_alive()
    else:
        out["note"] = f"playing in the background for about {stop - start:g}s"
    return out


@tool("stop_midi", "Stop playback, silence the coils and disable the output stage.",
      {"node": NODE_ARG})
def t_stop_midi(node=None):
    n = DRIVE.target(node)
    DRIVE.stop_playback.set()
    player = DRIVE.player
    if player is not None and player.is_alive():
        player.join(timeout=2.0)
    with DRIVE.lock:
        tp.silence(DRIVE.ensure(), n)
    return {"ok": True, "node": n, "tone": "off"}


# ============================================================================
# Tools: diagnostics
# ============================================================================


@tool("telemetry_config",
      "Configure one cyclic telemetry slot. The drive then emits that chunk unprompted at the "
      "given period, which is what 'monitor' listens for. period_ms=0 disables the slot.",
      {"node": NODE_ARG,
       "slot": {"type": "integer", "minimum": 0, "maximum": 7},
       "chunk": {"type": "string", "description": "Status chunk name, e.g. motion1 or heartbeat."},
       "period_ms": {"type": "integer", "minimum": 0, "maximum": 65535}},
      required=["slot", "chunk", "period_ms"])
def t_telemetry_config(slot, chunk, period_ms, node=None):
    op = tp.OP.get(chunk)
    if op is None or op > tp.OP["heartbeat"]:
        raise McpError(f"'{chunk}' is not a telemetry chunk")
    n = DRIVE.target(node)
    flags = 0x80 | (0x01 if period_ms else 0x00)    # Bit 7 applies, bit 0 enables
    payload = bytes([tp.OP["telemetry"], slot, op if period_ms else 0, flags]) + \
        struct.pack("<HH", period_ms, 0)
    return DRIVE.command(n, payload, f"telemetry slot {slot} = {chunk} @ {period_ms}ms")


@tool("monitor",
      "Listen for a fixed window and decode every frame that arrives - telemetry, emergencies, "
      "anything another master is sending. Nothing is transmitted. Configure a telemetry slot "
      "first, or there may be nothing to hear.",
      {"node": NODE_ARG,
       "seconds": {"type": "number", "default": 2.0, "minimum": 0.1, "maximum": 30.0},
       "all_nodes": {"type": "boolean", "default": False,
                     "description": "Include frames addressed to other nodes."}},
      mutating=False)
def t_monitor(node=None, seconds=2.0, all_nodes=False):
    n = DRIVE.target(node)
    with DRIVE.lock:
        messages = DRIVE.ensure().drain(seconds)
    frames = []
    for msg in messages:
        cid = msg.arbitration_id
        kind, src = _classify(cid)
        if not all_nodes and src is not None and src != n and kind != "nmt":
            continue
        frames.append({"kind": kind, "node": src,
                       "frame": decode_frame(bytes(msg.data).ljust(8, b"\x00"))})
    return {"node": n, "window_s": seconds, "count": len(frames), "frames": frames,
            "note": None if frames else "nothing arrived: telemetry may not be configured"}


def _classify(cid):
    """Map an identifier onto its channel and source node."""
    if cid < tp.ID_NMT:
        return "emergency", cid
    if cid < tp.ID_SETPOINT:
        return "nmt", None
    if cid < tp.ID_REQUEST:
        return "setpoint", cid - tp.ID_SETPOINT
    if cid < tp.ID_RESPONSE:
        return "request", cid - tp.ID_REQUEST
    if cid < tp.ID_DISCOVERY:
        return "response", cid - tp.ID_RESPONSE
    if cid < tp.ID_TELEMETRY:
        return "discovery", None
    return "telemetry", cid - tp.ID_TELEMETRY


@tool("raw",
      "Send an arbitrary eight-byte request frame and decode the answer. The escape hatch for "
      "anything the typed tools do not cover; see CAN_PROTOCOL.md for the layouts.",
      {"node": NODE_ARG,
       "hex": {"type": "string",
               "description": "Payload bytes as hex, e.g. '01' or '20 00 00 05'. Padded to 8."}},
      required=["hex"])
def t_raw(hex, node=None):  # noqa: A002 - matches the CLI's argument name
    n = DRIVE.target(node)
    try:
        payload = bytes.fromhex(hex.replace(" ", "")).ljust(8, b"\x00")[:8]
    except ValueError as exc:
        raise McpError(f"not valid hex: {exc}") from exc
    reply = DRIVE.request(n, payload)
    return {"node": n, "sent": payload.hex(" "),
            "reply": decode_frame(reply) if reply else None,
            "ok": reply is not None and reply[0] != tp.OP["nack"]}


# ============================================================================
# Resources
# ============================================================================


@resource("tpcp://registers", "TPCP register map",
          "Every register the drive exposes: name, id, type and group.", "application/json")
def r_registers():
    return json.dumps(t_list_registers(), indent=2)


@resource("tpcp://protocol", "TunePulse CAN Protocol specification",
          "The full wire specification: identifier map, opcodes, chunk layouts, registers, "
          "worked examples.", "text/markdown")
def r_protocol():
    path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "src", "api", "CAN_PROTOCOL.md")
    try:
        with open(path, encoding="utf-8") as fh:
            return fh.read()
    except OSError as exc:
        return f"CAN_PROTOCOL.md is not readable from this checkout ({exc})."


INSTRUCTIONS = """\
This server talks to a TunePulse motor drive (STM32G431, CLN17 v3) over the TunePulse CAN
Protocol, carried on USB - the drive's vendor HID interface when it has one, its CDC console
otherwise.

The link opens on first use; call connect only to pin a transport, a port or the default node.

Reading is free: read_status, read_register, list_registers and monitor never change anything.

Anything that moves a real shaft says so in capitals at the start of its description - move, jog,
pvt_move, calibrate, and the tone tools, which energize the windings. Check read_status first,
keep estop and stop in mind, and prefer small displacements until you know the mechanics are free
to turn.

Two things about this drive surprise people:

  * The trajectory queue only advances in MOTION mode with the output stage enabled. Segments
    pushed in any other mode are accepted, queued, and never run - which looks exactly like a
    mechanical failure.

  * In a calibration status chunk, stage and progress describe the ELECTRICAL identification
    routine only. During an angle sweep they keep reporting whatever the last electrical run
    left behind. Follow an angle sweep through the calibrated / settling / persisting flags
    instead; the calibrate tool already does.
"""


# ============================================================================
# Entry point
# ============================================================================


def main():
    import argparse
    p = argparse.ArgumentParser(
        description="MCP server exposing a TunePulse drive over stdio.",
        epilog="Register it with:  claude mcp add tunepulse -- "
               f"{sys.executable} {os.path.abspath(__file__)}")
    p.add_argument("--read-only", action="store_true",
                   help="withhold every tool that could move the shaft or write flash")
    p.add_argument("--interface", default="auto",
                   help="transport to open by default: auto, hid, serial, or a python-can name")
    p.add_argument("--channel", default=os.environ.get("TPCP_CHANNEL"),
                   help="serial device or adapter channel; omit to auto-detect")
    p.add_argument("--node", type=int, default=1, help="default node address (default: 1)")
    p.add_argument("--connect", action="store_true",
                   help="open the link at startup instead of on first use, and fail loudly if "
                        "there is no drive")
    args = p.parse_args()

    DRIVE.node = args.node
    DRIVE.pref_interface = args.interface
    DRIVE.pref_channel = args.channel

    if args.connect:
        try:
            DRIVE.open(args.interface, args.channel, args.node)
            print(f"tpcp-mcp: connected over {DRIVE.interface} ({DRIVE.channel})", file=sys.stderr)
        except Exception as exc:  # noqa: BLE001 - report and keep serving
            print(f"tpcp-mcp: could not connect: {exc}", file=sys.stderr)

    surface = len([t for t in TOOLS.values() if not (args.read_only and t.mutating)])
    print(f"tpcp-mcp: serving {surface} tool(s)"
          f"{' (read-only)' if args.read_only else ''} on stdio", file=sys.stderr)
    Server(read_only=args.read_only).run()


if __name__ == "__main__":
    main()
