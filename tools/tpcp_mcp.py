#!/usr/bin/env python3
"""Model Context Protocol server for the TunePulse drive.

This exposes the same drive that tools/tpcp_host.py drives from a shell, but
as a set of MCP tools an assistant can call directly: connect, read status,
read and write registers (by name and field), select a control mode, move the
shaft, stream a profiled trajectory, calibrate, persist, and play a MIDI file
through the motor coils - all over TPCP v1 as specified in src/api/CAN_PROTOCOL.md.

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

    def chunk(self, node, key, timeout=0.5, selector=None):
        """Read one register by name (or 'group:record') and decode it."""
        reg = tp.REGISTERS[key]
        reply = self.request(node, reg.request(selector), timeout)
        if reply is None:
            return None
        return decode_frame(reply)

    def command(self, node, payload, what, timeout=0.5):
        """Send a request that answers ACK or NACK and report which.

        The NACK layout is the one place this protocol is easy to misread: byte
        1 echoes the CMD that was rejected and the error code is in byte 2.
        Reading byte 1 as the error turns every rejection into a nonsense
        diagnosis, so it is decoded once, here.
        """
        reply = self.request(node, payload, timeout)
        if reply is None:
            return {"ok": False, "command": what, "error": "no response",
                    "hint": "the drive did not answer - check the node address and the link, "
                            "and remember accepted setpoints are silent in comm mode 1"}
        if reply[0] == tp.CMD["nack"]:
            return {"ok": False, "command": what,
                    "rejected": tp.CMD_NAME.get(reply[1], f"0x{reply[1]:02X}"),
                    "error": tp.ERRORS.get(reply[2], f"0x{reply[2]:02X}"),
                    "detail": reply[3]}
        return {"ok": True, "command": what, "detail": reply[3] if reply[0] == tp.CMD["ack"] else None,
                "reply": decode_frame(reply)}

    def fire(self, node, payload, what):
        """Send a setpoint that is silent when accepted (comm mode 1) and report a NACK if one comes."""
        reply = self.request(node, payload, timeout=0.05)
        if reply is not None and reply[0] == tp.CMD["nack"]:
            return {"ok": False, "command": what,
                    "rejected": tp.CMD_NAME.get(reply[1], f"0x{reply[1]:02X}"),
                    "error": tp.ERRORS.get(reply[2], f"0x{reply[2]:02X}"), "detail": reply[3]}
        return {"ok": True, "command": what, "reply": decode_frame(reply) if reply else None}


DRIVE = Drive()



# ============================================================================
# Structured frame decoding
# ============================================================================
#
# tpcp_host.decode_frame renders a frame as a line of text for a human. A model
# needs the fields themselves, so the register model decodes them into a dict
# and the text form travels alongside - the two views of one frame always agree.


def decode_frame(d):
    """Decode one eight-byte frame into {"register", "text", ...fields}."""
    d = bytes(d).ljust(8, b"\x00")
    cmd = d[0]
    out = {"text": tp.decode_frame(d), "raw": d.hex(" ")}
    if cmd >= 0xE0:
        out["register"] = "burst"
        return out
    if 0x70 <= cmd <= 0x7F:
        qts = d[1] | (d[2] << 8)
        wme = d[4] | (d[5] << 8)
        out.update(register="pulsebeat", seq=cmd & 0xF, queued=qts & 0xF,
                   time_scale=tp.signext(qts >> 4, 12) / 256.0, state=tp.STATES[d[3] & 0xF],
                   fault_group=tp.FAULT_GROUPS.get(d[3] >> 4, d[3] >> 4), warn_group=wme & 0xF,
                   mode=tp.MODES[min((wme >> 4) & 0x3F, 5)], error=tp.signext(wme >> 10, 6) / 16.0,
                   load=tp.signext(d[6], 8), io=d[7])
        return out
    if cmd == tp.CMD["ack"]:
        out.update(register="ack", acked=tp.CMD_NAME.get(d[1], f"0x{d[1]:02X}"), detail=d[3])
        return out
    if cmd == tp.CMD["nack"]:
        out.update(register="nack", rejected=tp.CMD_NAME.get(d[1], f"0x{d[1]:02X}"),
                   error=tp.ERRORS.get(d[2], f"0x{d[2]:02X}"), detail=d[3])
        return out
    found = tp.register_for_frame(d)
    if found is None:
        out["register"] = f"0x{cmd:02X}"
        return out
    key, reg = found
    vals = reg.decode(d)
    for k, v in list(vals.items()):
        if isinstance(v, float) and v != v:
            vals[k] = None                                   # NaN: not available
    out["register"] = key
    out.update(vals)
    if key == "system_status":
        w = vals["fault_word"]
        out.update(faults=_bits(w & 0xFF, tp.FAULT_BITS), warnings=_bits((w >> 8) & 0xFF, tp.WARN_BITS),
                   state=tp.STATES[(w >> 16) & 0xF], mode=tp.MODES[min((w >> 20) & 0xF, 5)])
    elif key == "driver_status":
        out.update(faults=_bits(vals["fault"], tp.FAULT_BITS), warnings=_bits(vals["warn"], tp.WARN_BITS))
    elif key == "motor_status":
        out.update(state_name=tp.STATES[vals["state"]], mode_name=tp.MODES[min(vals["mode"], 5)],
                   type_name={0: "none", 1: "stepper", 2: "bldc", 3: "error"}[vals["type"]])
    elif key == "motion_status_1":
        out["time_scale"] = vals["time_scale"] / 256.0
    elif key == "diag:calibration":
        fl = vals["flags"]
        out.update(calibrated=bool(fl & 1), settling=bool(fl & 2), persisting=bool(fl & 4),
                   failed=bool(fl & 8), identified=bool(fl & 16), profile_loaded=bool(fl & 32),
                   running_names=[n for n, b in tp.CAL_BITS.items()
                                  if n in ("encoder", "offset", "phase", "resistance", "inductance") and vals["running"] & b],
                   stage_name=(SWEEP_STAGES.get(vals["stage"] & 0x7F, f"sweep stage {vals['stage'] & 0x7F}")
                               if vals["stage"] & 0x80 else
                               CALIB_STAGES.get(vals["stage"], f"stage {vals['stage']}")))
    elif key in ("discovery_assign", "discovery_search"):
        for k in ("uid", "prefix"):
            if k in out:
                out[k] = f"{out[k]:012X}"
    elif key == "system_control":
        out.update(enabled=vals["enable"] == tp.SYS_ENABLE,
                   calibration_valid=[n for n, b in tp.CAL_BITS.items()
                                      if n in ("encoder", "offset", "phase", "resistance", "inductance") and vals["calibration"] & b])
    return out


CALIB_STAGES = {0: "success", 1: "soft-warmup", 2: "settle", 3: "slow-rotation", 4: "resistance",
                5: "inductance", 6: "calibration-error", 7: "electrical-done", 8: "hfi-warmup",
                9: "hfi-d", 10: "hfi-q", 11: "hfi-d2", 12: "hfi-q2"}

# Angle-sweep CalStage values (bit 7 of the stage byte set), matching angle_calibrator.h.
SWEEP_STAGES = {0: "sweep-idle", 1: "sweep-setup", 2: "direction-test", 3: "forward-pass",
                4: "reverse-pass", 5: "checking-table", 6: "table-ready", 7: "sweep-error",
                8: "reading-stored-table", 9: "storing-table"}


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
      {"interface": {"type": "string", "enum": ["auto", "hid", "serial", "slcan", "pcan", "socketcan"],
                     "description": "Transport. 'auto' (default) picks HID when the board exposes it."},
       "channel": {"type": "string", "description": "Serial device or adapter channel. Omit to auto-detect."},
       "node": NODE_ARG},
      mutating=False)
def t_connect(interface="auto", channel=None, node=None):
    detail = DRIVE.open(interface, channel, node)
    info = DRIVE.chunk(DRIVE.node, "device_info")
    status = DRIVE.chunk(DRIVE.node, "system_status")
    out = {"connected": True, "interface": DRIVE.interface, "channel": DRIVE.channel,
           "default_node": DRIVE.node, "device": info, "status": status}
    out.update(detail)
    if info is None:
        out["warning"] = (f"the link is open but node {DRIVE.node} did not answer. Run 'scan' if the "
                          f"drive is unaddressed (it reports node 255), or set a different node.")
    return out


@tool("disconnect",
      "Close the link and release the USB device. A HID device can only be held by one process, "
      "so this is what frees it for tpcp_host.py or the desktop app.", mutating=False)
def t_disconnect():
    was = DRIVE.interface
    DRIVE.close()
    return {"connected": False, "released": was}


@tool("link_status", "Report whether a link is open, over what, and to which default node.", mutating=False)
def t_link_status():
    if DRIVE.link is None:
        return {"connected": False, "hint": "any tool will connect on its own; use 'connect' to pin a transport"}
    return {"connected": True, "interface": DRIVE.interface, "channel": DRIVE.channel,
            "default_node": DRIVE.node, "open_for_s": round(time.time() - DRIVE.opened_at, 1),
            "playing_midi": bool(DRIVE.player and DRIVE.player.is_alive())}


@tool("scan",
      "Find unaddressed drives by binary HashUID search and hand out node addresses. Needed only "
      "when a drive reports node 255, which answers discovery and nothing else.",
      {"first_id": {"type": "integer", "minimum": 1, "maximum": 254, "default": 1},
       "window": {"type": "number", "default": 0.03, "description": "Reply window per probe [s]."},
       "limit": {"type": "integer", "default": 32}})
def t_scan(first_id=1, window=0.03, limit=32):
    args = _Args(first_id=first_id, window=window, limit=limit)
    with DRIVE.lock:
        tp.cmd_scan(DRIVE.ensure(), args)
    return {"ok": True}


@tool("ping", "Inventory every addressed node on the link with one broadcast identity read.",
      {"window": {"type": "number", "default": 0.3}}, mutating=False)
def t_ping(window=0.3):
    with DRIVE.lock:
        link = DRIVE.ensure()
        link.send(tp.ID_COMMAND + tp.NODE_BROADCAST, bytes([tp.CMD["discovery_assign"]]) + b"\x00" * 7)
        seen = {}
        for m in link.drain(window):
            d = bytes(m.data).ljust(8, b"\x00")
            if (m.arbitration_id & 0x700) == tp.ID_RESPONSE and d[0] == tp.CMD["discovery_assign"]:
                seen[f"{int.from_bytes(d[2:8], 'little'):012X}"] = d[1]   # keyed by identity: unassigned drives share 255
    return {"nodes": [{"node": n, "uid": u} for u, n in sorted(seen.items(), key=lambda kv: (kv[1], kv[0]))]}


# ============================================================================
# Tools: status and registers
# ============================================================================

DEFAULT_REGISTERS = ["system_status", "motor_status", "driver_status", "motion_status_2",
                     "motion_status_1", "diag:queue"]
READ_ONLY_INFO = tp.INFO_REGISTERS


@tool("read_status",
      "Read decoded registers from the drive. The default set covers state, faults, mode, position, "
      "following error and the trajectory queue. Pass registers=['all'] for the whole info set, or "
      "name the ones you want ('group:record' for sub-spaces). Read-only.",
      {"node": NODE_ARG,
       "registers": {"type": "array", "items": {"type": "string"},
                     "description": "Register names, or ['all']. Available: " + ", ".join(READ_ONLY_INFO)}},
      mutating=False)
def t_read_status(node=None, registers=None):
    n = DRIVE.target(node)
    names = DEFAULT_REGISTERS if not registers else (READ_ONLY_INFO if "all" in registers else list(registers))
    bad = [c for c in names if c not in tp.REGISTERS]
    if bad:
        raise McpError(f"unknown register(s) {bad}. Available: {', '.join(sorted(tp.REGISTERS))}")
    out = {"node": n}
    for name in names:
        out[name] = DRIVE.chunk(n, name) or {"error": "no response"}
    return out


@tool("list_registers",
      "List the drive's register map: name, CMD, fields and whether it is writable. Use it to find "
      "the right names for read_register / write_register. Read-only, no drive access.",
      {"filter": {"type": "string", "description": "Substring match on the register name."}},
      mutating=False)
def t_list_registers(filter=None):  # noqa: A002
    out = []
    for name, reg in sorted(tp.REGISTERS.items(), key=lambda kv: (kv[1].cmd, kv[1].sub or 0)):
        if filter and filter.lower() not in name.lower():
            continue
        out.append({"name": name, "cmd": f"0x{reg.cmd:02X}", "sub": reg.sub, "writable": reg.writable,
                    "keyed_by": reg.keyed,
                    "fields": [{"name": f.name, "kind": f.kind, "read_only": f.ro} for f in reg.fields]})
    return {"count": len(out), "registers": out}


@tool("read_register",
      "Read one or more registers by name ('group:record' for sub-spaces). A keyed register (the IO "
      "map) takes its selector in 'selector', e.g. selector=3 for function 3. Read-only.",
      {"node": NODE_ARG,
       "names": {"type": "array", "items": {"type": "string"}},
       "selector": {"type": "integer", "description": "Record selector for keyed registers."}},
      required=["names"], mutating=False)
def t_read_register(names, node=None, selector=None):
    n = DRIVE.target(node)
    values, errors = {}, {}
    for name in names:
        if name not in tp.REGISTERS:
            errors[name] = "unknown register"
            continue
        reply = DRIVE.request(n, tp.REGISTERS[name].request(selector), timeout=0.4)
        if reply is None:
            errors[name] = "no response"
        elif reply[0] == tp.CMD["nack"]:
            errors[name] = tp.ERRORS.get(reply[2], f"0x{reply[2]:02X}")
        else:
            values[name] = decode_frame(reply)
    out = {"node": n, "values": values}
    if errors:
        out["errors"] = errors
    return out


@tool("write_register",
      "Write fields of one register (read-modify-write, so untouched fields keep their value) and "
      "report what the drive holds afterwards. Values are in the register's own units; give 'nan' "
      "to leave a float field alone. Takes effect immediately; NOT persisted until you call persist.",
      {"node": NODE_ARG,
       "name": {"type": "string", "description": "Register name, e.g. motor_props_1 or axis:led."},
       "fields": {"type": "object", "additionalProperties": {"type": ["number", "string"]},
                  "description": "field -> value."},
       "selector": {"type": "integer", "description": "Record selector for keyed registers."}},
      required=["name", "fields"])
def t_write_register(name, fields, node=None, selector=None):
    if name not in tp.REGISTERS:
        raise McpError(f"unknown register '{name}'. Call list_registers to see them all.")
    reg = tp.REGISTERS[name]
    if not reg.writable:
        raise McpError(f"'{name}' is read only")
    known = {f.name for f in reg.fields}
    bad = [f for f in fields if f not in known]
    if bad:
        raise McpError(f"'{name}' has no field(s) {bad}. Fields: {', '.join(sorted(known))}")
    n = DRIVE.target(node)
    with DRIVE.lock:
        reply = tp.write_register(DRIVE.ensure(), n, name, {k: str(v) for k, v in fields.items()}, key=selector)
    if reply is None:
        return {"ok": False, "node": n, "register": name, "error": "no response"}
    if reply[0] == tp.CMD["nack"]:
        return {"ok": False, "node": n, "register": name,
                "error": tp.ERRORS.get(reply[2], f"0x{reply[2]:02X}"), "detail": reply[3]}
    return {"ok": True, "node": n, "register": name, "result": decode_frame(reply)}


# ============================================================================
# Tools: mechanics (gearbox)
# ============================================================================


@tool("set_gearbox",
      "Configure the gear ratio between the motor shaft and the output shaft: "
      "the motor turns 'mul' turns for every 'div' turns of the output (mul "
      "may be negative to invert the direction). Every position, velocity, "
      "acceleration and jerk on the wire is then expressed in OUTPUT-shaft "
      "turns and converted internally, so 'move' targets are output-shaft "
      "revolutions. Backlash is stored but not yet compensated. Takes effect "
      "immediately; call persist to keep it across a reboot.",
      {"node": NODE_ARG,
       "mul": {"type": "integer", "minimum": -32768, "maximum": 32767,
               "description": "Motor turns per output turn (ratio numerator)."},
       "div": {"type": "integer", "minimum": 1, "maximum": 32767,
               "description": "Output turns per motor turn (ratio denominator)."},
       "backlash": {"type": "number", "minimum": 0,
                    "description": "Backlash [rev]; omit or 'nan' to keep the stored value."}},
      required=["mul", "div"])
def t_set_gearbox(mul, div, backlash=None, node=None):
    n = DRIVE.target(node)
    fields = {"ratio_mul": int(mul), "ratio_div": int(div)}
    if backlash is not None:
        fields["backlash"] = float(backlash)
    with DRIVE.lock:
        reply = tp.write_register(DRIVE.ensure(), n, "axis:gearbox", {k: str(v) for k, v in fields.items()})
    if reply is None:
        return {"ok": False, "node": n, "error": "no response"}
    if reply[0] == tp.CMD["nack"]:
        return {"ok": False, "node": n, "rejected": tp.CMD_NAME.get(reply[1], f"0x{reply[1]:02X}"),
                "error": tp.ERRORS.get(reply[2], f"0x{reply[2]:02X}"), "detail": reply[3]}
    return {"ok": True, "node": n, "gearbox": decode_frame(reply),
            "ratio_out_per_mot": int(mul) / int(div), "note": "not persisted until you call persist"}


@tool("get_gearbox",
      "Report the configured gear ratio (ratio_mul, ratio_div) and backlash, "
      "plus the resulting float ratio. Read-only.",
      {"node": NODE_ARG},
      mutating=False)
def t_get_gearbox(node=None):
    n = DRIVE.target(node)
    with DRIVE.lock:
        reply = tp.read_register(DRIVE.ensure(), n, "axis:gearbox")
    if reply is None:
        return {"ok": False, "node": n, "error": "no response"}
    if reply[0] == tp.CMD["nack"]:
        return {"ok": False, "node": n, "error": tp.ERRORS.get(reply[2], f"0x{reply[2]:02X}")}
    g = decode_frame(reply)
    g["ratio_out_per_mot"] = g.get("ratio_mul", 0) / max(g.get("ratio_div", 1), 1)
    return {"ok": True, "node": n, "gearbox": g}


# ============================================================================
# Tools: control
# ============================================================================


@tool("set_mode",
      "Select the control mode (disabled/torque/velocity/position/motion/tone), optionally arming "
      "the output stage. Note that setpoint frames select their own mode: move -> position, jog -> "
      "velocity, torque -> torque, queued segments and bursts -> motion.",
      {"node": NODE_ARG, "mode": {"type": "string", "enum": tp.MODES},
       "enable": {"type": "boolean", "default": False}},
      required=["mode"])
def t_set_mode(mode, node=None, enable=False):
    if mode not in tp.MODES:
        raise McpError(f"unknown mode '{mode}'. One of: {', '.join(tp.MODES)}")
    n = DRIVE.target(node)
    with DRIVE.lock:
        ok = tp.write_fields(DRIVE.ensure(), n, "motor_props_1", mode=tp.MODES.index(mode))
    out = {"ok": ok, "node": n, "mode": mode}
    if ok and enable:
        with DRIVE.lock:
            reply = tp.system_control(DRIVE.ensure(), n, enable=tp.SYS_ENABLE)
        out["enable"] = decode_frame(reply) if reply else "no response"
        out["ok"] = reply is not None and reply[0] != tp.CMD["nack"]
    return out


@tool("enable",
      "Arm or disarm the output stage without changing the control mode. Disarming freewheels.",
      {"node": NODE_ARG, "on": {"type": "boolean"}}, required=["on"])
def t_enable(on, node=None):
    n = DRIVE.target(node)
    with DRIVE.lock:
        reply = tp.system_control(DRIVE.ensure(), n, enable=tp.SYS_ENABLE if on else tp.SYS_DISABLE)
    if reply is None:
        return {"ok": False, "node": n, "error": "no response"}
    return {"ok": reply[0] != tp.CMD["nack"], "node": n, "result": decode_frame(reply)}


@tool("stop",
      "Stop the motor: hold keeps position, freewheel releases the bridge, brake shorts the windings, "
      "reset clears the trajectory queue and holds.",
      {"node": NODE_ARG, "style": {"type": "string", "enum": ["hold", "freewheel", "brake", "reset"], "default": "hold"}})
def t_stop(node=None, style="hold"):
    n = DRIVE.target(node)
    return DRIVE.fire(n, tp.mt1(0, tp.MT_MODES[style]), f"stop {style}")


@tool("estop", "Broadcast freewheel + disable to every node on the link. Use this first if anything is "
      "moving that should not be.")
def t_estop():
    with DRIVE.lock:
        link = DRIVE.ensure()
        link.send(tp.ID_COMMAND + tp.NODE_BROADCAST, tp.mt1(0, tp.MT_MODES["freewheel"]))
        link.send(tp.ID_COMMAND + tp.NODE_BROADCAST,
                  bytes([tp.CMD["system_control"] | tp.CMD_WRITE, tp.SYS_DISABLE, 0, 0, 0, 0, 0, 0]))
    return {"ok": True, "command": "emergency stop broadcast"}


@tool("move",
      "MOVES THE SHAFT. Command a position in revolutions. By default a cyclic setpoint (POSITION "
      "mode); with scurve=true or relative=true a jerk-limited point-to-point move is queued in "
      "MOTION mode instead. Returns as soon as the drive accepts it unless wait is set.",
      {"node": NODE_ARG,
       "revolutions": {"type": "number"},
       "relative": {"type": "boolean", "default": False, "description": "Queue a displacement from the queue tail."},
       "scurve": {"type": "boolean", "default": False, "description": "Queue an absolute S-curve move."},
       "velocity": {"type": "number", "description": "Cruise velocity for the S-curve [rev/s]."},
       "wait": {"type": "boolean", "default": False},
       "tolerance": {"type": "number", "default": 0.01},
       "timeout": {"type": "number", "default": 30.0}},
      required=["revolutions"])
def t_move(revolutions, node=None, relative=False, scurve=False, velocity=None, wait=False,
           tolerance=0.01, timeout=30.0):
    n = DRIVE.target(node)
    with DRIVE.lock:
        link = DRIVE.ensure()
        if scurve or relative:
            seg = 1 + int(time.time() * 10) % 250
            frames = [tp.mt1(seg, tp.MT_MODES["scurve"], is_abs=not relative, pos_mode=1, is_vel=velocity is not None),
                      tp.mt2(seg, revolutions)]
            if velocity is not None:
                frames.append(tp.mt3(seg, velocity=velocity))
            link.send_many(tp.ID_COMMAND + n, frames)
            result = {"ok": True, "command": f"scurve {revolutions:+g}rev", "segment": seg}
        else:
            result = DRIVE.fire(n, tp.mt2(0, revolutions), f"move {revolutions:+g}rev")
    if not result["ok"] or not wait:
        return result
    with DRIVE.lock:
        settled = tp.wait_settled(DRIVE.ensure(), n, tolerance, timeout)
    result["settled"] = settled
    result["motion_status_1"] = DRIVE.chunk(n, "motion_status_1")
    return result


@tool("jog",
      "MOVES THE SHAFT. Command a continuous velocity in revolutions per second (VELOCITY mode). "
      "The shaft keeps turning until stopped, so pair it with stop.",
      {"node": NODE_ARG, "rps": {"type": "number"}}, required=["rps"])
def t_jog(rps, node=None):
    n = DRIVE.target(node)
    return DRIVE.fire(n, tp.mt3(0, velocity=rps), f"jog {rps:+g} RPS")


@tool("torque",
      "MOVES THE SHAFT. Command a torque setpoint (TORQUE mode): N*m when Kt is set in motor_props_2, "
      "amperes otherwise.",
      {"node": NODE_ARG, "value": {"type": "number"}}, required=["value"])
def t_torque(value, node=None):
    n = DRIVE.target(node)
    return DRIVE.fire(n, tp.mt3(0, torque=value), f"torque {value:+g}")


@tool("pvt_move",
      "MOVES THE SHAFT. Stream a profiled move as PVT burst frames: the leg is cut into time-tagged "
      "position/velocity points paced against the drive's free queue slots. Set back or cycles>1 to "
      "return to the start.",
      {"node": NODE_ARG,
       "revolutions": {"type": "number"}, "seconds": {"type": "number"},
       "segments": {"type": "integer", "default": 20, "minimum": 1, "maximum": 2000},
       "cycles": {"type": "integer", "default": 1, "minimum": 1, "maximum": 100},
       "back": {"type": "boolean", "default": False},
       "set_mode": {"type": "boolean", "default": True, "description": "Enable the stage first."},
       "wait": {"type": "boolean", "default": False},
       "tolerance": {"type": "number", "default": 0.01}},
      required=["revolutions", "seconds"])
def t_pvt_move(revolutions, seconds, node=None, segments=20, cycles=1, back=False, set_mode=True,
               wait=False, tolerance=0.01):
    n = DRIVE.target(node)
    args = _Args(node=n, revolutions=revolutions, seconds=seconds, segments=segments, cycles=cycles,
                 back=back, set_mode=set_mode, wait=wait, tolerance=tolerance)
    with DRIVE.lock:
        rc = tp.cmd_pvt(DRIVE.ensure(), args)
    return {"ok": rc == 0, "node": n, "queue": DRIVE.chunk(n, "diag:queue"),
            "position": DRIVE.chunk(n, "motion_status_2")}


@tool("queue",
      "Inspect or clear the trajectory queue. Clearing discards queued segments; the shaft holds.",
      {"node": NODE_ARG, "action": {"type": "string", "enum": ["status", "clear"], "default": "status"}})
def t_queue(node=None, action="status"):
    n = DRIVE.target(node)
    if action == "status":
        return {"node": n, "queue": DRIVE.chunk(n, "diag:queue") or {"error": "no response"}}
    result = DRIVE.fire(n, tp.mt1(0, tp.MT_MODES["reset"]), "queue clear")
    result["queue"] = DRIVE.chunk(n, "diag:queue")
    return result


# ============================================================================
# Tools: calibration, persistence, faults
# ============================================================================


def _follow_calibration(node, bits, timeout):
    """Poll the calibration diagnostics until the requested routines stop running."""
    deadline = time.time() + timeout
    started, last = False, None
    while time.time() < deadline:
        time.sleep(0.25)
        status = DRIVE.chunk(node, "diag:calibration")
        if status is None:
            continue
        last = status
        if status["failed"]:
            return {"ok": False, "finished": True, "status": status,
                    "error": f"calibration failed in stage '{status['stage_name']}'",
                    "hint": "the reason is printed on the drive's CDC console"}
        running = (status["running"] & bits) != 0
        if running:
            started = True
        elif started:
            return {"ok": True, "finished": True, "status": status}
    if not started:
        return {"ok": True, "finished": True, "status": last,
                "warning": "the routine never reported itself running"}
    return {"ok": False, "finished": False, "status": last, "error": f"still running after {timeout:g}s"}


@tool("calibrate",
      "TURNS THE SHAFT (kinds 'angle', 'electrical' and 'full'). Start a calibration routine. 'offset' "
      "measures current-sense offsets and does not move. 'electrical' identifies wiring, resistance "
      "and inductance. 'angle' sweeps a full turn to build the encoder table (always a physical "
      "sweep). 'full' runs electrical then angle.",
      {"node": NODE_ARG,
       "kind": {"type": "string", "enum": ["offset", "electrical", "angle", "full"], "default": "angle"},
       "wait": {"type": "boolean", "default": True},
       "timeout": {"type": "number", "default": 120.0}})
def t_calibrate(node=None, kind="angle", wait=True, timeout=120.0):
    n = DRIVE.target(node)
    bits = tp.CAL_BITS[kind]
    with DRIVE.lock:
        reply = tp.system_control(DRIVE.ensure(), n, calibration=bits)
    if reply is None:
        return {"ok": False, "node": n, "error": "no response"}
    if reply[0] == tp.CMD["nack"]:
        return {"ok": False, "node": n, "error": tp.ERRORS.get(reply[2], f"0x{reply[2]:02X}")}
    result = {"ok": True, "node": n, "command": f"calibrate {kind}"}
    if not wait:
        result["status"] = DRIVE.chunk(n, "diag:calibration")
        result["note"] = "started; poll read_status(registers=['diag:calibration']) for progress"
        return result
    result.update(_follow_calibration(n, bits, timeout))
    result["result"] = DRIVE.chunk(n, "diag:calib_result")
    return result


@tool("persist",
      "Save the configuration to flash, restore the compiled-in defaults into the live shadow, or "
      "erase the stored record. Saving requires the drive to be disabled. action='erase' loses the "
      "node address (a discovery sweep is needed afterwards), so it needs confirm=true.",
      {"node": NODE_ARG,
       "action": {"type": "string", "enum": ["save", "defaults", "erase"], "default": "save"},
       "target": {"type": "string", "enum": ["all", "config", "motor"], "default": "config"},
       "confirm": {"type": "boolean", "default": False}})
def t_persist(node=None, action="save", target="config", confirm=False):
    n = DRIVE.target(node)
    if action == "erase" and not confirm:
        raise McpError("erasing the stored configuration leaves the node unaddressed. Pass confirm=true.")
    catalog = {"all": 0xFFFFFF, "config": 1, "motor": 2}[target]
    key = {"save": tp.SAVE_KEY, "defaults": tp.DEFAULTS_KEY, "erase": tp.ERASE_KEY}[action]
    payload = bytes([tp.CMD["save_config"]]) + catalog.to_bytes(3, "little") + struct.pack("<I", key)
    result = DRIVE.command(n, payload, f"{action} {target}")
    result["persist"] = DRIVE.chunk(n, "diag:persist")
    return result


@tool("fault_reset", "Clear latched faults. The condition that caused them must be gone first.", {"node": NODE_ARG})
def t_fault_reset(node=None):
    n = DRIVE.target(node)
    with DRIVE.lock:
        reply = tp.system_control(DRIVE.ensure(), n, action=tp.SYS_FAULT_RESET)
    out = {"ok": reply is not None and reply[0] != tp.CMD["nack"], "node": n}
    out["status"] = DRIVE.chunk(n, "system_status")
    return out


@tool("ripple",
      "Torque-ripple feed-forward map (anti-cogging). 'learn' MOVES THE SHAFT: one slow revolution "
      "(0.05 RPS, 20 s) under position control while the node averages the loop's command per "
      "electrical-angle bin, then enables the map; it removes the low-speed growl (2x/4x electrical "
      "torque ripple). 'clear' disables it, 'show' reads the 64 bins [mA]. RAM only: lost at reset.",
      {"node": NODE_ARG, "what": {"type": "string", "enum": ["learn", "clear", "show"]},
       "rps": {"type": "number"}, "rev": {"type": "number"}}, required=["what"])
def t_ripple(what, node=None, rps=0.05, rev=1.0):
    n = DRIVE.target(node)
    with DRIVE.lock:
        link = DRIVE.ensure()
        if what == "clear":
            reply = tp.system_control(link, n, action=tp.SYS_RIPPLE_CLEAR)
            if reply is None or reply[0] == tp.CMD["nack"]:
                return {"ok": False, "node": n, "error": "clear refused"}
        elif what == "learn":
            seconds = rev / rps
            seg = 1 + int(time.time() * 10) % 250
            link.send_many(tp.ID_COMMAND + n, [tp.mt1(seg, tp.MT_MODES["scurve"], is_abs=False, pos_mode=1, is_vel=True),
                                               tp.mt2(seg, rev), tp.mt3(seg, velocity=rps)])
            time.sleep(1.0)                                    # Past the ramp
            reply = tp.system_control(link, n, action=tp.SYS_RIPPLE_LEARN)
            if reply is None or reply[0] == tp.CMD["nack"]:
                return {"ok": False, "node": n, "error": "learn refused (old firmware?)"}
            time.sleep(max(seconds - 2.0, 0.5))                # Stop before the final ramp
            reply = tp.system_control(link, n, action=tp.SYS_RIPPLE_COMMIT)
            if reply is None or reply[0] == tp.CMD["nack"]:
                return {"ok": False, "node": n, "error": "commit refused"}
            time.sleep(1.5)
        bins, flags = tp.ripple_table(link, n)
    if bins is None:
        return {"ok": False, "node": n, "error": "no ripple map register (old firmware?)"}
    return {"ok": True, "node": n, "enabled": bool(flags & 1), "learning": bool(flags & 2),
            "harmonics_ma": {f"{k}x": round(tp.ripple_harmonic(bins, k)[0], 1) for k in (1, 2, 4, 8)},
            "bins_ma": bins}


@tool("reboot",
      "Reboot the drive, optionally into the USB DFU bootloader (needs confirm=true: it drops the link "
      "until the board is reflashed or power-cycled).",
      {"node": NODE_ARG, "dfu": {"type": "boolean", "default": False}, "confirm": {"type": "boolean", "default": False}})
def t_reboot(node=None, dfu=False, confirm=False):
    if dfu and not confirm:
        raise McpError("rebooting into DFU leaves no protocol link. Pass confirm=true to proceed.")
    n = DRIVE.target(node)
    with DRIVE.lock:
        reply = tp.system_control(DRIVE.ensure(), n, action=tp.SYS_REBOOT_DFU if dfu else tp.SYS_REBOOT)
    result = {"ok": reply is not None and reply[0] != tp.CMD["nack"], "node": n,
              "command": "reboot into DFU" if dfu else "reboot"}
    DRIVE.close()
    result["link"] = "closed - the drive is re-enumerating; call connect again"
    return result


@tool("watchdog",
      "Configure the master-liveness watchdog. With it armed the drive stops on its own if the master "
      "goes quiet. timeout_ms=0 disables it.",
      {"node": NODE_ARG, "timeout_ms": {"type": "integer", "minimum": 0, "maximum": 60000}},
      required=["timeout_ms"])
def t_watchdog(timeout_ms, node=None):
    n = DRIVE.target(node)
    with DRIVE.lock:
        reply = tp.write_register(DRIVE.ensure(), n, "watchdog", {"timeout": timeout_ms / 1000.0})
    if reply is None:
        return {"ok": False, "node": n, "error": "no response"}
    return {"ok": reply[0] != tp.CMD["nack"], "node": n, "result": decode_frame(reply)}


# ============================================================================
# Tools: motor speaker
# ============================================================================


@tool("tone",
      "MAKES THE MOTOR SING. Play one MIDI note through the coils (the drive selects TONE mode and "
      "enables itself). note=0 silences it and disables the stage.",
      {"node": NODE_ARG,
       "note": {"type": "integer", "minimum": 0, "maximum": 127, "description": "MIDI note (69 = A4 = 440 Hz)."},
       "volume": {"type": "integer", "minimum": 0, "maximum": 255, "default": 128},
       "duration_ms": {"type": "number", "default": 0, "description": "0 = until the next command."}},
      required=["note"])
def t_tone(note, node=None, volume=128, duration_ms=0):
    n = DRIVE.target(node)
    DRIVE.stop_playback.set()
    with DRIVE.lock:
        link = DRIVE.ensure()
        if note == 0:
            tp.silence(link, n)
            return {"ok": True, "node": n, "tone": "off"}
    return DRIVE.command(n, tp.midi_frame(note, volume, duration_ms), f"tone {note}")


def _play(node, schedule, volume):
    """Play a parsed schedule of (time, note) pairs on the motor coils."""
    t0 = time.time()
    try:
        for t, note in schedule:
            delay = t0 + t - time.time()
            if delay > 0 and DRIVE.stop_playback.wait(delay):
                break
            if DRIVE.stop_playback.is_set():
                break
            with DRIVE.lock:
                DRIVE.ensure().request(node, tp.midi_frame(note, volume), timeout=0.2)
    finally:
        with DRIVE.lock:
            if DRIVE.link is not None:
                tp.silence(DRIVE.link, node)


@tool("play_midi",
      "MAKES THE MOTOR SING. Play a window of a MIDI file through the motor coils, reduced to one "
      "voice. Plays in the background and returns immediately; call stop_midi or tone(note=0) to stop.",
      {"node": NODE_ARG, "file": {"type": "string"},
       "start": {"type": "number", "default": 0.0}, "end": {"type": "number"},
       "seconds": {"type": "number", "default": 30.0},
       "volume": {"type": "integer", "minimum": 0, "maximum": 255, "default": 128},
       "voice": {"type": "integer", "minimum": 0, "maximum": 15},
       "blocking": {"type": "boolean", "default": False}},
      required=["file"])
def t_play_midi(file, node=None, start=0.0, end=None, seconds=30.0, volume=128, voice=None, blocking=False):
    path = os.path.expanduser(file)
    if not os.path.isfile(path):
        raise McpError(f"no such file: {path}")
    stop = end if end is not None else start + seconds
    if stop <= start:
        raise McpError(f"end ({stop:g}s) must be after start ({start:g}s)")
    if DRIVE.player is not None and DRIVE.player.is_alive():
        raise McpError("a clip is already playing; call stop_midi first")
    schedule, chosen = tp.parse_midi_melody(path, start, stop, voice)
    notes = sum(1 for _, n in schedule if n)
    if notes == 0:
        return {"ok": False, "file": os.path.basename(path), "error": "no notes in that window"}
    DRIVE.stop_playback.clear()
    DRIVE.player = threading.Thread(target=_play, args=(DRIVE.target(node), schedule, volume), daemon=True)
    DRIVE.player.start()
    out = {"ok": True, "file": os.path.basename(path), "node": DRIVE.target(node), "notes": notes,
           "window_s": [start, stop], "channel": chosen, "volume": volume}
    if blocking:
        DRIVE.player.join(timeout=(stop - start) + 10.0)
        out["finished"] = not DRIVE.player.is_alive()
    else:
        out["note"] = f"playing in the background for about {stop - start:g}s"
    return out


@tool("stop_midi", "Stop playback, silence the coils and disable the output stage.", {"node": NODE_ARG})
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
      "Configure one cyclic telemetry slot: the drive then emits that register unprompted at the given "
      "period, which is what 'monitor' listens for. period_ms=0 disables the slot.",
      {"node": NODE_ARG, "slot": {"type": "integer", "minimum": 0, "maximum": 3},
       "register": {"type": "string", "description": "e.g. motion_status_2, pulsebeat, currents:dq"},
       "period_ms": {"type": "integer", "minimum": 0, "maximum": 65535}},
      required=["slot", "register", "period_ms"])
def t_telemetry_config(slot, register, period_ms, node=None):
    if register == "pulsebeat":
        code = 0x70
    elif register in tp.REGISTERS:
        code = tp.REGISTERS[register].cmd
    else:
        raise McpError(f"'{register}' is not a readable register")
    n = DRIVE.target(node)
    payload = bytes([tp.CMD["telemetry"] | tp.CMD_WRITE, slot, code if period_ms else 0,
                     0x01 if period_ms else 0]) + struct.pack("<HH", period_ms, 0)
    return DRIVE.command(n, payload, f"telemetry slot {slot} = {register} @ {period_ms}ms")


@tool("sync", "Broadcast the host clock to every node so PVT time tags line up. Nothing moves.",
      mutating=False)
def t_sync():
    args = _Args()
    with DRIVE.lock:
        tp.cmd_sync(DRIVE.ensure(), args)
    return {"ok": True}


@tool("monitor",
      "Listen for a fixed window and decode every frame that arrives - PulseBeats, telemetry, "
      "emergencies. Nothing is transmitted.",
      {"node": NODE_ARG, "seconds": {"type": "number", "default": 2.0, "minimum": 0.1, "maximum": 30.0},
       "all_nodes": {"type": "boolean", "default": False}},
      mutating=False)
def t_monitor(node=None, seconds=2.0, all_nodes=False):
    n = DRIVE.target(node)
    with DRIVE.lock:
        messages = DRIVE.ensure().drain(seconds)
    frames = []
    kinds = {tp.ID_EMCY: "emergency", tp.ID_COMMAND: "command", tp.ID_RESPONSE: "response",
             tp.ID_TELEMETRY: "telemetry"}
    for msg in messages:
        cid = msg.arbitration_id
        src = cid & 0xFF
        if not all_nodes and src != n and src != tp.NODE_BROADCAST:
            continue
        frames.append({"kind": kinds.get(cid & 0x700, "unknown"), "node": src,
                       "frame": decode_frame(bytes(msg.data).ljust(8, b"\x00"))})
    return {"node": n, "window_s": seconds, "count": len(frames), "frames": frames,
            "note": None if frames else "nothing arrived: is a telemetry slot or the PulseBeat configured?"}


@tool("raw",
      "Send an arbitrary eight-byte command frame and decode the answer. The escape hatch for anything "
      "the typed tools do not cover; see CAN_PROTOCOL.md for the layouts.",
      {"node": NODE_ARG, "hex": {"type": "string", "description": "Payload bytes as hex, padded to 8."}},
      required=["hex"])
def t_raw(hex, node=None):  # noqa: A002
    n = DRIVE.target(node)
    try:
        payload = bytes.fromhex(hex.replace(" ", "")).ljust(8, b"\x00")[:8]
    except ValueError as exc:
        raise McpError(f"not valid hex: {exc}") from exc
    reply = DRIVE.request(n, payload)
    return {"node": n, "sent": payload.hex(" "), "reply": decode_frame(reply) if reply else None,
            "ok": reply is not None and reply[0] != tp.CMD["nack"]}


# ============================================================================
# Resources
# ============================================================================


@resource("tpcp://registers", "TPCP register map", "Every register the drive exposes: name, CMD, fields.",
          "application/json")
def r_registers():
    return json.dumps(t_list_registers(), indent=2)


@resource("tpcp://protocol", "TunePulse CAN Protocol specification",
          "The full wire specification: identifier map, CMD groups, every register, worked examples.",
          "text/markdown")
def r_protocol():
    path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src", "api", "CAN_PROTOCOL.md")
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

Reading is free: read_status, read_register, list_registers, ping and monitor never change anything.

Anything that moves a real shaft says so in capitals at the start of its description - move, jog,
torque, pvt_move, calibrate, and the tone tools, which energize the windings. Check read_status
first, keep estop and stop in mind, and prefer small displacements until you know the mechanics are
free to turn.

Three things about this drive surprise people:

  * Setpoint frames select their own control mode (move -> POSITION, jog -> VELOCITY, torque ->
    TORQUE, queued segments and bursts -> MOTION), but the output stage is enabled separately.
    Nothing moves until enable(on=true) has been accepted, and enabling is refused while the drive
    is uncalibrated or faulted.

  * Accepted setpoints are SILENT in the default response policy: only a rejection comes back as a
    NACK. A tool that reports ok with no reply has simply not been refused.

  * Torque fields are in N*m only once Kt has been written into motor_props_2; until then they are
    amperes.
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
