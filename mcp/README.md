# TunePulse MCP server

`tpcp_mcp.py` exposes a TunePulse drive to an AI assistant as a set of
[Model Context Protocol](https://modelcontextprotocol.io) tools: connect, read
status, read and write registers by name and field, select a control mode, move
the shaft, stream a profiled trajectory, calibrate, persist, and play MIDI
through the motor coils. It speaks TPCP v1 (`src/api/CAN_PROTOCOL.md`) over the
same USB link as the command-line tool, `tools/tpcp_host.py`, and imports that
file for the wire format, the register map and the transports - the two never
drift apart.

Any MCP client that can spawn a local process can use it: Claude Code, Claude
Desktop, Cursor, VS Code, Codex CLI, Gemini CLI, Windsurf, and so on. The
per-client steps are below; they all reduce to *interpreter path + script path
+ optional flags*.

## 1. Prerequisites

**A drive on USB.** Flash the firmware, connect the board's USB-C port to the
computer. On the CLN17 the cable orientation matters for the full-featured
port: if nothing enumerates, flip it.

**A Python interpreter with the USB bindings.** The server itself is standard
library only (no MCP SDK), but `tpcp_host.py` needs:

| Package      | Needed for                                                | Install                |
|--------------|-----------------------------------------------------------|------------------------|
| `hidapi`     | The vendor HID interface - the preferred transport        | `pip install hidapi`   |
| `pyserial`   | The CDC console fallback (`--interface serial`)           | `pip install pyserial` |
| `python-can` | Only if you drive the bus through a CAN adapter           | `pip install python-can` |

Install them into **the interpreter you will point the client at**. On a bench
that already builds the firmware with PlatformIO, its virtualenv is the natural
home and the path is stable:

| OS            | PlatformIO interpreter                                   |
|---------------|----------------------------------------------------------|
| macOS / Linux | `~/.platformio/penv/bin/python3`                         |
| Windows       | `%USERPROFILE%\.platformio\penv\Scripts\python.exe`      |

```bash
~/.platformio/penv/bin/python3 -m pip install hidapi pyserial
```

Always write the **absolute** interpreter and script paths into a client
config. MCP clients do not run a login shell, so `python3` may resolve to a
different interpreter than in your terminal, and no virtualenv is activated for
them. This is the single most common reason a server "works in the terminal but
not in the app".

**Linux only: device permissions.** hidraw nodes are root-only by default. Save
this as `/etc/udev/rules.d/99-tunepulse.rules`, then
`sudo udevadm control --reload-rules && sudo udevadm trigger` and replug:

```
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5741", MODE="0660", TAG+="uaccess"
```

For the CDC console fallback the user also needs to be in the `dialout`
(Debian/Ubuntu) or `uucp` (Arch) group.

## 2. Check it works before wiring up a client

From the repository root:

```bash
~/.platformio/penv/bin/python3 mcp/tpcp_mcp.py --connect
```

The server prints its startup report on stderr and then waits for JSON-RPC on
stdin. Two lines mean everything is in place - `Ctrl-C` to leave:

```
tpcp-mcp: connected over hid (<device path>)
tpcp-mcp: serving 30 tool(s) on stdio
```

`could not connect` with the second line still printed means the protocol side
is fine and only the USB link is missing; the tools will retry on first use.

To exercise the tools interactively without any assistant, the reference
inspector works out of the box:

```bash
npx @modelcontextprotocol/inspector ~/.platformio/penv/bin/python3 mcp/tpcp_mcp.py
```

## 3. Command-line flags

| Flag                | Effect                                                                                     |
|---------------------|--------------------------------------------------------------------------------------------|
| `--read-only`       | Withhold every tool that could move the shaft or write flash (see [Safety](#5-safety))      |
| `--interface NAME`  | Transport to open: `auto` (default: HID if present, else serial), `hid`, `serial`, or a python-can name (`slcan`, `pcan`, `socketcan`) |
| `--channel DEV`     | Serial device or adapter channel, e.g. `/dev/ttyACM0`, `COM7`. Default `$TPCP_CHANNEL`, else auto-detect |
| `--node N`          | Default node address (default `1`)                                                         |
| `--connect`         | Open the link at startup and report loudly if no drive answers, instead of on first use     |

Flags go after the script path in every client config below, in `args`.

## 4. Client setup

Replace `/ABS/PATH/TO/python3` and `/ABS/PATH/TO/TP_FW` with your own absolute
paths in every snippet. On Windows use forward slashes (`C:/Users/me/...`) or
doubled backslashes inside JSON.

### Claude Code

```bash
claude mcp add --transport stdio tunepulse -- /ABS/PATH/TO/python3 /ABS/PATH/TO/TP_FW/mcp/tpcp_mcp.py
```

Add `--read-only` (or any other flag) after the script path. To pin a serial
port, pass it as an environment variable or a flag:

```bash
claude mcp add --transport stdio --env TPCP_CHANNEL=/dev/ttyACM0 tunepulse -- /ABS/PATH/TO/python3 /ABS/PATH/TO/TP_FW/mcp/tpcp_mcp.py
```

Scopes: the default `--scope local` registers the server for this checkout
only (stored in `~/.claude.json`); `--scope user` makes it available in every
project; `--scope project` writes a `.mcp.json` at the repository root that can
be committed and shared. Claude Code launches a project server from the
repository directory, so the script path can stay relative, and `.mcp.json`
expands `${VAR:-default}` so each machine can name its own interpreter:

```json
{
  "mcpServers": {
    "tunepulse": {
      "command": "${TPCP_PYTHON:-python3}",
      "args": ["mcp/tpcp_mcp.py", "--read-only"]
    }
  }
}
```

With that file in place, `export TPCP_PYTHON=~/.platformio/penv/bin/python3` in
the shell that starts `claude` selects the PlatformIO interpreter; without it,
whatever `python3` is on the PATH is used. Each collaborator approves the
project server once, on first use.

Useful afterwards: `claude mcp list` (with a health check), `claude mcp get
tunepulse`, `claude mcp remove tunepulse`, and `/mcp` inside a session to see
the connection state and the tool list.

### Claude Desktop

Edit the config file (create it if missing), then quit and restart the app:

| OS      | File                                                                 |
|---------|----------------------------------------------------------------------|
| macOS   | `~/Library/Application Support/Claude/claude_desktop_config.json`    |
| Windows | `%APPDATA%\Claude\claude_desktop_config.json`                        |

```json
{
  "mcpServers": {
    "tunepulse": {
      "command": "/ABS/PATH/TO/python3",
      "args": ["/ABS/PATH/TO/TP_FW/mcp/tpcp_mcp.py", "--read-only"]
    }
  }
}
```

The server's stderr lands in `~/Library/Logs/Claude/mcp-server-tunepulse.log`
(macOS) or `%APPDATA%\Claude\logs\` (Windows), which is where the startup
report and any connection error appear.

### Cursor

`.cursor/mcp.json` in the repository (project-wide) or `~/.cursor/mcp.json`
(every project):

```json
{
  "mcpServers": {
    "tunepulse": {
      "command": "/ABS/PATH/TO/python3",
      "args": ["/ABS/PATH/TO/TP_FW/mcp/tpcp_mcp.py"]
    }
  }
}
```

Then enable it under *Settings → MCP*; the tools appear in agent mode.

### VS Code (GitHub Copilot agent mode)

`.vscode/mcp.json` - note the `servers` key and the explicit `type`. VS Code
expands `${workspaceFolder}`, so the script path can stay relative to the
checkout:

```json
{
  "servers": {
    "tunepulse": {
      "type": "stdio",
      "command": "/ABS/PATH/TO/python3",
      "args": ["${workspaceFolder}/mcp/tpcp_mcp.py"]
    }
  }
}
```

`.vscode/` is gitignored in this repository, so this file is per machine.
Start it from the *MCP: List Servers* command or the inline *Start* link the
editor shows above the server entry.

### OpenAI Codex CLI

```bash
codex mcp add tunepulse -- /ABS/PATH/TO/python3 /ABS/PATH/TO/TP_FW/mcp/tpcp_mcp.py
```

or, by hand, in `~/.codex/config.toml`:

```toml
[mcp_servers.tunepulse]
command = "/ABS/PATH/TO/python3"
args = ["/ABS/PATH/TO/TP_FW/mcp/tpcp_mcp.py"]
```

### Gemini CLI

```bash
gemini mcp add tunepulse /ABS/PATH/TO/python3 /ABS/PATH/TO/TP_FW/mcp/tpcp_mcp.py
```

or in `~/.gemini/settings.json` (user-wide) / `.gemini/settings.json`
(project):

```json
{
  "mcpServers": {
    "tunepulse": {
      "command": "/ABS/PATH/TO/python3",
      "args": ["/ABS/PATH/TO/TP_FW/mcp/tpcp_mcp.py"]
    }
  }
}
```

### Windsurf

`~/.codeium/windsurf/mcp_config.json`, same shape as Cursor and Claude Desktop:

```json
{
  "mcpServers": {
    "tunepulse": {
      "command": "/ABS/PATH/TO/python3",
      "args": ["/ABS/PATH/TO/TP_FW/mcp/tpcp_mcp.py"]
    }
  }
}
```

### Any other client

The server is newline-delimited JSON-RPC 2.0 on stdin/stdout, MCP protocol
revisions `2024-11-05` through `2025-06-18`, with `tools` and `resources`
capabilities and no SDK-specific behaviour. If a client can launch a command
and talk MCP over its pipes, it can use this one - give it the interpreter, the
script and the flags. The server writes nothing but protocol frames to stdout;
diagnostics go to stderr.

A manual handshake, for a client whose logs are unhelpful:

```bash
printf '%s\n' \
 '{"jsonrpc":"2.0","id":1,"method":"initialize","params":{"protocolVersion":"2025-06-18","capabilities":{},"clientInfo":{"name":"probe","version":"0"}}}' \
 '{"jsonrpc":"2.0","method":"notifications/initialized"}' \
 '{"jsonrpc":"2.0","id":2,"method":"tools/list"}' \
| /ABS/PATH/TO/python3 mcp/tpcp_mcp.py
```

## 5. Safety

The drive turns a real shaft. Three layers stand between an assistant and an
unwanted move:

1. **Tool descriptions say so.** Every tool that energises the windings starts
   its description with `MOVES THE SHAFT`, `TURNS THE SHAFT` or `MAKES THE
   MOTOR SING`, and the server's instructions tell the model to read status
   first and prefer small displacements.
2. **`confirm=true` for the irreversible.** Erasing the stored configuration
   (`persist` with `action=erase`, which also loses the node address) and
   rebooting into the DFU bootloader (`reboot` with `dfu=true`) refuse without
   it.
3. **`--read-only` removes the whole mutating half.** The 21 tools that can move
   the shaft, write a register, or touch flash are not refused - they are not
   listed, so the model cannot even see them. What remains: `connect`,
   `disconnect`, `link_status`, `ping`, `read_status`, `list_registers`,
   `read_register`, `sync` and `monitor`.

A practical arrangement is to register the read-only server under one name for
everyday sessions and the full server under another (`tunepulse-rw`, say) that
you enable only while at the bench.

The drive's own protections are unchanged: the output stage must be enabled
(`enable`), which is refused while uncalibrated or faulted; the trajectory
queue only advances in motion mode; and an armed `watchdog` stops a drive whose
master has gone quiet. `estop` broadcasts freewheel + disable to every node and
is the right first call if anything is moving that should not be.

**One process holds the HID device.** While the server has the link open,
`tools/tpcp_host.py` and the desktop application cannot reach the drive. Call
the `disconnect` tool (or end the session) first; the link reopens on the next
tool call.

## 6. What the assistant gets

Thirty tools and two resources. `--read-only` leaves the first group and the
listeners.

| Group        | Tools                                                                                       |
|--------------|---------------------------------------------------------------------------------------------|
| Link         | `connect`, `disconnect`, `link_status`, `scan`, `ping`, `sync`                              |
| Read         | `read_status`, `read_register`, `list_registers`, `monitor`, `queue`                        |
| Configure    | `write_register`, `persist`, `watchdog`, `telemetry_config`, `fault_reset`, `reboot`        |
| Mode         | `set_mode`, `enable`, `stop`, `estop`                                                       |
| Motion       | `move`, `jog`, `torque`, `pvt_move`, `calibrate`                                            |
| Sound        | `tone`, `play_midi`, `stop_midi`                                                            |
| Escape hatch | `raw` - any eight-byte command frame, decoded answer                                        |

| Resource           | Content                                                        |
|--------------------|----------------------------------------------------------------|
| `tpcp://registers` | The register map: name, CMD, fields, writability                |
| `tpcp://protocol`  | `src/api/CAN_PROTOCOL.md` in full, read from this checkout      |

Every tool answers JSON - decoded fields, register values, queue depth,
calibration flags - so the model works with numbers rather than prose. Where
the underlying CLI helper prints something, that text is kept alongside under
`log`.

## 7. Troubleshooting

**Works in the terminal, fails in the client.** Almost always the interpreter:
the client launched a `python3` without `hidapi`, or a relative path was
resolved from a different working directory. Use absolute paths for both the
interpreter and the script, and check the client's server log for the
`tpcp-mcp:` startup lines.

**`no TunePulse HID interface found`.** In order: is `hidapi` installed in
*that* interpreter (`/ABS/PATH/TO/python3 -c "import hid"`); is the udev rule
in place (Linux); is the USB-C cable the right way round; is another process -
`tpcp_host.py`, the desktop app, a second assistant session - already holding
the device.

**`python-can is not installed`.** Harmless unless you selected a CAN
`--interface`. The server stands in a stub for the module so a USB-only bench
does not need a CAN stack.

**A tool reports `ok` but nothing happened.** Accepted setpoints are silent
under the default response policy - only rejections come back. Read
`read_status` to see what the drive actually did, and remember nothing moves
until `enable` has been accepted.

**The client disconnects the server immediately.** Something wrote to stdout
that was not a frame. The server rebinds `sys.stdout` to stderr at startup
precisely so that `tpcp_host`'s prints cannot do this; if you extend the
server, keep that rule - print to stderr, return data.
