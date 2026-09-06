"""
 * @file upload_all_stlinks.py
 * @brief PlatformIO post-script: upload firmware to ALL boards via ALL ST-Links
 *
 * This script hooks into the PlatformIO build system and replaces the standard
 * "upload" command. Instead of flashing a single board, it enumerates every
 * ST-Link probe currently connected to the PC and flashes the built firmware
 * to each attached board in parallel (one OpenOCD instance per probe).
 *
 * How it works:
 * 1. When loaded by SCons (Import("env")), it replaces UPLOADER/UPLOADERFLAGS
 *    so the built-in "upload" target invokes this same file as a standalone
 *    Python program with the firmware (.elf) path as an argument.
 * 2. As a standalone program it:
 *    - Detects all present ST-Link USB devices (VID 0x0483, known ST-LINK
 *      PIDs) and extracts their serial numbers. Enumeration is per-platform:
 *      the Windows PnP subsystem (PowerShell), the macOS I/O registry
 *      (ioreg) or the Linux USB sysfs tree.
 *    - For every detected probe, launches OpenOCD from the PlatformIO
 *      tool-openocd package with "hla_serial <serial>" to bind the instance
 *      to that specific probe, then programs, verifies and resets the target.
 *    - Runs all flashing jobs concurrently in threads and streams each
 *      instance output prefixed with its serial number.
 *    - Exits with non-zero status if any of the flashes failed.
 *
 * Usage (standalone, not required when wired via platformio.ini):
 *   python tools/upload_all_stlinks.py path/to/firmware.elf
 *   python tools/upload_all_stlinks.py --list
 *
 * Dependencies: OpenOCD (shipped with PlatformIO tool-openocd package).
 * Supported hosts: Windows, macOS and Linux.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
"""

import argparse                                     # Command line parsing (standalone mode)
import os                                           # Path manipulation
import re                                           # Serial number validation
import subprocess                                   # Launch the USB enumerator and OpenOCD
import sys                                          # Exit codes and interpreter path
import threading                                    # Lock for interleaved console output
from concurrent.futures import ThreadPoolExecutor   # Parallel flashing per probe

# Known ST-LINK USB product IDs (VID 0x0483, STMicroelectronics)
STLINK_PIDS = (
    "3744",  # ST-LINK/V1
    "3748",  # ST-LINK/V2
    "374A",  # ST-LINK/V2.1 (audio)
    "374B",  # ST-LINK/V2.1
    "374D",  # ST-LINK/V2.1 NoMSD
    "374E",  # STLINK-V3E
    "374F",  # STLINK-V3S
    "3752",  # STLINK-V3S
    "3753",  # STLINK-V2.1
    "3754",  # STLINK-V3A
    "3755",  # STLINK-V3 MINIE
)

_print_lock = threading.Lock()                      # Serializes console output of threads

# Host-platform discriminators: every platform-specific branch keys off these.
_IS_WINDOWS = sys.platform.startswith("win")        # Windows (PowerShell PnP enumeration)
_IS_MACOS = sys.platform == "darwin"                # macOS (ioreg enumeration)
_IS_LINUX = sys.platform.startswith("linux")        # Linux (USB sysfs enumeration)

# Numeric form of the ST-LINK PIDs, used by the ioreg/sysfs backends which
# report idProduct as a decimal (ioreg) or lowercase-hex (sysfs) value.
_STLINK_PIDS_INT = frozenset(int(pid, 16) for pid in STLINK_PIDS)
_STLINK_VID_INT = 0x0483                            # STMicroelectronics vendor ID


# ---------------------------------------------------------------------------
# Toolchain path resolution
# ---------------------------------------------------------------------------
def openocd_exe_name() -> str:
    """OpenOCD executable file name for the running host.

    @return "openocd.exe" on Windows, "openocd" everywhere else
    """
    return "openocd.exe" if _IS_WINDOWS else "openocd"


def resolve_openocd(package_dir: str) -> str:
    """Build the path to the OpenOCD binary inside a tool-openocd package.

    @param package_dir Root of the PlatformIO tool-openocd package
    @return Absolute path to the platform-appropriate OpenOCD executable
    """
    return os.path.join(package_dir, "bin", openocd_exe_name())


def resolve_scripts_dir(openocd: str) -> str:
    """Locate the OpenOCD scripts (.cfg search path) for this package layout.

    The directory moved between packaging variants: Windows builds keep it
    next to bin/ ("<pkg>/scripts"), the macOS/Linux PlatformIO builds nest it
    one level deeper ("<pkg>/openocd/scripts") and distro builds install it
    under share/. Probing beats hardcoding any single one of them.

    @param openocd Path to the OpenOCD executable
    @return First existing scripts directory, or the legacy guess as fallback
    """
    package_dir = os.path.dirname(os.path.dirname(os.path.abspath(openocd)))
    candidates = (
        os.path.join(package_dir, "openocd", "scripts"),        # PlatformIO macOS/Linux layout
        os.path.join(package_dir, "scripts"),                   # PlatformIO Windows layout
        os.path.join(package_dir, "share", "openocd", "scripts"),  # Distro/source install layout
    )
    for candidate in candidates:                        # First hit wins
        if os.path.isdir(candidate):                    # Directory must actually exist
            return os.path.normpath(candidate)
    return os.path.normpath(candidates[0])              # Nothing found: report the expected one


# ---------------------------------------------------------------------------
# ST-Link enumeration
# ---------------------------------------------------------------------------
def _serials_windows() -> list:
    """Enumerate ST-Link probes through the Windows PnP device tree.

    Queries present USB devices with VID 0483 and one of the known ST-LINK
    PIDs via PowerShell, then extracts the serial number (last segment of
    the InstanceId).

    @return List of serial number strings, one per detected probe
    """
    # Wildcard clauses per known PID, e.g. "$_.InstanceId -like '*VID_0483&PID_3748*'"
    clauses = " -or ".join(
        f"$_.InstanceId -like '*VID_0483&PID_{pid}*'" for pid in STLINK_PIDS)

    # PowerShell query: list InstanceIds of present ST-LINK USB devices
    ps_cmd = (
        f"Get-PnpDevice -PresentOnly "
        f"| Where-Object {{ {clauses} }} "
        "| Select-Object -ExpandProperty InstanceId"
    )
    try:
        result = subprocess.run(
            ["powershell", "-NoProfile", "-NonInteractive", "-Command", ps_cmd],
            capture_output=True, text=True, check=True, timeout=30)
    except (subprocess.SubprocessError, OSError) as exc:
        print(f"[ERROR] Cannot enumerate USB devices: {exc}")
        return []

    serials = []                                        # Deduplicated serial list
    for line in result.stdout.splitlines():             # One InstanceId per line
        line = line.strip()                             # Trim whitespace
        if not line:                                    # Skip empty lines
            continue
        serial = line.rsplit("\\", 1)[-1]               # Serial = last DeviceID segment
        if serial not in serials:                       # Skip duplicates
            serials.append(serial)
    return serials


def _serials_macos() -> list:
    """Enumerate ST-Link probes through the macOS I/O registry.

    "ioreg -p IOUSB -l" prints the USB device tree; every device node starts
    with a "+-o " line and is followed by its property block. Properties are
    therefore accumulated per node and evaluated when the next node (or EOF)
    is reached.

    @return List of serial number strings, one per detected probe
    """
    try:
        result = subprocess.run(
            ["ioreg", "-p", "IOUSB", "-l", "-w", "0"],
            capture_output=True, text=True, check=True, timeout=30)
    except (subprocess.SubprocessError, OSError) as exc:
        print(f"[ERROR] Cannot enumerate USB devices: {exc}")
        return []

    serials = []                                        # Deduplicated serial list
    node = {}                                           # Properties of the node being parsed

    def commit(props: dict) -> None:
        """Adopt the parsed node when it is an ST-Link with a usable serial."""
        if props.get("vid") != _STLINK_VID_INT:         # Not STMicroelectronics
            return
        if props.get("pid") not in _STLINK_PIDS_INT:    # Not a known ST-LINK PID
            return
        serial = props.get("serial")                    # Probe serial as reported by ioreg
        if serial and serial not in serials:            # Keep the first occurrence only
            serials.append(serial)

    for line in result.stdout.splitlines():             # Walk the flattened device tree
        if "+-o " in line:                              # A new device node begins here
            commit(node)                                # Evaluate the node that just ended
            node = {}                                   # Start collecting the new one
            continue
        match = re.search(r'"idVendor"\s*=\s*(\d+)', line)
        if match:                                       # Vendor ID (decimal in ioreg)
            node["vid"] = int(match.group(1))
            continue
        match = re.search(r'"idProduct"\s*=\s*(\d+)', line)
        if match:                                       # Product ID (decimal in ioreg)
            node["pid"] = int(match.group(1))
            continue
        match = re.search(r'"USB Serial Number"\s*=\s*"([^"]+)"', line)
        if match:                                       # Probe serial number string
            node["serial"] = match.group(1)
    commit(node)                                        # Evaluate the final node at EOF
    return serials


def _serials_linux() -> list:
    """Enumerate ST-Link probes through the Linux USB sysfs tree.

    Every USB device exposes idVendor/idProduct/serial as plain files under
    /sys/bus/usb/devices/<dev>/, so no external tool is required.

    @return List of serial number strings, one per detected probe
    """
    serials = []                                        # Deduplicated serial list
    root = "/sys/bus/usb/devices"                       # Sysfs USB device root
    try:
        entries = sorted(os.listdir(root))              # One directory per USB device/interface
    except OSError as exc:
        print(f"[ERROR] Cannot enumerate USB devices: {exc}")
        return []

    def read_attr(device: str, name: str) -> str:
        """Read one sysfs attribute, returning an empty string when absent."""
        try:
            with open(os.path.join(root, device, name), encoding="utf-8") as handle:
                return handle.read().strip()
        except OSError:
            return ""                                   # Interface nodes lack these attributes

    for entry in entries:                               # Probe every enumerated device
        vid = read_attr(entry, "idVendor")              # Vendor ID (lowercase hex)
        if not vid or int(vid, 16) != _STLINK_VID_INT:  # Not STMicroelectronics
            continue
        pid = read_attr(entry, "idProduct")             # Product ID (lowercase hex)
        if not pid or int(pid, 16) not in _STLINK_PIDS_INT:  # Not a known ST-LINK PID
            continue
        serial = read_attr(entry, "serial")             # Probe serial number string
        if serial and serial not in serials:            # Keep the first occurrence only
            serials.append(serial)
    return serials


def list_stlink_serials() -> list:
    """Enumerate all currently connected ST-Link probes on any supported host.

    Dispatches to the platform-specific backend and filters the result to
    serials OpenOCD can accept as an "hla_serial" argument.

    @return List of serial number strings, one per detected probe
    """
    if _IS_WINDOWS:                                     # Windows: PowerShell PnP query
        serials = _serials_windows()
    elif _IS_MACOS:                                     # macOS: I/O registry dump
        serials = _serials_macos()
    elif _IS_LINUX:                                     # Linux: USB sysfs tree
        serials = _serials_linux()
    else:                                               # Unknown host: fail loudly, not silently
        print(f"[ERROR] Unsupported host platform: {sys.platform}")
        return []

    # Sanity check on the serial format, applied uniformly to every backend.
    return [s for s in serials if re.match(r"^[0-9A-Za-z]+$", s)]


# ---------------------------------------------------------------------------
# Flashing
# ---------------------------------------------------------------------------
def flash_one(openocd: str, serial: str, target_cfg: str,
              firmware: str, script_dir: str) -> int:
    """Flash a single board through one specific ST-Link probe.

    Launches an OpenOCD instance bound to the probe by serial number
    (hla_serial), which allows many probes to run concurrently.

    @param openocd    Path to openocd executable
    @param serial     ST-Link serial number (empty string = first probe found)
    @param target_cfg OpenOCD target config, e.g. "target/stm32g4x.cfg"
    @param firmware   Path to firmware image (.elf/.bin/.hex)
    @param script_dir OpenOCD scripts directory (-s search path)
    @return Process exit code (0 = success)
    """
    firmware = firmware.replace("\\", "/")              # Forward slashes: TCL-safe path
    # Base arguments: scripts search path, SWD interface on ST-LINK, MCU target
    args = [openocd, "-d1", "-s", script_dir,
            "-f", "interface/stlink.cfg"]
    if serial:                                          # Bind to specific probe if known
        args += ["-c", f"hla_serial {serial}"]
    args += ["-f", target_cfg,
             # Program firmware, verify flash, run target, then exit OpenOCD
             "-c", f'program "{firmware}" verify reset; shutdown;']

    tag = serial if serial else "default"               # Output prefix per probe
    print(f"[{tag}] Flashing started...")
    try:
        proc = subprocess.Popen(args, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True)
    except OSError as exc:                              # openocd binary missing/unusable
        print(f"[{tag}] [ERROR] Cannot start OpenOCD: {exc}")
        return 1
    for line in proc.stdout:                            # Stream output with probe prefix
        with _print_lock:                               # Prevent interleaved writes
            print(f"[{tag}] {line.rstrip()}")
    proc.wait()                                         # Block until OpenOCD exits
    status = "OK" if proc.returncode == 0 else "FAILED"
    print(f"[{tag}] Flashing finished: {status}")
    return proc.returncode


def run(openocd: str, target_cfg: str, firmware: str, jobs: int = 0) -> int:
    """Detect all ST-Links and flash the firmware to all boards in parallel.

    @param openocd    Path to openocd executable
    @param target_cfg OpenOCD target config name
    @param firmware   Path to firmware image
    @param jobs       Max parallel workers (0 = one per probe)
    @return 0 if all boards flashed successfully, 1 otherwise
    """
    if not os.path.isfile(firmware):                    # Firmware image must exist
        print(f"[ERROR] Firmware not found: {firmware}")
        return 1
    if not os.path.isfile(openocd):                     # OpenOCD binary must exist
        print(f"[ERROR] OpenOCD not found: {openocd}")
        print("        Install it with: pio pkg install --tool tool-openocd")
        return 1

    serials = list_stlink_serials()                     # Discover connected probes
    if not serials:
        print("[ERROR] No ST-Link probes detected. "
              "Connect at least one board and retry.")
        return 1

    print(f"Detected {len(serials)} ST-Link probe(s): " + ", ".join(serials))
    script_dir = resolve_scripts_dir(openocd)           # OpenOCD scripts search path (layout-dependent)

    # Flash every probe concurrently; each thread owns one OpenOCD process
    with ThreadPoolExecutor(max_workers=jobs or len(serials)) as pool:
        results = list(pool.map(
            lambda s: flash_one(openocd, s, target_cfg, firmware, script_dir),
            serials))

    failed = sum(1 for rc in results if rc != 0)        # Count failed flashes
    print(f"\nDone: {len(results) - failed}/{len(results)} board(s) flashed")
    return 0 if failed == 0 else 1                      # Non-zero if any failed


# ---------------------------------------------------------------------------
# Entry points
# ---------------------------------------------------------------------------
def pio_post_hook(env) -> None:
    """Called when loaded as a PlatformIO extra_script (post).

    Replaces the built-in upload command so that `pio run -t upload` runs
    this script instead of the single-probe OpenOCD invocation.
    """
    platform = env.PioPlatform()                        # Access PlatformIO platform
    openocd_dir = platform.get_package_dir("tool-openocd")  # Locate tool-openocd package
    openocd = resolve_openocd(openocd_dir)              # Platform-appropriate binary path
    # Target config from the board manifest (e.g. stm32g4x for G431)
    target = env.BoardConfig().get("debug.openocd_target", "stm32g4x")
    script = os.path.join(env.subst("$PROJECT_DIR"),    # Absolute path of this script
                          "tools", "upload_all_stlinks.py")
    # Firmware path computed at hook time; forward slashes avoid TCL escape
    # issues in OpenOCD's "program" command (e.g. ".pio\build..." -> "\b")
    firmware = env.subst("$BUILD_DIR/${PROGNAME}.elf").replace("\\", "/")
    # Replace uploader: python <this script> --openocd <path> --target <cfg> <firmware>
    env.Replace(
        UPLOADER=f'"{sys.executable}" "{script}"',
        UPLOADERFLAGS=[
            "--openocd", openocd,                       # Resolved OpenOCD binary
            "--target", f"target/{target}.cfg",         # Target config file
            firmware,                                   # Firmware .elf path
        ])


def main() -> int:
    """Standalone entry point used by the replaced upload command."""
    parser = argparse.ArgumentParser(
        description="Upload firmware to all boards through all ST-Links")
    parser.add_argument("firmware", nargs="?", default=None,
                        help="Firmware image (.elf/.bin/.hex)")
    parser.add_argument("--openocd", default=None, metavar="PATH",
                        help="Path to openocd executable")
    parser.add_argument("--target", default="target/stm32g4x.cfg",
                        metavar="CFG", help="OpenOCD target config")
    parser.add_argument("--jobs", type=int, default=0, metavar="N",
                        help="Max parallel flashing jobs (0 = auto)")
    parser.add_argument("--list", action="store_true",
                        help="Only list detected ST-Link probes and exit")
    args = parser.parse_args()

    if args.list:                                       # Discovery-only mode
        serials = list_stlink_serials()
        if not serials:
            print("No ST-Link probes detected")
            return 1
        print("\n".join(serials))                       # One serial per line
        return 0

    if not args.firmware:                               # Firmware required otherwise
        parser.error("firmware image is required (or use --list)")
    if not args.openocd:                                # Default: PlatformIO package
        base = os.environ.get(                          # PIO home dir or ~/.platformio
            "PLATFORMIO_HOME_DIR",
            os.path.join(os.path.expanduser("~"), ".platformio"))
        args.openocd = resolve_openocd(
            os.path.join(base, "packages", "tool-openocd"))
    return run(args.openocd, args.target, args.firmware, args.jobs)


try:
    Import  # Check SCons availability (defined only when loaded by PlatformIO)
except NameError:
    # Standalone execution: python tools/upload_all_stlinks.py ...
    if __name__ == "__main__":
        sys.exit(main())
else:
    Import("env")                                       # Pull in the build environment
    pio_post_hook(env)                                  # Install the upload override
