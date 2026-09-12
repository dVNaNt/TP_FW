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
 *      PIDs) through the Windows PnP subsystem and extracts their serial
 *      numbers.
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
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
"""

import argparse                                     # Command line parsing (standalone mode)
import os                                           # Path manipulation
import re                                           # Serial number validation
import subprocess                                   # Launch PowerShell and OpenOCD
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


# ---------------------------------------------------------------------------
# ST-Link enumeration
# ---------------------------------------------------------------------------
def list_stlink_serials() -> list:
    """Enumerate all currently connected ST-Link probes.

    Uses the Windows PnP device tree via PowerShell: queries present USB
    devices with VID 0483 and one of the known ST-LINK PIDs, then extracts
    the serial number (last segment of the InstanceId).

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
        if not re.match(r"^[0-9A-Za-z]+$", serial):     # Sanity check on serial format
            continue
        if serial not in serials:                       # Skip duplicates
            serials.append(serial)
    return serials


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
        return 1

    serials = list_stlink_serials()                     # Discover connected probes
    if not serials:
        print("[ERROR] No ST-Link probes detected. "
              "Connect at least one board and retry.")
        return 1

    print(f"Detected {len(serials)} ST-Link probe(s): " + ", ".join(serials))
    script_dir = os.path.normpath(                      # OpenOCD scripts search path
        os.path.join(os.path.dirname(openocd), "..", "scripts"))

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
    openocd_dir = platform.get_package_dir("tool-openocd")  # May be None if not installed
    # Target config from the board manifest (e.g. stm32g4x for G431)
    target = env.BoardConfig().get("debug.openocd_target", "stm32g4x")
    script = os.path.join(env.subst("$PROJECT_DIR"),    # Absolute path of this script
                          "tools", "upload_all_stlinks.py")
    # Firmware path computed at hook time; forward slashes avoid TCL escape
    # issues in OpenOCD's "program" command (e.g. ".pio\build..." -> "\b")
    firmware = env.subst("$BUILD_DIR/${PROGNAME}.elf").replace("\\", "/")
    # Upload flags: target config file and firmware image path. When
    # tool-openocd is not installed (e.g. build-only CI runs) the --openocd
    # option is omitted on purpose: main() resolves the binary lazily at
    # upload time, so a plain `pio run` build never touches OpenOCD and
    # cannot crash on the missing package.
    flags = [
        "--target", f"target/{target}.cfg",             # Target config file
        firmware,                                       # Firmware .elf path
    ]
    if openocd_dir is not None:                         # Package present: resolve now
        # On POSIX the binary has no .exe suffix; pick the name by host OS
        exe = "openocd.exe" if os.name == "nt" else "openocd"
        flags = ["--openocd",                           # Resolved OpenOCD binary
                 os.path.join(openocd_dir, "bin", exe)] + flags
    # Replace uploader: python <this script> [--openocd <path>] <firmware>
    env.Replace(
        UPLOADER=f'"{sys.executable}" "{script}"',
        UPLOADERFLAGS=flags)


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
        exe = "openocd.exe" if os.name == "nt" else "openocd"  # Host-specific binary name
        args.openocd = os.path.join(
            base, "packages", "tool-openocd", "bin", exe)
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
