"""
 * @file monitor_com16.py
 * @brief Temporary USB CDC capture: logs COM port output to a text file.
 *
 * Survives USB re-enumeration (flashing resets the device): retries opening
 * the port until data flows or the deadline expires.
 *
 * Usage: python tools/monitor_com16.py COM16 90 serial_log.txt
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
"""

import sys
import time
import serial  # pyserial

port_name = sys.argv[1] if len(sys.argv) > 1 else "COM16"
duration = float(sys.argv[2]) if len(sys.argv) > 2 else 90.0
out_path = sys.argv[3] if len(sys.argv) > 3 else "serial_log.txt"

deadline = time.time() + duration
port = None
with open(out_path, "w", encoding="utf-8", errors="replace") as out:
    out.write(f"[capture] waiting for {port_name}\n")
    out.flush()
    while time.time() < deadline:
        try:
            if port is None:
                port = serial.Serial(port_name, 576000, timeout=1, write_timeout=1)
                out.write(f"[capture] {port_name} opened at {time.strftime('%H:%M:%S')}\n")
                out.flush()
            data = port.read(1024)
            if data:
                out.write(data.decode("utf-8", errors="replace"))
                out.flush()
        except (serial.SerialException, OSError):
            port = None  # Device re-enumerating: retry until the deadline
            time.sleep(0.1)
    if port is not None:
        port.close()
    out.write(f"\n[capture] {port_name} closed at {time.strftime('%H:%M:%S')}\n")
