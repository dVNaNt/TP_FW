"""
 * @file can_test_host.py
 * @brief PC-side host test for the two-board CAN peer-to-peer text exchange.
 *
 * Opens the USB CDC serial ports of both STM32 boards, continuously prints
 * everything they send, and can push a text line into one of the boards so
 * it gets transmitted over the CAN bus and printed by the peer.
 *
 * Usage:
 *   python tools/can_test_host.py list                 # list candidate ports
 *   python tools/can_test_host.py COM11 COM12          # interactive session
 *   python tools/can_test_host.py COM11 COM12 "hello"  # one-shot: send to 1st, watch both
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
"""

import sys
import time
import threading

import serial
import serial.tools.list_ports


class Board:
    """Owns one serial port: a background reader thread plus a thread-safe
    write() used by the main thread, so a single handle serves both roles."""

    def __init__(self, port_name: str):
        self.name = port_name
        self.ser = serial.Serial(port_name, 115200, timeout=0.1)
        self.lock = threading.Lock()
        self.lines = []                               # Received line history
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def _reader(self):
        """Background loop: print every complete line received from the board."""
        buf = b""
        while not self.stop.is_set():
            try:
                data = self.ser.read(256)         # Non-blocking chunk read
            except serial.SerialException as e:
                print(f"[{self.name}] ERROR: {e}", flush=True)
                return
            if data:
                buf += data
                while b"\n" in buf:               # Emit complete lines only
                    line, buf = buf.split(b"\n", 1)
                    text = line.decode(errors="replace").rstrip("\r")
                    if text:
                        print(f"[{self.name}] {text}", flush=True)
                        with self.lock:           # Keep a copy for wait_for()
                            self.lines.append(text)

    def write_line(self, text: str):
        """Send one text line (bytes go to CAN through the firmware)."""
        with self.lock:                           # Serialize with nothing else
            self.ser.write((text + "\n").encode())

    def wait_for(self, prefix: str, timeout_s: float):
        """Wait for a line starting with @p prefix; return it (consumed) or None."""
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            with self.lock:
                for i, line in enumerate(self.lines):     # Scan received history
                    if line.startswith(prefix):
                        del self.lines[:i + 1]            # Consume up to the match
                        return line
            time.sleep(0.05)                              # Small poll period
        return None

    def clear(self):
        """Drop all buffered lines (mark a logical boundary in the log)."""
        with self.lock:
            self.lines.clear()

    def close(self):
        self.stop.set()
        self.ser.close()


def list_ports():
    """Print all serial ports with descriptions to help identify the boards."""
    for p in serial.tools.list_ports.comports():
        print(f"{p.device} | {p.description} | {p.hwid}")


def interactive(port_a: str, port_b: str):
    """Interactive session: stdin lines are sent to port A; both ports watched."""
    a = Board(port_a)
    b = Board(port_b)
    print(f"Watching {port_a} and {port_b}. Type text + Enter to send to {port_a}, 'quit' to exit.")
    try:
        for line in sys.stdin:
            text = line.strip()
            if text.lower() == "quit":
                break
            if text:
                a.write_line(text)                # Board packs this into CAN frames
    finally:
        a.close()
        b.close()
        time.sleep(0.2)


def one_shot(port_a: str, port_b: str, message: str, watch_s: float = 6.0):
    """Send one message to port A, watch both ports for the given duration."""
    a = Board(port_a)
    b = Board(port_b)
    time.sleep(0.5)                               # Let banner/stats lines flow
    a.write_line(message)                         # Trigger CAN transmission
    print(f">>> sent to {port_a}: {message!r}", flush=True)
    time.sleep(watch_s)
    a.close()
    b.close()
    time.sleep(0.2)


def capture(port_a: str, port_b: str, seconds: float = 10.0):
    """Wait for both boards' CDC ports to (re)enumerate, then print all
    output for a while - used right after flashing to catch boot banners."""
    import serial.tools.list_ports as lp

    def wait_reboot(name, timeout_s=25.0):
        """Wait for the port to disappear (board reset) and re-enumerate,
        then open it. This catches the boot banner reliably after flashing."""
        deadline = time.time() + timeout_s
        phase1_deadline = time.time() + 2.0      # Brief window to spot the reset
        while time.time() < phase1_deadline:    # Phase 1: wait for reset/disappear
            names = [p.device for p in serial.tools.list_ports.comports()]
            if name not in names:
                break                           # Board dropped off USB: rebooting
            time.sleep(0.1)
        while time.time() < deadline:           # Phase 2: wait for re-enumeration
            names = [p.device for p in serial.tools.list_ports.comports()]
            if name in names:
                try:
                    return Board(name)          # Try to grab the fresh handle
                except (serial.SerialException, OSError):
                    pass                        # Still re-enumerating: retry
            time.sleep(0.2)
        return None

    a = wait_reboot(port_a)                     # May be None if never appears
    b = wait_reboot(port_b)
    print(f">>> watching {port_a}={a is not None} {port_b}={b is not None}", flush=True)
    time.sleep(seconds)                         # Let boot banners arrive
    if a:
        a.close()
    if b:
        b.close()
    time.sleep(0.2)


def stress(port_a: str, port_b: str, rates, packets: int = 1000):
    """Run a packet-loss stress test for each rate tuple in @p rates.

    @p rates is a list of (arbitration_bps, fd_bps) tuples; fd_bps = 0 means
    classic frames. For every rate: reconfigures BOTH boards (B command),
    resets the receiver counters (R), makes board A blast @p packets
    (T command), waits for TXDONE, then reads the receiver's stats (pkt=,
    miss=) and logs the speed and the error percentage.
    """
    import re                                        # Stats line parsing

    a = Board(port_a)                                # Sender board
    b = Board(port_b)                                # Receiver board
    time.sleep(1.0)                                  # Flush boot banners
    a.clear()
    b.clear()

    print(f"{'bps (arb[:fd])':>17} | {'received':>8} | {'lost':>5} | "
          f"{'err %':>8} | {'proto_errs':>10} | LEC", flush=True)
    print("-" * 70, flush=True)

    for arb, fd in rates:
        bcmd = f"{arb}:{fd}" if fd else f"{arb}"     # Firmware B-command payload

        # Reconfigure both nodes to the target bit rate.
        a.clear()
        b.clear()
        a.write_line(f"B{bcmd}")
        b.write_line(f"B{bcmd}")
        ok_a = a.wait_for(f"BAUD {bcmd} OK", 3.0) is not None
        ok_b = b.wait_for(f"BAUD {bcmd} OK", 3.0) is not None
        if not (ok_a and ok_b):
            print(f"{bcmd:>17} | RECONFIG FAILED (a={ok_a} b={ok_b})", flush=True)
            continue

        # Reset the receiver packet counters and start the burst.
        b.clear()
        b.write_line("R")
        b.wait_for("RST OK", 2.0)

        a.clear()
        a.write_line(f"T{packets}")
        txd = a.wait_for("TXDONE", 120.0)            # Send completion report
        if txd is None:
            print(f"{bcmd:>17} | TX TIMEOUT", flush=True)
            continue

        time.sleep(1.0)                              # Let the last frames land
        b.clear()
        b.write_line("?")                            # Immediate stats request
        stats = b.wait_for("[STATS]", 3.0)
        if stats is None:
            print(f"{bcmd:>17} | STATS TIMEOUT", flush=True)
            continue

        pkt = int(dict(re.findall(r"(\w+)=(\d+)", stats))["pkt"])
        miss = int(dict(re.findall(r"(\w+)=(\d+)", stats))["miss"])
        cel = int(dict(re.findall(r"(\w+)=(\d+)", txd))["cel_delta"])
        lec = (re.search(r"LEC=(\d)", stats) or [None, "?"])[1]
        lost = packets - pkt                         # Packets never received
        err_pct = lost / packets * 100.0             # Error percentage
        print(f"{bcmd:>17} | {pkt:>8} | {lost:>5} | {err_pct:>7.2f}% | "
              f"{cel:>10} | {lec}", flush=True)

    a.close()
    b.close()
    time.sleep(0.2)


if __name__ == "__main__":
    FD_DATA_RATES = [1000000, 2500000, 3400000, 4250000, 5000000]
    if len(sys.argv) < 2 or sys.argv[1] == "list":
        list_ports()
    elif len(sys.argv) == 3:
        interactive(sys.argv[1], sys.argv[2])
    elif len(sys.argv) == 5 and sys.argv[1] == "capture":
        capture(sys.argv[2], sys.argv[3], float(sys.argv[4]))
    elif len(sys.argv) == 4 and sys.argv[1] == "stress":
        stress(sys.argv[2], sys.argv[3],
               [(s, 0) for s in [125000, 250000, 500000, 1000000]])
    elif len(sys.argv) == 5 and sys.argv[1] == "stress":
        stress(sys.argv[2], sys.argv[3],
               [(s, 0) for s in [125000, 250000, 500000, 1000000]],
               int(sys.argv[4]))
    elif len(sys.argv) == 6 and sys.argv[1] == "stress":
        speeds = [int(s) for s in sys.argv[5].split(",")]
        stress(sys.argv[2], sys.argv[3], [(s, 0) for s in speeds], int(sys.argv[4]))
    elif len(sys.argv) == 4 and sys.argv[1] == "stressfd":
        stress(sys.argv[2], sys.argv[3], [(1000000, d) for d in FD_DATA_RATES])
    elif len(sys.argv) == 5 and sys.argv[1] == "stressfd":
        stress(sys.argv[2], sys.argv[3], [(1000000, d) for d in FD_DATA_RATES],
               int(sys.argv[4]))
    elif len(sys.argv) == 4:
        one_shot(sys.argv[1], sys.argv[2], sys.argv[3])
    else:
        print(__doc__)
