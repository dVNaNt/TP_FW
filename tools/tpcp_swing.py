#!/usr/bin/env python3
"""Swing a TunePulse drive back and forth forever.

The script selects MOTION mode, arms the output stage, applies the motion
limits (cruise velocity, acceleration, jerk) to the motion_limits register and
then queues jerk-limited S-curve point-to-point moves between two positions,
waiting for each leg to settle before reversing. It runs until interrupted
(Ctrl+C), then holds position and disarms the stage.

Defaults restore the exact manual sequence used to validate the drive:
10 revolutions at 5 rev/s with a 0.5 s pause on every reversal.

Wire format and helpers come from tpcp_host.py (the TPCP v1 master); this
script is a thin parameterized loop on top of it.

Usage
-----
    python tools/tpcp_swing.py                        # 10 rev, 5 rev/s, forever
    python tools/tpcp_swing.py --rev 5 --velocity 3 --accel 20 --jerk 200
    python tools/tpcp_swing.py --cycles 10 --delay 1.0
    python tools/tpcp_swing.py --node 1 --interface hid

Copyright 2026, creapunk
Licensed under the Apache License, Version 2.0
"""

import argparse
import sys
import time
import types

# Reuse the host-side master: transports, frame builders, register codec.
try:
    import can  # noqa: F401                       # Present on a bench with a CAN adapter
except ImportError:
    # tpcp_host imports python-can unconditionally and exits if it is missing.
    # HID does not need it: install a stand-in module whose every attribute
    # raises, so the failure only surfaces if a CAN interface is ever selected.
    class _MissingCan(types.ModuleType):
        def __getattr__(self, name):
            raise RuntimeError("python-can is not installed: only the 'hid' and 'serial' "
                               "transports are available (pip install python-can for CAN)")
    sys.modules["can"] = _MissingCan("can")

import tpcp_host as tp



def parse_args():
    """Command line: every knob of the swing profile."""
    p = argparse.ArgumentParser(
        description="Swing a TunePulse drive back and forth between two positions.")
    p.add_argument("--node", type=int, default=1, help="drive node address (default 1)")
    p.add_argument("--interface", default="auto", choices=["auto", "hid"],
                   help="transport (default: auto-detect HID)")
    p.add_argument("--rev", type=float, default=10.0,
                   help="stroke length in revolutions (default 10)")
    p.add_argument("--velocity", type=float, default=5.0,
                   help="cruise velocity in rev/s (default 5)")
    p.add_argument("--accel", type=float, default=None,
                   help="acceleration limit written to motion_limits (None = keep)")
    p.add_argument("--jerk", type=float, default=None,
                   help="jerk limit written to motion_limits (None = keep)")
    p.add_argument("--delay", type=float, default=0.5,
                   help="dwell on each reversal in seconds (default 0.5)")
    p.add_argument("--cycles", type=int, default=0,
                   help="number of back-and-forth cycles, 0 = forever (default 0)")
    p.add_argument("--tolerance", type=float, default=0.01,
                   help="settle window in revolutions (default 0.01)")
    p.add_argument("--timeout", type=float, default=60.0,
                   help="per-leg settle timeout in seconds (default 60)")
    return p.parse_args()


def open_link(interface):
    """Open the HID control interface (the only transport a bench drive needs)."""
    path = tp.find_hid_path(None) if interface == "auto" else None
    if path is None and interface == "hid":
        sys.exit("no TunePulse HID interface found")
    args = types.SimpleNamespace(interface="hid", channel=path, verbose=False)
    return tp.HidLink(args)


def read_state(link, node, timeout=0.5):
    """Return the drive's state name, or None when it does not answer."""
    _, vals = tp.read_register(link, node, "system_status", timeout=timeout)
    if vals is None:
        return None
    return tp.STATES[(vals["fault_word"] >> 16) & 0xF]


def wait_state(link, node, want=("idle", "ready", "running"), timeout=10.0):
    """Block until the drive reports one of the given states."""
    start = time.time()
    while time.time() - start < timeout:
        state = read_state(link, node, timeout=0.3)
        if state in want:
            return True
        time.sleep(0.2)
    return False


def setup(link, node, args):
    """Fault reset if needed, apply motion limits, select MOTION mode, arm."""
    state = read_state(link, node)
    if state is None:
        sys.exit(f"node {node} is not answering")
    if state == "calibrating":
        # A calibration in flight must finish (or be cleared) before any move.
        print("  drive is calibrating, waiting...")
        wait_state(link, node)
        state = read_state(link, node)
    if state == "fault":
        tp.system_control(link, node, action=tp.SYS_FAULT_RESET)
        wait_state(link, node)

    # Motion limits: read-modify-write, so unspecified fields keep their value.
    limits = {"velocity": args.velocity}
    if args.accel is not None:
        limits["accel"] = args.accel
    if args.jerk is not None:
        limits["jerk"] = args.jerk
    if tp.write_fields(link, node, "motion_limits", **limits):
        print(f"  motion_limits: {', '.join(f'{k}={v:g}' for k, v in limits.items())}")
    else:
        print("  motion_limits: write rejected, using the stored values")

    # Queue segments only advance in MOTION mode with the stage enabled.
    if not tp.write_fields(link, node, "motor_props_1", mode=tp.MODES.index("motion")):
        sys.exit("mode: rejected")
    reply = tp.system_control(link, node, enable=tp.SYS_ENABLE)
    if reply is None or reply[0] == tp.CMD["nack"]:
        sys.exit("enable: rejected")
    print(f"  mode=motion, stage enabled, swinging 0 <-> {args.rev:g} rev")


def swing(link, node, args):
    """Queue alternating S-curve legs until interrupted or the cycle budget ends."""
    seg, leg, cycles = 1, 0, 0
    targets = (args.rev, 0.0)  # absolute endpoints: forward stroke, then home
    try:
        while args.cycles == 0 or cycles < args.cycles:
            target = targets[leg % 2]
            frames = [tp.mt1(seg, tp.MT_MODES["scurve"], is_abs=True, pos_mode=1, is_vel=True),
                      tp.mt2(seg, target),
                      tp.mt3(seg, velocity=args.velocity)]
            link.send_many(tp.ID_COMMAND + node, frames)
            if not tp.wait_settled(link, node, args.tolerance, args.timeout):
                print("  leg did not settle, stopping the loop")
                return False
            leg += 1
            if leg % 2 == 0:
                cycles += 1  # a full cycle is one out-and-back
            print(f"  cycle {cycles if cycles else '-'} leg {leg}: at {target:+g} rev")
            time.sleep(args.delay)
            seg = seg % 250 + 1  # segment ids wrap below the 255 protocol cap
    except KeyboardInterrupt:
        pass
    return True


def shutdown(link, node):
    """Hold position and disarm the output stage."""
    link.send(tp.ID_COMMAND + node, tp.mt1(0, tp.MT_MODES["hold"]))
    tp.system_control(link, node, enable=tp.SYS_DISABLE)
    print("  stopped: hold + disabled")


def main():
    args = parse_args()
    link = open_link(args.interface)
    try:
        setup(link, args.node, args)
        ok = swing(link, args.node, args)
    finally:
        try:
            shutdown(link, args.node)
        finally:
            link.close()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
