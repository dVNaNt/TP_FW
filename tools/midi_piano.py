#!/usr/bin/env python3
"""MIDI piano micro-GUI for a TunePulse drive: play every note, waveform and mode.

A small tkinter front end over tools/tpcp_host.py. It speaks the same 0xD0
MIDI Control command as the firmware (see api.md), so every knob here maps
one-to-one onto a frame field:

    Waveform  -> frame byte 1 low nibble   (sine / square / triangle / sawtooth)
    Envelope  -> frame byte 1 high nibble  (linear / exponential / percussive / organ)
    Effect    -> frame byte 1 high nibble  (none / tremolo / vibrato / trill)
    Volume    -> byte 3, Attack / Decay    -> bytes 6 / 7

Playing:
  * click the keys with the mouse, or use the PC keyboard -
        lower row:  Z S X D C V G B H N J M   (white / black of the base octave)
        upper row:  Q 2 W 3 E R 5 T 6 Y 7 U   (the octave above)
  * Note length 0 holds the note until the key is released (or the next one);
    a positive length makes each press a self-terminating one-shot.
  * "Play .mid" streams a standard MIDI file through the coils exactly like
    `tpcp_host.py music`, using the currently selected timbre and effects.

Transport is whatever the drive offers: the vendor HID pipe (the fast path the
desktop tooling uses), the USB CDC console, or a CAN adapter via python-can.

    python tools/midi_piano.py                    # auto-detect, node 1
    python tools/midi_piano.py --channel COM7     # pin the serial port
    python tools/midi_piano.py --node 2           # talk to another drive

Copyright 2026, creapunk
Licensed under the Apache License, Version 2.0
"""

import argparse
import queue
import struct
import sys
import threading
import time
import types
from pathlib import Path

import tkinter as tk
from tkinter import filedialog, messagebox, ttk

try:
    import tpcp_host as tp
except SystemExit:
    # tpcp_host refuses to import without python-can. Only the CAN interface
    # actually needs it, so stub the module out and let HID / serial work;
    # picking a CAN interface later raises a clear error in connect().
    sys.modules["can"] = types.ModuleType("can")
    import tpcp_host as tp

# ----------------------------------------------------------------------------
# Note names for the key captions
# ----------------------------------------------------------------------------
NOTE_NAMES = ["C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"]

# PC keyboard -> semitone offset from the base octave (two rows, one octave each)
KEY_LOW = {"z": 0, "s": 1, "x": 2, "d": 3, "c": 4, "v": 5, "g": 6, "b": 7,
           "h": 8, "n": 9, "j": 10, "m": 11}
KEY_HIGH = {"q": 12, "2": 13, "w": 14, "3": 15, "e": 16, "r": 17, "5": 18,
            "t": 19, "6": 20, "y": 21, "7": 22, "u": 23}
KEYMAP = {**KEY_LOW, **KEY_HIGH}
# Layout-independent key map: tkinter's keysym follows the OS layout, so on a
# Russian layout the physical Z arrives as Cyrillic and the letter lookup
# fails. The Windows vkey code (event.keycode) does not - it is the ASCII code
# of the key's printed character regardless of layout. Digits map the same way.
KEYCODES = {ord(ch.upper()): semi for ch, semi in KEYMAP.items()}

WHITE_PATTERN = [0, 2, 4, 5, 7, 9, 11]           # Semitones of the white keys
BLACK_AFTER = {0: 1, 2: 3, 5: 6, 7: 8, 9: 10}    # White index -> black semitone to its right

OCTAVE_KEYS = 2                                  # White-key octaves drawn on the canvas


def note_freq_hz(note):
    """MIDI note number -> frequency [Hz] (69 = A4 = 440 Hz)."""
    return 440.0 * (2.0 ** ((note - 69) / 12.0))


# ============================================================================
# Polyphonic MIDI file playback
# ============================================================================

DRIVE_VOICES = 3                # The firmware overlaps at most three notes


def parse_midi_poly(path):
    """Parse a standard MIDI file into (start [beats], note, dur [beats]) events.

    The file is pure note data: every timestamp comes back in quarter-note
    beats (tick / division) with NO tempo interpretation - the application
    supplies the beat length. Unlike the monophonic reducer in tpcp_host, this
    keeps every note with its own length, so the drive's three voices can
    reproduce chords and pedals exactly. Format 0 and 1 are supported (tracks
    are merged), Note On / Note Off pairs are matched per pitch, and the
    percussion channel (10) is skipped. See api.md, 'MIDI file playback
    format', for the constraints a file must satisfy.
    """
    data = open(path, "rb").read()
    if data[:4] != b"MThd":
        raise ValueError("not a standard MIDI file")
    hlen, fmt, ntrk, div = struct.unpack(">IHHH", data[4:14])
    if div & 0x8000:
        raise ValueError("SMPTE-timed MIDI files are not supported")
    pos = 8 + hlen
    events = []                                  # (tick, on, pitch)
    tracks = ntrk if fmt != 2 else 1
    for _ in range(tracks):
        if data[pos:pos + 4] != b"MTrk":
            break
        ln = struct.unpack(">I", data[pos + 4:pos + 8])[0]
        i, end, tick, run = pos + 8, pos + 8 + ln, 0, None
        while i < end:
            d, i = tp._vlq(data, i)
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
                l, i = tp._vlq(data, i)
                i += l                           # Meta events carry no notes: skip
            elif st in (0xF0, 0xF7):
                l, i = tp._vlq(data, i)
                i += l
            else:
                hi = st & 0xF0
                n = 2 if hi in (0x80, 0x90, 0xA0, 0xB0, 0xE0) else 1
                d1 = data[i]
                d2 = data[i + 1] if n == 2 else 0
                i += n
                if (st & 0x0F) == 9 and hi in (0x80, 0x90):
                    continue                     # Channel 10: percussion, skip
                if hi == 0x90 and d2 > 0:
                    events.append((tick, 1, d1))
                elif hi == 0x80 or (hi == 0x90 and d2 == 0):
                    events.append((tick, 0, d1))
        pos = end
        if fmt == 0:
            break

    notes, open_on = [], {}                      # pitch -> stack of onset ticks
    for tick, on, pitch in sorted(events, key=lambda e: (e[0], -e[1])):
        if on:
            open_on.setdefault(pitch, []).append(tick)
        elif pitch in open_on and open_on[pitch]:
            start = open_on[pitch].pop()         # Close the most recent onset
            if tick > start:
                notes.append((start / div, pitch, (tick - start) / div))
    notes.sort()
    return notes


def play_events(conn, events, wave, mode, volume, bpm=120.0, duration_scale=1.0,
                transpose=0, floor_hz=0.0, attack_pct=0.0, decay_pct=0.0,
                on_progress=None, stop_evt=None):
    """Schedule note events on an open Connection, three voices at a time.

    Events are (start [beats], note, duration [beats]): the application's bpm
    turns beats into seconds and duration_scale stretches every note - the
    file itself carries no tempo. attack_pct / decay_pct shape every note's
    envelope: they are shares of the note's own duration (0..100 %) and travel
    in the 0xD0 frame's attack/decay bytes - Linear and Exponential envelopes
    use both, Percussive ignores the attack (instant hit), Organ ignores both.
    Each note is a self-terminating one-shot (the duration travels in the
    frame), so overlapping notes land on the firmware's three voices by
    themselves. A fourth simultaneous note is dropped - the lowest pitch of
    the four - which keeps the melody line intact in badly-authored files.
    Transpose shifts every note in semitones; floor_hz octave-shifts notes up
    until they clear the frequency floor (0 disables the floor).
    """
    beat_s = 60.0 / bpm
    atk = max(0, min(255, round(attack_pct * 2.55)))    # 0..100 % -> 0..255 share
    dcy = max(0, min(255, round(decay_pct * 2.55)))     # of the note's own duration
    total = len(events)
    done, t0 = 0, time.perf_counter()
    active = []                                  # (end_perf, pitch) sounding now
    for start, pitch, dur in sorted(events):
        if stop_evt is not None and stop_evt.is_set():
            break
        pitch = pitch + transpose
        while floor_hz > 0 and note_freq_hz(pitch) < floor_hz and pitch <= 102:
            pitch += 12                          # Floor: lift by octaves
        if not (29 <= pitch <= 90):              # Outside the drive's 40..2000 Hz
            done += 1
            continue
        target = t0 + start * beat_s
        delay = target - time.perf_counter()
        if delay > 0:
            time.sleep(delay)
        now = time.perf_counter()
        active = [(e, p) for (e, p) in active if e > now]   # Expire finished voices
        if len(active) >= DRIVE_VOICES:
            lowest = min(active + [(target + dur * beat_s, pitch)], key=lambda e: e[1])
            if lowest[1] == pitch:
                continue                         # The newcomer is the least important
            active.remove(lowest)                # Steal: the new note replaces it
        ms = dur * beat_s * 1000.0 * duration_scale
        ok = conn.send_midi(pitch, wave, mode, volume, ms, atk, dcy)
        active.append((time.perf_counter() + dur * beat_s, pitch))
        done += 1
        if on_progress is not None:
            on_progress(done, total)
    conn.note_off()                              # Release whatever still sounds


# ============================================================================
# Connection layer
# ============================================================================

class Connection:
    """Owns the transport link and serializes every protocol exchange.

    GUI callbacks never touch the link directly: they push ('midi', note, ...)
    tuples into a queue and a worker thread performs the (possibly slow) frame
    exchange, so a lost drive can never freeze the UI thread.
    """

    def __init__(self, on_status=None):
        self.link = None                       # SerialLink / HidLink / Link
        self.interface = None                  # 'hid' / 'serial' / can name
        self.node = 1
        self.on_status = on_status             # Optional legacy callback
        self.tx = queue.Queue()                # Frames the worker still has to send
        self.status_q = queue.Queue()          # Status lines for the GUI to poll
        self.lock = threading.Lock()           # Shared with the .mid playback thread
        self.current_held = None               # Held-note frame, kept alive while pressed
        self.worker = threading.Thread(target=self._run, daemon=True)
        self.worker.start()

    KEEPALIVE_S = 0.25                         # Held-note watchdog feed interval

    def _status(self, text):
        """Thread-safe status: queue for the GUI, callback if one was given."""
        self.status_q.put(text)
        if self.on_status is not None:
            try:
                self.on_status(text)
            except Exception:
                pass

    # ---- discovery -------------------------------------------------------

    @staticmethod
    def list_serial_ports():
        """Every COM/dev port pyserial can see, sorted, COM ports first."""
        ports = []
        try:
            import serial.tools.list_ports
            for p in serial.tools.list_ports.comports():
                ports.append(p.device)
        except ImportError:
            pass
        return sorted(ports, key=lambda s: (not s.upper().startswith("COM"), s))

    def auto_detect(self):
        """Probe HID first, then every serial port. Returns (interface, channel)."""
        try:
            path = tp.find_hid_path(None)
        except Exception:
            path = None
        if path:
            return "hid", path
        for port in self.list_serial_ports():
            try:
                found, _ = tp.probe_drive(port)
            except Exception:
                found = False
            if found:
                return "serial", port
        return None, None

    # ---- session ---------------------------------------------------------

    def connect(self, interface, channel, node):
        """Open the transport. Raises on failure; returns a human summary."""
        args = types.SimpleNamespace(interface=interface, channel=channel,
                                     bitrate=500000, data_bitrate=2000000,
                                     fd=False, verbose=False, log=False)
        if interface == "serial":
            link = tp.SerialLink(args)
        elif interface == "hid":
            link = tp.HidLink(args)
        else:
            link = tp.Link(args)
        self.link, self.interface, self.node = link, interface, node
        # Sanity read: a drive that answers Device Info is really there.
        reply = link.request(node, bytes([tp.CMD["device_info"], 0, 0, 0, 0, 0, 0, 0]), timeout=0.5)
        if reply is None:
            self.disconnect(silent=True)
            raise IOError("no response from the drive on this interface")
        return f"connected: {interface} node {node}"

    def disconnect(self, silent=False):
        """Silence the coils, release the transport."""
        self.stop()
        if self.link is not None:
            try:
                self.link.close()
            except Exception:
                pass
            self.link = None
            if not silent:
                self.on_status("disconnected")

    def connected(self):
        return self.link is not None

    # ---- sending ---------------------------------------------------------

    def send_midi(self, note, wave, mode, volume, duration_ms, attack, decay):
        """Thread-safe single MIDI frame exchange with one self-healing retry.

        A drive that latched a fault or dropped into its safe state answers
        NACK or stays silent; a fault reset plus re-energize clears that
        without touching anything else, and then the same frame goes once more.
        """
        with self.lock:
            if self.link is None:
                return False
            payload = tp.midi_frame(note, volume, duration_ms, attack, decay, wave, mode)
            reply = self.link.request(self.node, payload, timeout=0.3)
            if (reply is None or reply[0] == tp.CMD["nack"]) and note != 0:
                self._revive_locked()                  # Self-heal once, then retry
                reply = self.link.request(self.node, payload, timeout=0.3)
        return reply is not None and reply[0] != tp.CMD["nack"]

    def _revive_locked(self):
        """Fault reset + re-energize (caller holds the lock)."""
        try:
            tp.system_control(self.link, self.node, action=tp.SYS_FAULT_RESET)
            tp.system_control(self.link, self.node, enable=tp.SYS_ENABLE)
        except Exception:
            pass

    def prepare(self):
        """Make the drive playable after (re)boot: in a worker thread."""
        threading.Thread(target=self._prepare, daemon=True).start()

    def _prepare(self):
        """Fault reset, full calibration when needed, then energize.

        After a power cycle the drive is uncalibrated and every note is
        refused with NOT_CALIBRATED. This runs the routine the firmware
        would run by itself, then clears the latched result flag - a finished
        calibration that latched 'failed' otherwise leaves the motor state
        machine stuck and the coils silent.
        """
        with self.lock:
            if self.link is None:
                return
            self._status("preparing: fault reset")
            tp.system_control(self.link, self.node, action=tp.SYS_FAULT_RESET)
            _, motor = tp.read_register(self.link, self.node, "motor_status")
            if motor and motor.get("calibrated"):
                tp.system_control(self.link, self.node, enable=tp.SYS_ENABLE)
                self._status("drive ready (already calibrated)")
                return
            self._status("preparing: full calibration - keep the shaft free")
            tp.system_control(self.link, self.node, calibration=tp.CAL_BITS["full"])
            for _ in range(240):                       # Up to two minutes
                time.sleep(0.5)
                _, calib = tp.read_register(self.link, self.node, "diag:calibration")
                if calib and calib.get("calibrated") and not calib.get("running"):
                    break
            tp.system_control(self.link, self.node, action=tp.SYS_FAULT_RESET)
            tp.system_control(self.link, self.node, enable=tp.SYS_ENABLE)
            self._status("drive ready")

    def note_on(self, note, wave, mode, volume, duration_ms, attack, decay):
        self.tx.put(("midi", note, wave, mode, volume, duration_ms, attack, decay))

    def note_off(self):
        self.tx.put(("midi", 0, 0, 0, 0, 0.0, 0, 0))

    def stop(self):
        """Panic: silence the note and drop the output stage to its safe state."""
        self.current_held = None
        try:
            with self.lock:
                if self.link is not None:
                    tp.silence(self.link, self.node)
        except Exception:
            pass

    def _run(self):
        """Worker: note frames plus a keepalive for held notes.

        While a key is down and no envelope is running, only the initial frame
        was sent - and nothing else touches the drive, so a master-liveness
        watchdog may drop it into its safe state mid-note. Every KEEPALIVE_S a
        register READ goes out instead: any addressed frame reloads the
        watchdog, and a read never retriggers the note (no envelope/LFO reset).
        """
        while True:
            try:
                item = self.tx.get(timeout=self.KEEPALIVE_S)
            except queue.Empty:
                item = None
            if item is not None:
                if item[0] == "quit":
                    return
                try:
                    _, note, wave, mode, volume, duration_ms, attack, decay = item
                    ok = self.send_midi(note, wave, mode, volume, duration_ms, attack, decay)
                    if note and duration_ms <= 0:
                        self.current_held = (note, wave, mode, volume)
                    else:
                        self.current_held = None       # Timed note: self-terminating
                    name = NOTE_NAMES[note % 12] + str(note // 12 - 1) if note else "rest"
                    hz = note_freq_hz(note) if note else 0.0
                    self._status(f"{name}  {hz:6.1f} Hz  "
                                 f"{'ok' if ok else 'NO RESPONSE / NACK'}")
                except Exception as exc:               # Never let the worker die
                    self._status(f"error: {exc}")
            elif self.current_held is not None:
                try:
                    with self.lock:
                        if self.link is not None:
                            self.link.request(self.node,
                                              tp.REGISTERS["motor_status"].request(),
                                              timeout=0.2)
                except Exception:
                    pass


# ============================================================================
# GUI
# ============================================================================

class PianoApp:
    """One tkinter window: synth controls, a keyboard, .mid playback."""

    def __init__(self, root, args):
        self.root = root
        self.conn = Connection()
        self.octave = tk.IntVar(value=4)           # Base octave (C4 = note 60)
        self.wave = tk.StringVar(value="sine")
        self.env = tk.StringVar(value="linear")
        self.fx = tk.StringVar(value="none")
        self.volume = tk.IntVar(value=180)
        self.attack = tk.IntVar(value=0)
        self.decay = tk.IntVar(value=0)
        self.length_ms = tk.DoubleVar(value=0.0)   # 0 = hold until key release
        self.hold = tk.BooleanVar(value=False)     # Sustain: releases are ignored
        self.held = {}                             # source (mouse / keysym) -> (physical, sent) note
        self.key_down = set()                      # PC keys already processed (auto-repeat guard)
        self.stop_evt = threading.Event()          # .mid playback cancel
        self.player = None                         # Player thread while active

        root.title("TunePulse MIDI piano")
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        self._build_connection()
        self._build_synth()
        self._build_keyboard()
        self._build_player()
        self._build_status()
        # bind_all: the keys work no matter which widget holds focus. The status
        # bar is fed through a queue and polled here (tkinter calls are only
        # safe from the main thread - calling after() from a worker deadlocks).
        root.bind_all("<KeyPress>", self._on_key)
        root.bind_all("<KeyRelease>", self._off_key)
        root.after(80, self._poll_status)
        root.focus_set()

    # ---- construction ----------------------------------------------------

    def _build_connection(self):
        row = ttk.Frame(self.root, padding=4)
        row.pack(fill="x")
        ttk.Label(row, text="Interface").pack(side="left")
        self.iface = tk.StringVar(value="auto")
        ttk.Combobox(row, textvariable=self.iface, width=7, state="readonly",
                     values=["auto", "hid", "serial", "slcan"]).pack(side="left", padx=3)
        ttk.Label(row, text="Port").pack(side="left")
        self.port = tk.StringVar(value="")
        self.port_box = ttk.Combobox(row, textvariable=self.port, width=26,
                                     values=self.conn.list_serial_ports())
        self.port_box.pack(side="left", padx=3)
        ttk.Button(row, text="Refresh", command=self.refresh_ports).pack(side="left", padx=2)
        ttk.Label(row, text="Node").pack(side="left")
        self.node_var = tk.IntVar(value=1)
        ttk.Spinbox(row, from_=1, to=254, width=4, textvariable=self.node_var).pack(side="left", padx=3)
        ttk.Button(row, text="Connect", command=self.connect).pack(side="left", padx=6)
        ttk.Button(row, text="Disconnect", command=self.disconnect).pack(side="left")

    def _build_synth(self):
        row = ttk.Frame(self.root, padding=4)
        row.pack(fill="x")

        def add(label, var, values):
            ttk.Label(row, text=label).pack(side="left")
            ttk.Combobox(row, textvariable=var, width=11, state="readonly",
                         values=values).pack(side="left", padx=(2, 10))

        add("Wave", self.wave, list(tp.WAVEFORMS))
        add("Envelope", self.env, list(tp.ENVELOPES))
        add("Effect", self.fx, list(tp.EFFECTS))

        def slider(label, var, hi):
            ttk.Label(row, text=label).pack(side="left")
            ttk.Scale(row, from_=0, to=hi, variable=var, length=100).pack(side="left", padx=2)
            lbl = ttk.Label(row, text=str(var.get()), width=3)
            lbl.pack(side="left", padx=(0, 8))
            var.trace_add("write", lambda *_, v=var, l=lbl: l.config(text=str(int(v.get()))))

        slider("Vol", self.volume, 255)
        slider("Att", self.attack, 255)
        slider("Dec", self.decay, 255)

        row2 = ttk.Frame(self.root, padding=(4, 0))
        row2.pack(fill="x")
        ttk.Label(row2, text="Octave").pack(side="left")
        ttk.Spinbox(row2, from_=0, to=8, width=3, textvariable=self.octave,
                    command=self.draw_keyboard).pack(side="left", padx=(2, 10))
        ttk.Label(row2, text="Note length [ms] (0 = hold)").pack(side="left")
        ttk.Spinbox(row2, from_=0, to=10000, increment=50, width=6,
                    textvariable=self.length_ms).pack(side="left", padx=(2, 10))
        ttk.Checkbutton(row2, text="Sustain (ignore releases)",
                        variable=self.hold).pack(side="left", padx=10)
        ttk.Button(row2, text="Prepare drive", command=self.conn.prepare).pack(side="right")
        ttk.Button(row2, text="Panic / stop", command=self.panic).pack(side="right", padx=4)

        hint = ttk.Label(self.root, padding=(6, 2), foreground="#555", text=
                         "Play: Z S X D C V G B H N J M  /  Q 2 W 3 E R 5 T 6 Y 7 U (any layout)   ·   "
                         "Up/Down: octave   ·   Space: sustain   ·   Esc: panic   ·   "
                         "Transpose shifts every note   ·   songs: assets/*.mid")
        hint.pack(fill="x")

    def _build_keyboard(self):
        self.keys = tk.Canvas(self.root, height=150, bg="#222", highlightthickness=0)
        self.keys.pack(fill="x", padx=6, pady=6)
        self.keys.bind("<ButtonPress-1>", self._mouse_on)
        self.keys.bind("<ButtonRelease-1>", self._mouse_off)
        self.key_ids = {}                          # canvas item -> midi note
        self.draw_keyboard()

    def _build_player(self):
        row = ttk.Frame(self.root, padding=4)
        row.pack(fill="x")
        ttk.Button(row, text="Play .mid file...", command=self.play_file).pack(side="left")
        ttk.Button(row, text="Stop playback", command=self.stop_file).pack(side="left", padx=6)
        ttk.Label(row, text="Start [s]").pack(side="left", padx=(10, 2))
        self.start_s = tk.DoubleVar(value=0.0)
        ttk.Spinbox(row, from_=0, to=3600, width=6, textvariable=self.start_s).pack(side="left")
        ttk.Label(row, text="Seconds").pack(side="left", padx=(10, 2))
        self.span_s = tk.DoubleVar(value=250.0)
        ttk.Spinbox(row, from_=1, to=3600, width=6, textvariable=self.span_s).pack(side="left")

        row2 = ttk.Frame(self.root, padding=(4, 0))
        row2.pack(fill="x")
        ttk.Label(row2, text="Tempo [bpm]").pack(side="left")
        self.tempo_bpm = tk.DoubleVar(value=120.0)  # The app owns the beat, not the file
        ttk.Spinbox(row2, from_=30, to=300, increment=5, width=5,
                    textvariable=self.tempo_bpm).pack(side="left", padx=(2, 10))
        ttk.Label(row2, text="Duration ×").pack(side="left")
        self.dur_scale = tk.DoubleVar(value=1.0)    # Note-length multiplier for files
        ttk.Spinbox(row2, from_=0.1, to=3.0, increment=0.1, width=4,
                    textvariable=self.dur_scale).pack(side="left", padx=(2, 10))
        ttk.Label(row2, text="Transpose [st]").pack(side="left")
        self.transpose = tk.IntVar(value=0)        # Base shift applied to every note
        ttk.Spinbox(row2, from_=-24, to=24, width=4,
                    textvariable=self.transpose).pack(side="left", padx=(2, 10))
        ttk.Label(row2, text="Floor [Hz] (0 = off)").pack(side="left")
        self.floor_hz = tk.IntVar(value=0)         # Nothing below this reaches the coils
        ttk.Spinbox(row2, from_=0, to=2000, increment=50, width=5,
                    textvariable=self.floor_hz).pack(side="left", padx=(2, 10))

        row3 = ttk.Frame(self.root, padding=(4, 0))
        row3.pack(fill="x")
        ttk.Label(row3, text="File attack %").pack(side="left")
        self.atk_pct = tk.DoubleVar(value=0.0)     # Rise share of each note's length
        ttk.Spinbox(row3, from_=0, to=100, increment=5, width=4,
                    textvariable=self.atk_pct).pack(side="left", padx=(2, 10))
        ttk.Label(row3, text="File decay %").pack(side="left")
        self.dec_pct = tk.DoubleVar(value=0.0)     # Fall share of each note's length
        ttk.Spinbox(row3, from_=0, to=100, increment=5, width=4,
                    textvariable=self.dec_pct).pack(side="left", padx=(2, 10))

    def _build_status(self):
        self.status = ttk.Label(self.root, text="not connected", relief="sunken",
                                anchor="w", padding=(6, 3))
        self.status.pack(fill="x", side="bottom")

    # ---- keyboard drawing --------------------------------------------------

    def draw_keyboard(self):
        """Redraw OCTAVE_KEYS octaves of piano keys, white first, black on top."""
        c = self.keys
        c.delete("all")
        self.key_ids.clear()
        w = max(c.winfo_width(), 700)
        base = self.octave.get() * 12
        whites = OCTAVE_KEYS * 7
        ww = w / whites
        for i in range(whites):                    # White keys
            semitone = WHITE_PATTERN[i % 7] + 12 * (i // 7)
            note = base + semitone
            x0 = i * ww
            rect = c.create_rectangle(x0, 0, x0 + ww, 150, fill="white",
                                      outline="#888", width=1, tags=str(note))
            c.create_text(x0 + ww / 2, 132, text=f"{NOTE_NAMES[semitone % 12]}\n{note}",
                          font=("TkDefaultFont", 7), justify="center")
            self.key_ids[rect] = note
        for i in range(whites - 1):                # Black keys overlap the seams
            if i % 7 in BLACK_AFTER:
                semitone = WHITE_PATTERN[i % 7] + 12 * (i // 7) + BLACK_AFTER[i % 7]
                note = base + semitone
                x0 = (i + 1) * ww
                rect = c.create_rectangle(x0 - ww * 0.32, 0, x0 + ww * 0.32, 95,
                                          fill="black", outline="black", tags=str(note))
                self.key_ids[rect] = note
        for entry in self.held.values():           # Re-highlight anything held
            self._paint(entry[0], True)

    def _is_black(self, note):
        return (note - self.octave.get() * 12) % 12 in (1, 3, 6, 8, 10)

    def _paint(self, note, pressed):
        if pressed:
            fill = "#e66"                          # Red while sounding
        else:
            fill = "black" if self._is_black(note) else "white"
        self.keys.itemconfig(str(note), fill=fill)

    # ---- events ------------------------------------------------------------

    def _mouse_on(self, event):
        note = self.key_ids.get(self.keys.find_closest(event.x, event.y)[0])
        if note is not None:
            self.key_on("mouse", note)

    def _mouse_off(self, _event):
        self.key_off("mouse")

    def _on_key(self, event):
        keysym = event.keysym.lower()
        if keysym == "up":                         # Octave up, redraw the keys
            self.octave.set(min(8, self.octave.get() + 1))
            return
        if keysym == "down":                       # Octave down
            self.octave.set(max(0, self.octave.get() - 1))
            return
        if keysym == "space":                      # Sustain pedal toggle
            if keysym in self.key_down:
                return                             # Auto-repeat guard
            self.key_down.add(keysym)
            self.hold.set(not self.hold.get())
            return
        if keysym == "escape":                     # Panic
            if keysym in self.key_down:
                return
            self.key_down.add(keysym)
            self.panic()
            return
        semi = KEYCODES.get(event.keycode)         # Layout-independent first
        if semi is None:
            semi = KEYMAP.get(keysym)              # Fallback for non-Windows platforms
        if semi is None or keysym in self.key_down:
            return                                 # Auto-repeat guard
        self.key_down.add(keysym)
        self.key_on(keysym, self.octave.get() * 12 + semi)

    def _off_key(self, event):
        keysym = event.keysym.lower()
        self.key_down.discard(keysym)
        if keysym not in ("up", "down", "space", "escape"):
            self.key_off(keysym)

    def key_on(self, source, note):
        """Start one note with the current synth settings.

        Polyphony is capped at the drive's three voices: when they are all
        sounding, the oldest held key is silently retired (its voice is stolen
        on the drive anyway, so the picture stays honest). The transpose knob
        shifts every note, live and from files alike.
        """
        if not self.conn.connected() or source in self.held:
            return
        if len(self.held) >= 3:
            oldest, old_note = next(iter(self.held))   # Dicts keep insertion order
            del self.held[oldest]
            self._paint(old_note[0], False)
        sent = note + self.transpose.get()             # Base-frequency shift
        self.held[source] = (note, sent)
        self._paint(note, True)
        length = 0.0 if self.length_ms.get() <= 0 else self.length_ms.get()
        self.conn.note_on(sent, tp.WAVEFORMS[self.wave.get()],
                          tp.ENVELOPES[self.env.get()] | (tp.EFFECTS[self.fx.get()] << 2),
                          self.volume.get(), length, self.attack.get(), self.decay.get())

    def key_off(self, source):
        """Release one key.

        Staccato notes (length > 0) and sustain mode swallow the release - the
        drive frees those voices by itself. In hold mode the 0xD0 silence is
        global (note 0 clears every voice), so the notes still held are
        re-asserted after it; they restart their envelopes, which reads as a
        natural retrigger instead of a stuck chord.
        """
        entry = self.held.pop(source, None)
        if entry is None:
            return
        physical, sent = entry
        self._paint(physical, False)
        if self.hold.get() or self.length_ms.get() > 0:
            return                                 # Sustained / self-terminating note
        self.conn.note_off()
        for other in self.held.values():           # Re-assert the surviving chord
            self.conn.note_on(other[1], tp.WAVEFORMS[self.wave.get()],
                              tp.ENVELOPES[self.env.get()] | (tp.EFFECTS[self.fx.get()] << 2),
                              self.volume.get(), 0.0, self.attack.get(), self.decay.get())

    def panic(self):
        """Stop everything: playback, notes, and the output stage itself."""
        self.stop_file()
        self.key_down.clear()
        for physical, _sent in self.held.values():
            self._paint(physical, False)
        self.held.clear()
        self.conn.stop()
        self.set_status("panic: silenced, stage disabled")

    # ---- connection actions --------------------------------------------------

    def refresh_ports(self):
        self.port_box["values"] = self.conn.list_serial_ports()

    def connect(self):
        try:
            interface = self.iface.get()
            channel = self.port.get().strip() or None
            if interface == "auto" or not channel:
                self.set_status("searching for the drive...")
                interface, channel = self.conn.auto_detect()
                if interface is None:
                    messagebox.showwarning("TunePulse", "Drive not found.\n"
                                           "Plug it in (or pick interface + port) and retry.")
                    self.set_status("not connected")
                    return
            self.set_status(self.conn.connect(interface, channel, self.node_var.get()))
            self.conn.prepare()                    # Fault reset / calibrate / enable if needed
        except Exception as exc:
            messagebox.showerror("TunePulse", f"Connect failed:\n{exc}")
            self.set_status("not connected")

    def disconnect(self):
        self.stop_file()
        self.conn.disconnect()

    # ---- .mid playback -------------------------------------------------------

    def play_file(self):
        """Stream a MIDI file window with 3-voice polyphony (see api.md format)."""
        if not self.conn.connected():
            messagebox.showwarning("TunePulse", "Connect to a drive first.")
            return
        assets = Path(__file__).resolve().parent.parent / "assets"
        path = filedialog.askopenfilename(
            initialdir=str(assets) if assets.is_dir() else str(Path(__file__).parent),
            filetypes=[("MIDI files", "*.mid *.midi"), ("All", "*.*")])
        if not path:
            return
        try:
            events = parse_midi_poly(path)         # (start beats, note, dur beats)
        except (ValueError, OSError) as exc:
            messagebox.showerror("TunePulse", f"{path}:\n{exc}")
            return
        # Window selection works in seconds of the CURRENT tempo knob: the
        # file only stores beats, so Start/Seconds are converted here.
        beat_s = 60.0 / max(self.tempo_bpm.get(), 1.0)
        start_b = self.start_s.get() / beat_s
        end_b = start_b + self.span_s.get() / beat_s
        events = [(t - start_b, n, d) for t, n, d in events if t + d > start_b and t < end_b]
        if not events:
            messagebox.showwarning("TunePulse", "No notes in the selected window.")
            return
        # Snapshot the timbre: turning knobs mid-song must not glitch the sender.
        wave = tp.WAVEFORMS[self.wave.get()]
        mode = tp.ENVELOPES[self.env.get()] | (tp.EFFECTS[self.fx.get()] << 2)
        volume = self.volume.get()
        bpm, scale = self.tempo_bpm.get(), self.dur_scale.get()
        transpose, floor_hz = self.transpose.get(), self.floor_hz.get()
        atk_pct, dec_pct = self.atk_pct.get(), self.dec_pct.get()
        self.stop_evt.clear()
        self.player = threading.Thread(target=self._play, daemon=True,
                                       args=(events, path, wave, mode, volume,
                                             bpm, scale, transpose, floor_hz,
                                             atk_pct, dec_pct))
        self.player.start()

    def _play(self, events, path, wave, mode, volume, bpm, scale, transpose,
              floor_hz, atk_pct, dec_pct):
        name = path.replace("\\", "/").rsplit("/", 1)[-1]
        self.set_status(f"playing {name}: {len(events)} events")
        play_events(self.conn, events, wave, mode, volume,
                    bpm=bpm, duration_scale=scale, transpose=transpose,
                    floor_hz=floor_hz, attack_pct=atk_pct, decay_pct=dec_pct,
                    stop_evt=self.stop_evt,
                    on_progress=lambda done, total:
                        self.set_status(f"{name}: {done}/{total}"))
        if not self.stop_evt.is_set():
            self.set_status(f"{name}: done")

    def stop_file(self):
        self.stop_evt.set()

    # ---- misc ------------------------------------------------------------------

    def set_status(self, text):
        """Post a status line from ANY thread; the poller below paints it."""
        self.conn.status_q.put(text)

    def _poll_status(self):
        """Drain the status queue.

        tkinter is single-threaded: the previous version called root.after()
        straight from the worker, which deadlocks the event loop after a few
        exchanges - that was the freeze. Worker threads now only enqueue.
        """
        try:
            while True:
                self.status.config(text=self.conn.status_q.get_nowait())
        except queue.Empty:
            pass
        self.root.after(80, self._poll_status)

    def on_close(self):
        try:
            self.panic()
        finally:
            self.conn.tx.put(("quit",))
            self.root.destroy()


def main():
    ap = argparse.ArgumentParser(description="TunePulse MIDI piano micro-GUI")
    ap.add_argument("--interface", default="auto", choices=["auto", "hid", "serial", "slcan"])
    ap.add_argument("--channel", default=None, help="port to open (default: search)")
    ap.add_argument("--node", type=int, default=1)
    ap.add_argument("--smoke", action="store_true", help="build the window, then exit")
    args = ap.parse_args()

    root = tk.Tk()
    app = PianoApp(root, args)
    if args.channel:
        app.port.set(args.channel)
        app.iface.set(args.interface)
    app.node_var.set(args.node)
    if args.smoke:
        root.update_idletasks()                    # Validate layout without a mainloop
        root.destroy()
        print("smoke test ok")
        return 0
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())