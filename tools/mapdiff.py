#!/usr/bin/env python3
"""
@file mapdiff.py
@brief Compare GNU ld map files by symbol size, ignoring addresses and object-file attribution.

A refactor that only moves code between translation units must not change what the linker
emits. firmware.bin is not byte-reproducible (it embeds build timestamps and the link order
shuffles addresses), so byte-comparing images proves nothing. What IS stable across a pure
refactor is the set of emitted symbols and the size of each one.

This tool extracts `symbol -> size` from a map and diffs two of them, reporting symbols that
were added, removed, or changed size. Address changes and moves between .o files are ignored
by design: after the BSP extraction a symbol legitimately migrates from one TU to another.

Usage:
    mapdiff.py BASE.map NEW.map            # diff two maps
    mapdiff.py --print-sizes MAP           # dump symbol -> size, sorted (for CI artifacts)
    mapdiff.py BASE.map NEW.map --allow jump_to_system_bootloader --allow SystemClock_Config

Exit status is 1 when unexpected deltas remain after --allow filtering, so CI can gate on it.

Copyright 2026, creapunk
Licensed under the Apache License, Version 2.0
"""

import argparse                              # Command line parsing
import re                                    # Map line matching
import sys                                   # Exit codes

# A section line is either "  .text.symbol" alone (size on the following line) or
# "  .text.symbol   0xADDR   0xSIZE  file.o" complete on one line.
SECTION_RE = re.compile(r'^\s+\.(text|rodata|data|bss)\.(\S+)\s*(.*)$')
# Trailing part of a complete line: address, size, then the object file.
INLINE_RE = re.compile(r'^0x[0-9a-f]+\s+0x([0-9a-f]+)\s')
# A continuation line carrying only the address and size for the section named above it.
CONT_RE = re.compile(r'^\s+0x[0-9a-f]+\s+0x([0-9a-f]+)\s')


def parse_map(path):
    """Return {symbol: total_size} for every sized .text/.rodata/.data/.bss section."""
    sizes = {}                               # Accumulated size per symbol
    pending = None                           # Symbol whose size is on the next line
    with open(path, 'r', errors='replace') as fh:
        for line in fh:
            if pending is not None:          # Previous line named a section with no size yet
                cont = CONT_RE.match(line)
                if cont:
                    sizes[pending] = sizes.get(pending, 0) + int(cont.group(1), 16)
                pending = None               # One continuation line only
                if not cont:
                    pass                     # Fall through: this line may open a new section
            match = SECTION_RE.match(line)
            if not match:
                continue
            symbol = match.group(2)          # Symbol name after the section prefix
            rest = match.group(3).strip()    # Whatever followed on the same line
            inline = INLINE_RE.match(rest)
            if inline:
                sizes[symbol] = sizes.get(symbol, 0) + int(inline.group(1), 16)
            else:
                pending = symbol             # Size arrives on the following line
    return sizes


def main():
    ap = argparse.ArgumentParser(description='Diff two ld map files by symbol size.')
    ap.add_argument('maps', nargs='+', help='BASE.map NEW.map, or one map with --print-sizes')
    ap.add_argument('--print-sizes', action='store_true', help='dump symbol -> size and exit')
    ap.add_argument('--allow', action='append', default=[],
                    help='substring of a symbol whose delta is expected (repeatable)')
    args = ap.parse_args()

    if args.print_sizes:
        for sym, size in sorted(parse_map(args.maps[0]).items()):
            print(f'{size:8d}  {sym}')
        return 0

    if len(args.maps) != 2:
        ap.error('need BASE.map and NEW.map')
    base, new = parse_map(args.maps[0]), parse_map(args.maps[1])

    added = sorted(set(new) - set(base))                       # Symbols only in the new build
    removed = sorted(set(base) - set(new))                     # Symbols only in the baseline
    changed = sorted(s for s in set(base) & set(new) if base[s] != new[s])

    def allowed(sym):
        return any(a in sym for a in args.allow)

    unexpected = 0                                             # Deltas not covered by --allow
    for sym in removed:
        mark = 'ok ' if allowed(sym) else 'NEW'
        unexpected += 0 if allowed(sym) else 1
        print(f'[{mark}] removed  {base[sym]:6d} -> {"":6}  {sym}')
    for sym in added:
        mark = 'ok ' if allowed(sym) else 'NEW'
        unexpected += 0 if allowed(sym) else 1
        print(f'[{mark}] added    {"":6} -> {new[sym]:6d}  {sym}')
    for sym in changed:
        mark = 'ok ' if allowed(sym) else 'NEW'
        unexpected += 0 if allowed(sym) else 1
        print(f'[{mark}] resized  {base[sym]:6d} -> {new[sym]:6d}  {sym}')

    print(f'\nbase total {sum(base.values())} B in {len(base)} symbols'
          f' | new total {sum(new.values())} B in {len(new)} symbols')
    print(f'{len(added)} added, {len(removed)} removed, {len(changed)} resized,'
          f' {unexpected} unexpected')
    return 1 if unexpected else 0


if __name__ == '__main__':
    sys.exit(main())
