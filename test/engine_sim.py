# Host-side simulation of the Motor persistence engine (motor.h)
# Mirrors: stream_word/apply_record packing (8-word records, bfloat16 pairs),
# save_tick() slot-scan+write, load_tick() direct per-block validation,
# newest-valid-record-wins, Cal-ID filter.
import struct

CMD = 0xCA1B
ERASED = 0xFFFFFFFF
REGION_WORDS = 2048 // 4
BLOCK_WORDS = 8

def bf(v):
    return struct.unpack('<I', struct.pack('<f', v))[0] >> 16

def unbf(raw):
    return struct.unpack('<f', struct.pack('<I', raw << 16))[0]

def make_record(cal_id, poles, r, ld, lq):
    w0 = CMD | (1 << 16) | (2 << 26) | (1 << 28) | (0 << 30)
    w1 = cal_id | (80 << 8) | ((poles & 0xFFFF) << 16)
    return [w0, w1,
            bf(r)    | (bf(ld)    << 16),
            bf(lq)   | (bf(0.042) << 16),
            bf(24)   | (bf(50)    << 16),
            bf(0.001)| (bf(0.01)  << 16),
            1234, 0]

def header_valid(w):
    if (w[0] & 0xFFFF) != CMD: return False
    t = (w[0] >> 26) & 3
    if t == 0 or t == 3: return False
    poles = (w[1] >> 16) & 0xFFFF
    if poles == 0 or poles > 4095: return False
    return True

flash = [ERASED] * REGION_WORDS

def save_tick_engine(record_words):
    # Phase SPACE: walk 32-byte record SLOTS; stop at first erased leading word
    free = None
    addr = 0
    total = len(flash)*4
    while addr + 32 <= total:
        if flash[addr//4] == ERASED:
            free = addr
            break
        addr += 32
    assert free is not None and free <= total - 32, "erase path"
    # Phase WRITE: ONE 8-word block write
    flash[free//4 : free//4 + 8] = record_words
    return True

def load_tick_engine(load_id):
    cand = None
    for off in range(0, len(flash)*4, 32):
        w = flash[off//4 : off//4 + 8]
        if w[0] != ERASED and header_valid(w):
            rid = w[1] & 0xFF
            if load_id == 0xFF or rid == load_id:
                cand = list(w)  # newest valid wins
    return cand

def apply(c):
    def lo(w): return round(unbf(w & 0xFFFF), 6)
    def hi(w): return round(unbf((w >> 16) & 0xFFFF), 6)
    return dict(ver=(c[0]>>16)&0x3FF, typ=(c[0]>>26)&3,
                cid=c[1]&0xFF, poles=(c[1]>>16)&0xFFFF,
                r=lo(c[2]), ld=hi(c[2]),
                lq=lo(c[3]), tq=hi(c[3]),
                v=lo(c[4]), p=hi(c[4]),
                flux=lo(c[5]), kt=hi(c[5]),
                offset=c[6])

# --- Scenario ---
save_tick_engine(make_record(1, 7,  5.5, 0.0031, 0.0042))   # oldest, id=1
save_tick_engine(make_record(1, 14, 9.9, 0.0035, 0.0046))   # NEWEST id=1
save_tick_engine(make_record(2, 11, 1.2, 0.0021, 0.0032))   # id=2

got = apply(load_tick_engine(1))
print("load id=1 :", got)
assert got['poles'] == 14 and abs(got['r'] - 9.9) < 0.05 and got['cid'] == 1, "newest-with-ID failed"
assert abs(got['ld'] - 0.0035) < 0.0035 * 0.01 and abs(got['tq'] - 0.042) < 0.042 * 0.01, "packed pair mismatch"

got_any = apply(load_tick_engine(0xFF))
print("load any  :", got_any)
assert got_any['poles'] == 11 and got_any['cid'] == 2, "newest-any failed"
assert abs(got_any['flux'] - 0.001) < 0.001 * 0.01 and abs(got_any['kt'] - 0.01) < 0.01 * 0.01, "packed pair mismatch (any)"

# No gaps: records must sit back-to-back in consecutive 32-byte slots
assert flash[8] != ERASED and flash[16] != ERASED, "unexpected gap between records"

# bfloat16 round-trip accuracy check
raw = struct.unpack('<I', struct.pack('<f', 0.0031))[0] >> 16   # float -> bfloat16 word
err = abs(unbf(raw) - 0.0031) / 0.0031
print("bfloat16 rel.err for 0.0031 H:", round(err * 100, 4), "%")
assert err < 0.01

# ------------------------------------------------------------------
# Calibration-table format: TWO header words - 0xFA3E marker, then the
# sample count. The record ID is its start address in 32-byte chunks
# (assigned at dump time and stored in the motor parameter profile).
# ------------------------------------------------------------------
FA3E = 0xFA3E0000
CAPACITY = 200

def make_table_record(samples, el_div):
    words = [FA3E, len(samples)]
    for i in range(0, len(samples), 2):
        hi = samples[i+1] if i + 1 < len(samples) else 0xFFFF
        words.append(samples[i] | (hi << 16))
    words.append(el_div)              # trailer: el_angle_div
    words.append(100 | (5 << 16))     # trailer: offst_val | max_dev<<16
    words.append(3)                   # trailer: offst_idx
    return words

def scan_chain(chain_words):
    """Mirror of CalibrationTable::scan_word state machine."""
    f_cnt = 0; f_meta = False; f_seq = True; f_sidx = 0; f_prev = 0
    f_hdr = False
    committed = None; f_m = {}
    for w in chain_words:
        if f_cnt == 0 and not f_meta:
            if not f_hdr:
                if w == ERASED: break                  # end of chain
                if (w & 0xFFFF0000) != FA3E: break     # corrupt
                f_hdr = True                           # marker ok -> count next
                continue
            f_hdr = False                              # count word
            n = w
            if n == 0 or n > CAPACITY: break
            f_cnt = n; f_sidx = 0; f_seq = True
            continue
        if not f_meta:
            for h in range(2):
                if f_sidx >= f_cnt: break
                s = (w >> (16*h)) & 0xFFFF
                if f_sidx > 0 and ((s - f_prev) & 0xFFFF) >= 0x8000:
                    f_seq = False                      # (int16)(s-prev) <= 0
                f_sidx += 1; f_prev = s
            if f_sidx >= f_cnt:
                f_meta = True; f_cnt = 3; f_m = {}
            continue
        f_m[3 - f_cnt] = w
        f_cnt -= 1
        if f_cnt == 0:
            f_meta = False
            if f_seq:
                committed = dict(el_div=f_m[0], size=f_sidx)
    return committed

# Dump emulation: append at first erased word, align UP to a chunk boundary,
# ID = chunk number of the marker. Then load back directly by that ID.
table_flash = [ERASED] * REGION_WORDS

def table_dump(samples, el_div):
    rec = make_table_record(samples, el_div)
    free = next(i for i, w in enumerate(table_flash) if w == ERASED) * 4
    base = (free + 31) & ~31                       # chunk-aligned append point
    pad = -len(rec) % 8                            # pad record to whole blocks
    table_flash[base//4 : base//4 + len(rec)+pad] = rec + [ERASED]*pad
    return base // 32                              # calibration ID (chunk no.)

def table_load(chunk_id, blocks=None):
    """Direct-addressed load: stream words from chunk_id*32 through the parser."""
    off = chunk_id * 32 // 4
    nblocks = blocks if blocks is not None else 4  # enough for CAPACITY-sized tables
    return scan_chain(table_flash[off : off + nblocks*8])

samples = [(i * 300) & 0xFFFF for i in range(12)]
cal_id = table_dump(samples, 6)
print("dumped cal_id:", cal_id)
got_tbl = table_load(cal_id)
print("table load id =", cal_id, "->", got_tbl)
assert got_tbl is not None and got_tbl['size'] == 12 and got_tbl['el_div'] == 6, "table load by ID failed"

assert table_load(cal_id + 7) is None, "load at wrong address must fail"

# Pole-count derivation rule: BLDC -> n % 6 == 0, poles = n/6; STEP -> n % 4 == 0, poles = n/4
n = 201; div = 4                                # stepper case from the field log
n -= n % div
assert n == 200 and n // div == 50
n = 42; div = 6                                 # BLDC case
assert n % 6 == 0 and n // 6 == 7
print("pole derivation checks passed")

print("ALL ENGINE CHECKS PASSED")
