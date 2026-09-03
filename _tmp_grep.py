import io, glob
out = []
hits = 0
for p in glob.glob('C:/Users/Admin/.platformio/packages/framework-arduinoststm32/cores/arduino/**/*.c*', recursive=True):
    try:
        lines = io.open(p, encoding='utf-8', errors='replace').read().splitlines()
    except OSError:
        continue
    for i, l in enumerate(lines):
        if 'SystemClock_Config' in l:
            out.append('%s %d: %s' % (p, i + 1, l.strip()))
            hits += 1
            if hits > 30:
                break
    if hits > 30:
        break
io.open('clk_ctx.txt', 'w').write('\n'.join(out))
print(len(out))
