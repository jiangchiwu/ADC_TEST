import serial
import time
from collections import Counter

port = "COM29"
ser = serial.Serial(port, 115200, timeout=0.05)
end = time.time() + 12
data = bytearray()
while time.time() < end:
    data += ser.read(4096)
ser.close()

frames = []
i = 0
while i + 12 <= len(data):
    if data[i] == 0xA5 and data[i + 1] == 0x09 and (sum(data[i + 2:i + 11]) & 0xFF) == data[i + 11] and 1 <= data[i + 2] <= 6:
        ts = sum(data[i + 3 + j] << (8 * j) for j in range(6))
        freq = data[i + 9] | (data[i + 10] << 8)
        frames.append((data[i + 2], ts, freq))
        i += 12
    else:
        i += 1

s = sorted(frames, key=lambda x: x[1])
groups = []
cur = []
for f in s:
    if not cur or f[1] - cur[-1][1] < 50000:
        cur.append(f)
    else:
        groups.append(cur)
        cur = [f]
if cur:
    groups.append(cur)

print("bytes", len(data), "frames", len(frames), "ch_count", dict(Counter(f[0] for f in frames)))
for gi, g in enumerate(groups[:20]):
    base = min(x[1] for x in g)
    print("G", gi, [(x[0], x[1] - base, x[2]) for x in g])
for ch_a, ch_b in [(1, 2), (1, 4), (2, 4)]:
    vals = []
    miss = 0
    for g in groups:
        a = [x for x in g if x[0] == ch_a]
        b = [x for x in g if x[0] == ch_b]
        if a and b:
            vals.append(b[0][1] - a[0][1])
        else:
            miss += 1
    print("PAIR", ch_a, ch_b, "n", len(vals), "miss", miss, "min", min(vals) if vals else None, "max", max(vals) if vals else None, "vals", vals)
