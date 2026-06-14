import serial
import time
from collections import Counter, defaultdict

port = "COM29"
ser = serial.Serial(port, 115200, timeout=0.05)
end = time.time() + 20
data = bytearray()
while time.time() < end:
    data += ser.read(4096)
ser.close()

frames = []
i = 0
while i + 12 <= len(data):
    if (
        data[i] == 0xA5
        and data[i + 1] == 0x09
        and (sum(data[i + 2 : i + 11]) & 0xFF) == data[i + 11]
        and 1 <= data[i + 2] <= 6
    ):
        ts = sum(data[i + 3 + j] << (8 * j) for j in range(6))
        freq = data[i + 9] | (data[i + 10] << 8)
        frames.append((i, data[i + 2], ts, freq))
        i += 12
    else:
        i += 1

print("PORT", port, "bytes", len(data), "frames", len(frames))
print("ch_count", dict(Counter(f[1] for f in frames)))

print("RAW_ORDER")
prev = None
for f in frames[:200]:
    gap = "" if prev is None else f[2] - prev[2]
    print(f[0], "CH", f[1], "TS", f[2], "F", f[3], "GAP", gap)
    prev = f

s = sorted(frames, key=lambda x: x[2])
print("SORTED_ADJ")
for a, b in zip(s[:199], s[1:200]):
    print("CH", a[1], "->", b[1], "DT", b[2] - a[2], "TS", a[2], "->", b[2])

print("PAIR_NEAREST")
for ch_a, ch_b in [(1, 2), (3, 4), (5, 6)]:
    a_list = [f for f in s if f[1] == ch_a]
    b_list = [f for f in s if f[1] == ch_b]
    vals = []
    for a in a_list[:80]:
        if b_list:
            b = min(b_list, key=lambda x: abs(x[2] - a[2]))
            if abs(b[2] - a[2]) < 20000:
                vals.append(b[2] - a[2])
    print(ch_a, ch_b, vals[:80])

print("PER_CH_PERIOD")
for ch in range(1, 7):
    ts_list = [f[2] for f in s if f[1] == ch]
    gaps = [b - a for a, b in zip(ts_list, ts_list[1:])]
    if gaps:
        print(ch, "n", len(ts_list), "min", min(gaps), "max", max(gaps), "first", gaps[:30])

print("BURST_GROUPS")
groups = []
cur = []
for f in s:
    if not cur or f[2] - cur[-1][2] < 50000:
        cur.append(f)
    else:
        groups.append(cur)
        cur = [f]
if cur:
    groups.append(cur)
for gi, g in enumerate(groups[:80]):
    base = min(x[2] for x in g)
    items = [(x[1], x[2] - base, x[2], x[3]) for x in g]
    print("G", gi, "base", base, "span", max(x[2] for x in g) - base, "items", items)

print("CH_PAIR_IN_GROUP")
for ch_a, ch_b in [(1, 2), (1, 4), (2, 4)]:
    vals = []
    miss = 0
    for g in groups:
        a = [x for x in g if x[1] == ch_a]
        b = [x for x in g if x[1] == ch_b]
        if a and b:
            vals.append(b[0][2] - a[0][2])
        else:
            miss += 1
    if vals:
        print(ch_a, ch_b, "n", len(vals), "miss", miss, "min", min(vals), "max", max(vals), "vals", vals[:80])
    else:
        print(ch_a, ch_b, "n", 0, "miss", miss)
