#!/usr/local/bin/python3
"""Daily medians per main-unit channel, and which channel's day pattern matches
`Grounding strip`.

The grounding-strip reading is intermittent on a day scale (~8 W some days, ~90 W
others). If it is a borrowed/shared neutral or a switching leakage path, some other
channel should be present on exactly the high days. This looks for that channel.
Correlation over ~9 days is weak statistics, but the contrast is 12x, so a real
match should be obvious rather than marginal.
"""
import statistics
import sys
from datetime import datetime

MAIN_COEFF = 385.0
PORTS = [
    ("Main Red", 18.8), ("Subpanel Red", 18.8), ("Main Black", 18.8),
    ("Subpanel Black", 18.8), ("22 heat exchanger", 50.0),
    ("25 Main A/C compressor", 25.0), ("24", 50.0), ("26", 50.0),
    ("Grounding strip", 50.0), ("28 Air handler attic", 25.0),
    ("38 Vova", 50.0), ("40 Lida", 50.0),
]
GND_IDX = 8


def day_of(ts):
    ts = ts.strip()
    if '-' in ts:
        return ts[:10]
    return datetime.fromtimestamp(float(ts)).strftime('%Y-%m-%d')


days = {}
for path in sys.argv[1:]:
    with open(path) as f:
        for line in f:
            p = line.rstrip("\n").split(",")
            if len(p) != len(PORTS) + 3:
                continue
            try:
                gnd = abs(float(p[-1]))
                vals = [abs(float(p[i + 2])) for i in range(len(PORTS))]
            except ValueError:
                continue
            d = days.setdefault(day_of(p[0]), [[] for _ in PORTS])
            for i, (_, pc) in enumerate(PORTS):
                d[i].append(max(vals[i] - gnd, 0.0) / MAIN_COEFF / pc * 1000.0)

keys = sorted(k for k in days if len(days[k][0]) > 2000)   # drop stub days
med = {k: [statistics.median(c) for c in days[k]] for k in keys}

print(f"{'day':12} " + " ".join(f"{n[:11]:>11}" for n, _ in PORTS))
for k in keys:
    print(f"{k:12} " + " ".join(f"{v:11.1f}" for v in med[k]))

g = [med[k][GND_IDX] for k in keys]
hi = [k for k in keys if med[k][GND_IDX] > 40]
lo = [k for k in keys if med[k][GND_IDX] <= 40]
print(f"\nhigh days ({len(hi)}): {', '.join(hi)}")
print(f"low  days ({len(lo)}): {', '.join(lo)}")

print(f"\n{'channel':24} {'hi mean':>9} {'lo mean':>9} {'ratio':>8}  {'r vs gnd':>9}")
gm = statistics.fmean(g)
gv = sum((x - gm) ** 2 for x in g)
for i, (name, _) in enumerate(PORTS):
    v = [med[k][i] for k in keys]
    hm = statistics.fmean([med[k][i] for k in hi]) if hi else float('nan')
    lm = statistics.fmean([med[k][i] for k in lo]) if lo else float('nan')
    vm = statistics.fmean(v)
    vv = sum((x - vm) ** 2 for x in v)
    cov = sum((v[j] - vm) * (g[j] - gm) for j in range(len(keys)))
    r = cov / (vv * gv) ** 0.5 if vv > 0 and gv > 0 else float('nan')
    ratio = hm / lm if lm > 0.05 else float('inf')
    print(f"{name:24} {hm:9.1f} {lm:9.1f} {ratio:8.2f}  {r:9.3f}")
