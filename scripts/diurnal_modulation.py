#!/usr/local/bin/python3
"""Which main-unit channels carry real load, by MODULATION rather than level.

After the signed-GND fix each channel still keeps a per-channel residual offset of order
±20 W (the GND column is not identical across mux inputs), so a level below ~25 W says
nothing. A residual is CONSTANT, though, so it cannot produce a diurnal shape: compare a
channel's hour-of-day swing against the GND column's own and a real load stands out.

Threshold: a swing >~10x the GND column's is load. Recompute the yardstick per month --
it scales with the offset, which stepped at the 2025-12 flash (~14 counts in 2026-03).

Handles both timestamp formats. The logs switch from epoch floats to human wall-clock at
the 2026-05-25 push era, and a parser that keys on '-' silently drops every earlier row.

    python3 diurnal_modulation.py /POOL/ARCHIVE/ESP_LOGS/PowerMonitor.v0.03.26.main.csv
"""
import statistics
import sys
from datetime import datetime

try:
    from zoneinfo import ZoneInfo
    TZ = ZoneInfo('America/New_York')
except Exception:                      # bsd python without tzdata: hours drift, ratios don't
    TZ = None

COEFF = 385.0
PORTS = [
    ("Main Red", 18.8), ("Subpanel Red", 18.8), ("Main Black", 18.8),
    ("Subpanel Black", 18.8), ("22 heat exch", 50.0), ("25 A/C", 25.0),
    ("24", 50.0), ("26", 50.0), ("Grounding strip", 50.0),
    ("28 Air handler", 25.0), ("38 Vova", 50.0), ("40 Lida", 50.0),
]
GND = len(PORTS)                       # the GND column's own bucket

hours = {i: {h: [] for h in range(24)} for i in range(GND + 1)}
n = 0
for path in sys.argv[1:]:
    with open(path, errors='ignore') as f:
        for line in f:
            p = line.rstrip("\n").split(",")
            if len(p) != len(PORTS) + 3:
                continue
            try:
                ts = p[0]
                h = int(ts[11:13]) if '-' in ts else datetime.fromtimestamp(float(ts), TZ).hour
                vals = [float(x) for x in p[2:]]
            except (ValueError, OSError):
                continue
            g = vals[GND]
            n += 1
            for i in range(GND):
                hours[i][h].append(vals[i] - g)     # signed GND subtraction, as read_file.m
            hours[GND][h].append(g)

if not n:
    sys.exit("no rows parsed -- check the column count and the timestamp format")


def swing(bucket):
    m = [statistics.fmean(bucket[h]) for h in range(24) if bucket[h]]
    return max(m) - min(m), min(m), max(m)


gnd_swing, _, _ = swing(hours[GND])
print(f"{n} rows;  GND reference swing = {gnd_swing:.1f} counts (the yardstick)\n")
print(f"{'channel':18} {'swing':>9} {'x GND':>8} {'min W':>10} {'max W':>10}  verdict")
for i, (name, pc) in enumerate(PORTS):
    sc, mn, mx = swing(hours[i])
    f = 1000.0 / COEFF / pc
    ratio = sc / gnd_swing
    verdict = "load" if ratio > 10 else ("marginal" if ratio > 3 else "TRACKS REFERENCE")
    print(f"{name:18} {sc:9.1f} {ratio:8.1f} {mn*f:10.1f} {mx*f:10.1f}  {verdict}")
