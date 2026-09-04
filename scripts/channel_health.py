#!/usr/local/bin/python3
"""Per-channel median watts for one merged month, using read_file.m's formula.

Point: is a channel that read dead in 2026-02 still dead now? A dead-looking channel
is either an off breaker or a fallen CT, and only the time course separates them --
a breaker comes back, a fallen CT does not.
"""
import statistics
import sys
from pathlib import Path

# conf_main_v0.m
MAIN_COEFF = 385.0
MAIN_PORTS = [
    ("Main Red", 18.8), ("Subpanel Red", 18.8), ("Main Black", 18.8),
    ("Subpanel Black", 18.8), ("22 - heat exchanger new", 50.0),
    ("25 - Main A/C compressor", 25.0), ("24", 50.0), ("26", 50.0),
    ("Grounding strip", 50.0), ("28 Air handler attic", 25.0),
    ("38 Vova", 50.0), ("40 Lida", 50.0),
]


def channel_stats(path, board_coeff, ports):
    cols = [[] for _ in ports]
    n = 0
    with open(path) as f:
        for line in f:
            p = line.rstrip("\n").split(",")
            if len(p) != len(ports) + 3:      # ts + V + ports + GND
                continue
            try:
                gnd = abs(float(p[-1]))
                vals = [abs(float(p[i + 2])) for i in range(len(ports))]
            except ValueError:
                continue
            n += 1
            for i, (name, pc) in enumerate(ports):
                cols[i].append(max(vals[i] - gnd, 0.0) / board_coeff / pc * 1000.0)
    out = []
    for i, (name, _) in enumerate(ports):
        c = cols[i]
        if not c:
            continue
        c_sorted = sorted(c)
        med = statistics.median(c_sorted)
        p95 = c_sorted[int(0.95 * (len(c_sorted) - 1))]
        live = 100.0 * sum(1 for x in c if x > 5.0) / len(c)
        out.append((name, med, p95, statistics.fmean(c), live))
    return n, out


for month in sys.argv[1:]:
    path = Path(f"/POOL/ARCHIVE/ESP_LOGS/PowerMonitor.v0.{month}.26.main.csv")
    if not path.is_file():
        print(f"{month}: no file")
        continue
    n, stats = channel_stats(path, MAIN_COEFF, MAIN_PORTS)
    print(f"\n=== main 2026-{month}  ({n} rows) ===")
    print(f"{'channel':28} {'median':>9} {'p95':>9} {'mean':>9} {'% >5W':>7}")
    for name, med, p95, mean, live in stats:
        print(f"{name:28} {med:9.1f} {p95:9.1f} {mean:9.1f} {live:7.1f}")
