#!/usr/local/bin/python3
"""When did main port 10 ('28 Air handler attic') stop reading, and does it read at all?

The air handler runs continuously, so 0 W is a fault. The question data can answer is
WHEN: a channel that once read and then stopped is a CT that came off or a breaker that
changed; a channel that never read is a wiring/installation error dating to the build.

Also prints the RAW column next to the GND reference. Watts hide the distinction --
max(|col|-|gnd|,0) floors at zero either way -- but the raw values say whether the input
is carrying a signal at all, or sitting at the same level as the ground channel.
"""
import glob
import os
import statistics
import sys

MAIN_COEFF = 385.0
PORTS = [("Main Red", 18.8), ("Subpanel Red", 18.8), ("Main Black", 18.8),
         ("Subpanel Black", 18.8), ("22 heat exch", 50.0), ("25 A/C", 25.0),
         ("24", 50.0), ("26", 50.0), ("Grounding strip", 50.0),
         ("28 Air handler", 25.0), ("38 Vova", 50.0), ("40 Lida", 50.0)]
WATCH = [4, 5, 9, 10, 11]           # heat exch, A/C, air handler, Vova, Lida
STEP = 17                           # sample rows; plenty for a median, ~17x faster

files = sorted(glob.glob('/POOL/ARCHIVE/ESP_LOGS/PowerMonitor.v0.*.main.csv'),
               key=lambda p: (os.path.basename(p).split('.')[3],      # YY
                              os.path.basename(p).split('.')[2]))     # MM

print(f"{'month':9} {'rows':>8} " +
      " ".join(f"{PORTS[i][0][:14]:>15}" for i in WATCH) +
      f" {'raw p10':>9} {'raw gnd':>9} {'raw p5':>9}")

for path in files:
    b = os.path.basename(path).split('.')
    label = f"20{b[3]}-{b[2]}"
    cols = {i: [] for i in WATCH}
    raw10, rawgnd, raw5 = [], [], []
    n = 0
    with open(path) as f:
        for k, line in enumerate(f):
            if k % STEP:
                continue
            p = line.rstrip("\n").split(",")
            if len(p) != 15:
                continue
            try:
                gnd = abs(float(p[-1]))
                vals = [abs(float(p[i + 2])) for i in range(12)]
            except ValueError:
                continue
            n += 1
            for i in WATCH:
                cols[i].append(max(vals[i] - gnd, 0.0) / MAIN_COEFF / PORTS[i][1] * 1000.0)
            raw10.append(vals[9])
            raw5.append(vals[4])
            rawgnd.append(gnd)
    if n < 100:
        print(f"{label:9} {n:8} (too few rows)")
        continue
    meds = " ".join(f"{statistics.median(cols[i]):15.1f}" for i in WATCH)
    print(f"{label:9} {n:8} {meds} {statistics.median(raw10):9.0f} "
          f"{statistics.median(rawgnd):9.0f} {statistics.median(raw5):9.0f}")
