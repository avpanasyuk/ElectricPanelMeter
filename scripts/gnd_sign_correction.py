#!/usr/local/bin/python3
"""How much every past main-board figure moves under the signed-GND fix, month by month.

`read_file.m` used to compute `max(|col| - |gnd|, 0)` per row. The GND column is a
grounded mux input carrying the additive offset common to every channel, so subtracting
it from the MAGNITUDE is the wrong sign on a channel whose CT is reversed -- Subpanel
Red, Main Black, `26` -- understating it by 2*gnd. Main Black is a feeder, so every
mains total ever published from this archive is low by that much, and the error scales
with gnd, which stepped at the 2025-12 firmware rebuild.

Emits one CSV row per month: old and new watts for the two feeders and the Grounding
strip, so a consumer (HOUSE_POWER's bill calibration) can correct rather than re-derive.
Writes CSV to stdout; run on bsd, it reads the whole archive and takes ~10 min.
"""
import csv
import glob
import os
import sys

COEFF = 385.0                                    # conf_main_v0.m board coefficient
PC = [18.8] * 4 + [50, 25, 50, 50, 50, 25, 50, 50]   # per-port CT coefficients
RED, BLACK, GNDSTRIP = 0, 2, 8
MIN_ROWS = 500

files = sorted(glob.glob('/POOL/ARCHIVE/ESP_LOGS/PowerMonitor.v0.*.main.csv'),
               key=lambda p: (os.path.basename(p).split('.')[3],     # YY
                              os.path.basename(p).split('.')[2]))    # MM

w = csv.writer(sys.stdout, lineterminator='\n')
w.writerow(['month', 'rows', 'gnd_counts',
            'old_main_red_W', 'new_main_red_W',
            'old_main_black_W', 'new_main_black_W',
            'old_mains_W', 'new_mains_W', 'mains_correction_pct',
            'old_gnd_strip_W', 'new_gnd_strip_W'])

for path in files:
    n = 0
    s_old = [0.0] * 12
    s_new = [0.0] * 12
    s_gnd = 0.0
    with open(path, errors='ignore') as f:
        for line in f:
            p = line.rstrip('\n').split(',')
            if len(p) != 15:
                continue
            try:
                v = [float(x) for x in p[2:15]]
            except ValueError:
                continue
            g = v[12]
            n += 1
            s_gnd += g
            for i in range(12):
                s_old[i] += max(abs(v[i]) - abs(g), 0.0)   # what read_file.m used to do
                s_new[i] += v[i] - g                       # signed; magnitude taken below
    if n < MIN_ROWS:
        continue
    old = [s_old[i] / n * 1000.0 / COEFF / PC[i] for i in range(12)]
    new = [abs(s_new[i] / n) * 1000.0 / COEFF / PC[i] for i in range(12)]
    om, nm = old[RED] + old[BLACK], new[RED] + new[BLACK]
    b = os.path.basename(path).split('.')
    w.writerow([f"20{b[3]}-{b[2]}", n, round(s_gnd / n, 1),
                round(old[RED], 1), round(new[RED], 1),
                round(old[BLACK], 1), round(new[BLACK], 1),
                round(om, 1), round(nm, 1), round(100 * (nm - om) / om, 2) if om else '',
                round(old[GNDSTRIP], 1), round(new[GNDSTRIP], 1)])
