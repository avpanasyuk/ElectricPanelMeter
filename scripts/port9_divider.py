#!/usr/local/bin/python3
"""Is port 9 still coupled to the service-neutral unbalance?

HOUSE_POWER's claim: port 9 acted as a current divider on our own neutral (share
~0.28, r ~0.92 in 2022-23) and today reads ~0.003 with r ~0.06 -- i.e. the CT is no
longer on that path. That claim is what decides whether re-seating the CT is worth
doing, so check it here rather than take it.

Regression of port 9 watts on |Main Red - Main Black| watts, per file.
"""
import statistics
import sys

COEFF = 385.0
P = {'red': (1, 18.8), 'black': (3, 18.8), 'gnd_strip': (9, 50.0)}   # 1-based port -> coeff


def watts(vals, gnd, port, pc):
    return max(abs(vals[port]) - gnd, 0.0) / COEFF / pc * 1000.0


for path in sys.argv[1:]:
    xs, ys = [], []
    with open(path) as f:
        for k, line in enumerate(f):
            if k % 7:                     # sample; plenty for a correlation
                continue
            p = line.rstrip("\n").split(",")
            if len(p) != 15:
                continue
            try:
                v = [float(x) for x in p[1:]]
            except ValueError:
                continue
            g = abs(v[-1])
            unbal = abs(watts(v, g, P['red'][0], P['red'][1])
                        - watts(v, g, P['black'][0], P['black'][1]))
            xs.append(unbal)
            ys.append(watts(v, g, P['gnd_strip'][0], P['gnd_strip'][1]))
    n = len(xs)
    if n < 500:
        print(f"{path.split('/')[-1]:38} n={n} (too few)")
        continue
    mx, my = statistics.fmean(xs), statistics.fmean(ys)
    sxx = sum((x - mx) ** 2 for x in xs)
    syy = sum((y - my) ** 2 for y in ys)
    sxy = sum((xs[i] - mx) * (ys[i] - my) for i in range(n))
    r = sxy / (sxx * syy) ** 0.5 if sxx > 0 and syy > 0 else float('nan')
    slope = sxy / sxx if sxx > 0 else float('nan')
    print(f"{path.split('/')[-1]:38} n={n:7}  slope={slope:7.3f}  r={r:6.3f}  "
          f"median unbal={statistics.median(xs):7.1f} W  median p9={statistics.median(ys):7.1f} W")
