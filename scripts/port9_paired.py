#!/usr/local/bin/python3
"""Level regression vs paired-difference estimator for the GEC share.

Two channels that both follow the daily household rhythm correlate through that shared
common mode whether or not they are physically coupled. Differencing removes anything
slowly varying and leaves only what moves together minute to minute, so the two
estimators agree when the coupling dominates and diverge when it does not. The
divergence is the diagnostic, which is why both are computed here.

Rows are ~5 s, so they are binned to 1-minute means first, then consecutive minutes
differenced. Pairs are kept only where the unbalance step is large enough to carry
signal (threshold in units of that month's sigma of d-unbalance).
"""
import statistics
import sys
from datetime import datetime

COEFF = 385.0
RED, BLACK, GS = 1, 3, 9
C_FEED, C_GS = 18.8, 50.0
SIGMA_K = 3.0


def w(vals, gnd, port, pc):
    # Signed GND subtraction (it is a common additive offset), magnitude last -- as
    # read_file.m does it. Against the old max(|col|-gnd,0) this shifts the unbalance by
    # a constant 2*gnd, so every SLOPE here is unchanged; only intercepts move.
    return abs(vals[port] - gnd) / COEFF / pc * 1000.0


def minute_of(ts):
    ts = ts.strip()
    if '-' in ts:
        return ts[:16]
    return datetime.fromtimestamp(float(ts)).strftime('%Y-%m-%d %H:%M')


def reg(xs, ys):
    n = len(xs)
    if n < 30:
        return float('nan'), float('nan'), n
    mx, my = statistics.fmean(xs), statistics.fmean(ys)
    sxx = sum((v - mx) ** 2 for v in xs)
    syy = sum((v - my) ** 2 for v in ys)
    sxy = sum((xs[i] - mx) * (ys[i] - my) for i in range(n))
    if sxx <= 0 or syy <= 0:
        return float('nan'), float('nan'), n
    return sxy / sxx, sxy / (sxx * syy) ** 0.5, n


for path in sys.argv[1:]:
    bins = {}
    with open(path) as f:
        for line in f:
            p = line.rstrip("\n").split(",")
            if len(p) != 15:
                continue
            try:
                v = [float(x) for x in p[1:]]
            except ValueError:
                continue
            g = float(v[-1])
            unbal = abs(w(v, g, RED, C_FEED) - w(v, g, BLACK, C_FEED))
            b = bins.setdefault(minute_of(p[0]), [[], []])
            b[0].append(unbal)
            b[1].append(w(v, g, GS, C_GS))

    mins = sorted(bins)
    U = [statistics.fmean(bins[m][0]) for m in mins]
    G = [statistics.fmean(bins[m][1]) for m in mins]

    lv_slope, lv_r, lv_n = reg(U, G)

    # consecutive minutes only
    du, dg = [], []
    for i in range(len(mins) - 1):
        t0 = datetime.strptime(mins[i], '%Y-%m-%d %H:%M')
        t1 = datetime.strptime(mins[i + 1], '%Y-%m-%d %H:%M')
        if (t1 - t0).total_seconds() != 60:
            continue
        du.append(U[i + 1] - U[i])
        dg.append(G[i + 1] - G[i])
    if len(du) > 30:
        sd = statistics.pstdev(du)
        keep = [i for i in range(len(du)) if abs(du[i]) > SIGMA_K * sd]
        d_slope, d_r, d_n = reg([du[i] for i in keep], [dg[i] for i in keep])
    else:
        d_slope = d_r = float('nan'); d_n = len(du)

    name = path.split('/')[-1]
    print(f"{name:34} min={len(mins):6}  LEVEL slope={lv_slope:6.3f} r={lv_r:6.3f}   "
          f"DIFF slope={d_slope:6.3f} r={d_r:6.3f} n={d_n:5}")
