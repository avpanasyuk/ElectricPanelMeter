---
name: main-panel-channels-needing-a-bench-check
description: "Grounding strip carries real power on alternating days (~90 W, p95 920 W) matching no metered circuit - safety item; 40 Lida and 28 Air handler read dead in every month"
metadata:
  node_type: memory
  type: project
---

Measured 2026-09-04 with `scripts/channel_health.py` and `scripts/gnd_daily.py`
(main unit, `read_file.m`'s formula).

## `Grounding strip` (port 9) — the safety item

Real power on a grounding conductor, **intermittent on a day scale**, not
seasonal and not load-correlated. Daily medians, 2026-08/09: 81, 4.6, 82, 8.6,
9.0, 114, 103, 101, 71, 13.5 W — alternating high/low, ratio 10.2x, p95 to 920 W.
Monthly medians sat at 11–14 W for 2026-01…07, so onset is between 2026-08-01 and
2026-08-25 (inside the window the rotation bug destroyed; it cannot be dated more
finely).

**No metered channel matches the pattern**, so the responsible load is on an
unmetered circuit. Correlation of each channel's daily median against the
grounding strip's: Main Black r = 0.48 (hi/lo ratio 1.27), Subpanel Black 0.31
(1.18), every Red-leg and branch channel |r| < 0.1, and the A/C compressor is
*anti*-correlated (−0.49) — high-ground days are not hot days. The Black-leg lean
on both feeders is the only structural hint.

Crosstalk and a load-dependent GND reference are both **ruled out**, by paired
differences on 1-min means (HOUSE_POWER, 2026-09-04): against 106 compressor steps
>50 W the feeders respond at slope 0.31–0.35 as they must, while `Grounding strip`
gives slope 0.0037, r = 0.023; and 1834 feeder steps >200 W move the GND reference
at slope −0.005, r = −0.078. So low-power channels carry **no** load-scaling floor.

**Next step is physical, and it is the one item here with a safety implication:**
a clamp meter on the actual conductor on a high day. That settles both things data
cannot — whether the clamp is on what `conf_main_v0.m` calls a grounding strip,
and whether the reading is current at all rather than a front-end artefact. Only
after that is it worth ranking causes (neutral-to-ground bond fault, a borrowed
neutral, a switching leakage path).

## `28 Air handler attic` (port 10) — a fault, and it failed TWICE

The user states the attic air handler runs continuously, so any small reading is
wrong. Monthly medians across the whole archive (`scripts/port10_history.py`):

- **2018-12 … 2024-09: 3–20 W**, seasonal (13–20 W Jan–Apr, 8–10 W Jul–Sep).
  An always-on air handler blower is hundreds of watts, so **the channel never
  measured the load correctly**, from the very first month of data. A CT clamped
  around a whole cable rather than one conductor fits: the go and return currents
  cancel and only the imbalance shows, which is small and still tracks the load.
- **2024-10 onwards: exactly 0.0 W.** The raw covariance value stepped from ~360
  (against a GND reference of ~308) to 201 in 2024-10, and now sits *below* the
  reference, so `max(|col|-|gnd|,0)` floors at zero. That is no signal at all.

So: check that the CT is present and connected, **and** that it is around a single
conductor. Fixing only the second fault gets 8–20 W back, not the real load.

## `40 Lida` (port 12) — worked for three years, then stopped

15–91 W from 2018-12 through 2021-08, then **exactly 0.0 from 2021-10 onward**.
The user says this circuit has little or no use, so a zero reading is plausibly
correct now — but it did read a real load for three years, so either the breaker
went off around 2021-09 or the CT came off then. **Low priority; not a fault
unless he expects that circuit to be live.**

## ⚠ Absolute scale is NOT comparable across firmware epochs

`22 heat exchanger` is the steadiest load in the house (within 2026-02: p05 109,
p95 123 W) and reads **122 W (2019-01) drifting to 79 W (2025-05), then 119 W
(2025-12), 106 W (2026-08)**. The raw value tracks it (2644 → 1828 → 2865), and
the GND reference itself jumps ~300 → ~550 at the same 2025-12 boundary — the
`/read` formatting fix, with the sampling loop reworked again in 2026-05. A
constant load cannot do that, so the meter's absolute scale is epoch-dependent.
**Normalise against `22 heat exchanger` before comparing watts across years**, or
an anomaly detector will fire on firmware changes.

## Reading the numbers

Never quote a `max` from this data — at 0.33 % duty an extremum is an aliasing
artefact. Resample on **local** time: a UTC monthly resample relabelled to local
shifts every bucket by a month. Feeder arithmetic closes on the main unit (Feb
means: Main Red + Black 2251.7 W, Subpanel Red + Black 1922.0 W, difference
329.7 W against 281.7 W of metered main branches), so the 40 % gap between the
subpanel feed and the sub unit's 14 branches is a sub-side question — unmetered
subpanel circuits, or the two boards' separate calibrations (`conf.coeff` 385 vs
328.2). A known resistive load read on both units separates them, and is immune to
the PLUG fleet's undocumented 1.08 power trim.

See [[epm-main-unreliable-for-years]].
