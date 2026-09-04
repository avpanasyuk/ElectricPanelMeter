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

## Two dead channels

`40 Lida` (port 12) and `28 Air handler attic` (port 10) read **exactly 0.0 W
daily median on every day of 2026-08/09**, and 0 W monthly median in 2026-02, -04
and -08 — dead across both heating and cooling season, so "wrong season" is out
and, with crosstalk ruled out, so is a noise-floor explanation. Fallen or open CT,
or a breaker off since before February. Physical check.

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
