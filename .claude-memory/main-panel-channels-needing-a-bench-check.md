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

**If the clamp IS on a ground conductor** (researched 2026-09-04; citations opened and
checked here, not taken on trust), the ranked causes are: a load-side
neutral-to-ground bond (NEC **250.142(B)** *Load-Side Equipment* — verified
numbering); a shared/borrowed neutral or miswired MWBC (300.13(B), 210.4(B)); an
open or high-resistance neutral joint; an appliance case fault below breaker trip;
objectionable current over parallel paths (NEC **250.6** *Objectionable Current* —
verified); a second bond on metal water/gas pipe. **EMI Y-capacitors are ruled out
by arithmetic**: 2π·60·4.7 nF·120 V = 0.21 mA each and capacitive, so ~no real
power, against the 87 W ≈ **0.725 A** in phase that is observed.

⭐ **One field test serves both branches:** CT on the conductor, then flip breakers
off one at a time. It names the responsible circuit whether the clamp is on a
ground conductor or on a mislabelled live one. Also: 0.725 A of genuine
ground-return current would trip any GFCI on that circuit instantly, and nothing
has tripped — which favours either a non-GFCI circuit or the mislabel.

Verified sources: Mike Holt's N-G voltage article (the E=I·R worked example rising
to 1.25 V) and EC&M's "What's Wrong Here" (green bonding screw in a load-side
subpanel, citing 250.24(A)(1), 250.24(A)(5), 408.20 in 2003 numbering). ⚠ The IAEI
"parallel paths" article returned 403 here — its "large percentage of the neutral
current" quote is **unverified**. ⚠ NEC section *text* was cited only to a
third-party HuggingFace scrape of NFPA-copyright material — no authority; the
section *numbers* above were confirmed independently instead.

**Next step is physical, and it is the one item here with a safety implication:**
a clamp meter on the actual conductor on a high day. That settles both things data
cannot — whether the clamp is on what `conf_main_v0.m` calls a grounding strip,
and whether the reading is current at all rather than a front-end artefact. Only
after that is it worth ranking causes (neutral-to-ground bond fault, a borrowed
neutral, a switching leakage path).

## ⛔ "Exactly 0.0 W" means FLOORED, not disconnected

`read_file.m` computes `max(|col| - |gnd|, 0)`, so any channel whose raw value sits
below the GND reference reports a flat zero. In 2024-10 **four** main channels read
0.0 every day — A/C compressor, Grounding strip, 28 Air handler, 40 Lida — and the
A/C one is simply "October, no cooling". A zero therefore means *below the noise
reference*, and only a **step in the raw value against a stable reference** says
something physically changed. Read the raw columns before calling a channel dead.

## ⛔ `conf_main_v0.m` channel LABELS are not reliable

Port 10 is labelled `28 Air handler attic`. The attic air handler runs
continuously and a blower is hundreds of watts, but the channel read **3–20 W from
2018-12 to 2024-09** and then **exactly 0.0 from 2024-10 on** (raw stepped ~360 →
201 against a ~308 GND reference; it now sits below the reference, which floors the
watts at zero). The user's reading: **3–20 W is far too low for that air handler,
so the channel is probably mislabelled** — it is measuring some other small load,
which then stopped in 2024-10.

**Consequence for everything else here: treat every label in `conf_main_v0.m` as a
hypothesis, not a fact.** It also raises the odds on the `Grounding strip`
explanation — a CT that is not on what its label says is a demonstrated failure
mode in this panel, not a speculative one.

Open questions for a panel visit: what port 10's CT is actually clamped on, why it
went dead in 2024-10, and whether the attic air handler is metered at all.

## `40 Lida` (port 12) — worked for three years, then stopped

15–91 W from 2018-12 through 2021-08, then **exactly 0.0 from 2021-10 onward**.
The user says this circuit has little or no use, so a zero reading is plausibly
correct now — but it did read a real load for three years, so either the breaker
went off around 2021-09 or the CT came off then. **Low priority; not a fault
unless he expects that circuit to be live.**

## ⛔ There is NO constant reference load — do not use `22 heat exchanger`

`22 heat exchanger` looks like the steadiest load in the house (within 2026-02:
p05 109, p95 123 W) and its monthly median moves 122 W (2019-01) → 79 W (2025-05)
→ 119 W (2025-12) → 107 W (2026-08). **That is not instrument drift.** The user
ran it through an **auto-transformer and adjusted it from time to time**, removing
the auto-transformer during 2026. Its level is a record of those adjustments, so it
cannot calibrate anything and a year-over-year change in it is not a finding.

What does stand: the **GND reference channel steps ~300 → ~550** at the 2025-12
firmware rebuild (the `/read` fix; the sampling loop was reworked again in
2026-05). That channel is a grounded mux input, so its magnitude is a noise
statistic set by samples-per-scan, not by any load — and since it is *subtracted*
from every channel, a higher floor pushes small channels down. Enough to matter for
low-power channels across that boundary; not evidence of a gain change.

**No validated constant load exists in this data**, so gain stability across
firmware epochs is currently unmeasured in either direction. A known resistive load
would settle it, and is the same bench test that settles the two-board question.

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
