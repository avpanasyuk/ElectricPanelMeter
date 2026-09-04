---
name: main-panel-channels-needing-a-bench-check
description: "The Grounding strip CT is on the GEC to the water main and it carries a measured 3.2-3.3 A - parallel-path objectionable current, owned by HOUSE_POWER; plus unreliable channel labels"
metadata:
  node_type: memory
  type: project
---

Measured 2026-09-04 with `scripts/channel_health.py` and `scripts/gnd_daily.py`
(main unit, `read_file.m`'s formula).

## `Grounding strip` (port 9) — MEASURED, and handed to HOUSE_POWER

**The conductor is the panel's grounding electrode conductor to the WATER MAIN**,
and a clamp meter on it read **3.2–3.3 A** on 2026-09-04. So it is not a
mislabelled CT and not an equipment ground fault — it is **parallel-path
objectionable current** (NEC 250.6): the metal water service is an electrode
bonded at the service, the neutral is bonded to it there, the utility neutral is
multi-grounded, so neutral return current divides between the service neutral and
the water main. A few amps there is a topology consequence, which is why no
breaker or GFCI ever tripped on it in six years.

**Decisive test, one breaker throw:** open the main disconnect and re-clamp the
GEC. Gone → his own load (look for a load-side N-G bond, 250.142(B)). Still there
→ utility/neighbour return through his water pipe, and it becomes a POCO matter.
A large and *growing* figure is the signature of a degrading service neutral.

⛔ **Never open the GEC/water bond while it carries current** — arcing, loss of the
ground reference, and if the utility neutral is compromised the pipe may carry the
neighbourhood's return, so breaking it can energise the plumbing. DSH's
"remove the bond jumper" test applies to a *load-side* bond only, never here.

⭐ **The SHARE, not the watts, is the number to watch** — and it is
calibration-proof, so it is the only quantity here comparable across firmware
epochs. Regressing port 9 W on the service-neutral unbalance
`|Main Red − Main Black|` W gives the fraction of neutral current diverted onto the
water main (`scripts/port9_divider.py`). **A rising share is the service-neutral
degradation signal, and it needs no clamp meter.**

⛔ **Use the PAIRED-DIFFERENCE slope, never the level regression.** Port 9 and the
unbalance both follow the daily household rhythm, so they correlate through that
common mode even when the CT is barely on the conductor. Differencing consecutive
1-minute means removes it (`scripts/port9_paired.py`). The two agree to a few
percent when the CT is seated and diverge 6–10× when it is not — **that divergence
is itself the CT-position diagnostic**:

| month | level slope | diff slope | verdict |
|---|---|---|---|
| 2022-06 | 0.315 | 0.319 | seated |
| 2024-04 | 0.350 | 0.336 | seated |
| 2024-08 | 0.064 | 0.063 | off |
| 2026-01 | 0.001 | −0.001 | off |
| 2026-08 | 0.204 | **0.034** | ~1/8 coupled |
| 2026-09 | 0.431 | **0.045** | ~1/8 coupled |

⭐ **The CT is on a DIFFERENT WIRE, not loosely on the GEC** — the user suspects he
moved it and forgot, and the levels confirm it. A loose CT scales the same waveform,
so port 9 would read `125.1 W × 0.034/0.319 = 13.3 W`; it actually reads 64.8 W in
2026-08 and 84.4 W in 2026-09, **4.8–4.9× more than the GEC can account for**. That
excess is a second, weakly-correlated source, i.e. another conductor.

⇒ **The GEC is not monitored at all right now**, and the fix is to move the CT back
— not to re-tighten it. The degradation signal is unmeasurable until then, and a
partially coupled channel reads low, so nothing supports a rising share; my earlier
reading of 0.36→0.43 as possible neutral degradation was the level estimator picking
up common mode, and is withdrawn.

⇒ It also explains the "three-state appliance" signature HOUSE_POWER found in
2026-08/09 (a flat ~87 W plateau held for 8+ hours, 500–1000 W bursts, ~8 W days):
**that is an appliance, because the CT is on an appliance circuit.** Which circuit
is unknown. Open question: whether the CT has been on it since 2024 and the load
started in August 2026, or whether it was moved again in August — the data cannot
separate those.

The CT came off **between 2024-04-30 and 2024-08-01** (0.336 → 0.063; the months
between hold too few rows to narrow it) and was partially re-seated between
2026-07 and 2026-08.

⚠ 2024-08 is unexplained at *both* estimators (0.064 / 0.063), so it is not an
estimator artefact: the slope collapsed while the level *rose* to a 434 W median,
and a pure coupling change would move both together.

⚠ Watts on this channel are not a calibrated current at any time, and at ~1/8
coupling they are roughly an eighth of the in-phase GEC power. In the fully-coupled
era the ~113 W median implies ~0.94 A in phase, against a clamp reading today of
3.2–3.3 A; whether that gap is out-of-phase or externally-sourced current is not
separable from this archive. The main-disconnect test separates it.

**The condition is chronic, not new.** Monthly medians, reproduced from the raw
archive: 61.7 W (2018-12) rising to 434.6 (2024-08) and 398.1 (2024-09); 0.0 in
2024-10 and -11; 7–11 W to 2025-06; 10–20 W 2025-12…2026-07; 64.6 (2026-08),
83.4 (2026-09). No firmware commit exists near 2024-10 or 2026-08 (the only 2024
commit is 2024-08-13 and does not touch `FirmwareV2`; nothing after 2026-06-25), so
those steps are **the CT being disturbed and later re-seated**, not the current
starting and stopping. The 2026-08 "onset" was an artefact of that.

**Owned by HOUSE_POWER from 2026-09-04**; this project keeps the meter itself.

## Historical note — how it looked before it was measured

Real power on a grounding conductor, **intermittent on a day scale**, not
seasonal and not load-correlated. Daily medians, 2026-08/09: 81, 4.6, 82, 8.6,
9.0, 114, 103, 101, 71, 13.5 W — alternating high/low, ratio 10.2x, p95 to 920 W.
Monthly medians sat at 11–14 W for 2026-01…07. That was read at the time as an
onset between 2026-08-01 and 2026-08-25; it was the CT being re-seated.

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

Causes researched 2026-09-04, citations opened and checked here rather than taken on
trust. With the conductor now known to be the GEC to the water main, **250.6 and the
water-pipe bond are the operative ones** and the appliance-fault entry is not: a load-side
neutral-to-ground bond (NEC **250.142(B)** *Load-Side Equipment* — verified
numbering); a shared/borrowed neutral or miswired MWBC (300.13(B), 210.4(B)); an
open or high-resistance neutral joint; an appliance case fault below breaker trip;
objectionable current over parallel paths (NEC **250.6** *Objectionable Current* —
verified); a second bond on metal water/gas pipe. **EMI Y-capacitors are ruled out
by arithmetic**: 2π·60·4.7 nF·120 V = 0.21 mA each and capacitive, so ~no real
power, against the 87 W ≈ **0.725 A** in phase that is observed.

⭐ The breaker-elimination sweep still names a contributing circuit if the main-off
test shows the current is load-side. The GFCI argument I had made here was wrong: a
GEC carries return current by design, so no GFCI sees it as an imbalance and nothing
would ever trip.

Verified sources: Mike Holt's N-G voltage article (the E=I·R worked example rising
to 1.25 V) and EC&M's "What's Wrong Here" (green bonding screw in a load-side
subpanel, citing 250.24(A)(1), 250.24(A)(5), 408.20 in 2003 numbering). ⚠ The IAEI
"parallel paths" article returned 403 here — its "large percentage of the neutral
current" quote is **unverified**. ⚠ NEC section *text* was cited only to a
third-party HuggingFace scrape of NFPA-copyright material — no authority; the
section *numbers* above were confirmed independently instead.

That clamp-meter step has been taken — see the top of this file. It is the reason
none of the above analysis settled anything: **one minute with a clamp meter beat
eight years of logs**, because the logs could not tell a real current from a
mislabelled CT and the meter could.

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
