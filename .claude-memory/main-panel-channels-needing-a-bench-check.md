---
name: main-panel-channels-needing-a-bench-check
description: "Port 9 is the x570 branch circuit (earth path before 2024-10), not the grounding strip; the GEC question is closed as topology; channel labels here are hypotheses, and a level under ~25 W is offset while its modulation is still real"
metadata:
  node_type: memory
  type: project
---

Measured 2026-09-04 with `scripts/channel_health.py` and `scripts/gnd_daily.py`
(main unit, `read_file.m`'s formula).

## Port 9 — the x570 branch circuit, and the GEC question it used to be

`conf_main_v0.m` labels port 9 `x570 branch (earth path before 2024-10)`. The label is
era-explicit because the conf is applied retroactively to all v0 main data and the CT has
been on more than one thing.

### What it is now

The branch circuit feeding the **x570 workstation**, on the **black leg**. Proven twice:
HOUSE_POWER caught x570's smart plug stepping 78 → 811 W and got
`port9 = 1.037 × plug_raw + 4.2 W`, residual sd 40 W, edge simultaneous to the minute —
the ~4 W intercept says that circuit carries x570 and essentially nothing else. Confirmed
here with no plug in the chain: over 2026-08/09 the 350 minutes where port 9 steps >3σ
land in `Main Black` alone (`Main Red` 0.032, `Subpanel Black` 0.007, `Subpanel Red`
0.031, `26` 0.000; r = 0.91 on black). **A CT on an earth path cannot do that** — it
appears as a fraction of the neutral *unbalance*, never 1:1 into one leg.

### Which era is which — the shape test

Correlation of the 1-minute difference in port 9 with each feeder, over **all** minutes,
no selection. ⛔ Slopes are useless inside this span: port 9 sits in the ground-subtraction
residual for most of it, so its steps are near-noise and slope estimates blow up past 100.
Correlation is scale-free and survives that. ⚠ `sub_r` and `sub_b` mirror `red` and
`black` exactly, an artefact of signing each channel by its own long-run mean — compare
r_red against r_black as magnitudes.

⛔ **First check whether the channel is the reference.** `r(Δp9, Δgnd)` in raw signed
counts — no sign convention involved — is ≈0 for every month 2018-12…2024-09, then
**0.890, 0.931, and 0.984–0.996 for 2024-10 through 2025-06**. In that window port 9 *is*
the GND column: the CT reads nothing, `p9 − gnd` is ~0, and any statistic built on it
correlates with every channel through the shared `gnd` term rather than through load.
It carries signal again from 2025-12 (r back to −0.25…−0.65).

| era | raw p9−gnd median | r(Δp9,Δgnd) | r_red | r_black | reading |
|---|---|---|---|---|---|
| 2019-06/07 | −1124 | 0.00 | 0.720 | 0.242 | earth path |
| 2022-06/07 | −866 | 0.01 | 0.807 | 0.359 | earth path |
| 2024-03/04 | −4412 | 0.02 | 0.505 | 0.496 | earth path |
| 2024-08/09 | **+8068** | 0.06 | 0.296 | 0.107 | earth path |
| 2024-10/11 | −60 | **0.89–0.93** | 0.211 | 0.694 | ⛔ no signal — inadmissible |
| 2024-12…2025-06 | ~+150 | **0.98–0.996** | — | — | ⛔ no signal — inadmissible |
| 2026-02/03 | +252 | −0.27 | 0.075 | 0.239 | one leg |
| 2026-08/09 | +1370 | −0.04 | 0.012 | 0.277 | one leg |

⚠ **The `r_black` = 0.694 at 2024-10/11 is that artefact, not a branch signature** —
"black high, red low" is the default for any channel sitting at the reference, because
`Main Black`'s CT is reversed so its watts carry `+gnd` just as a below-reference port 9
does, while red-signed channels carry `−gnd`. HOUSE_POWER caught this; its cleaner
statistic is `asym = pr_black + pr_sblack` (gnd-partialled), which cancels common mode
because those two reversed CTs mirror each other exactly, and which puts 2024-10/11 at
+0.002 — indistinguishable from 44 months of earth path.

**Two CT events. The first is dated; the second is bounded, not dated:**

1. **June–July 2024**, his panel work (240 V garage circuit, another outlet, "maybe moved
   ground wire CT"): the raw sign flips −4412 → +8068 and the magnitude nearly doubles,
   but the shape stays earth-path. **Reversed or repositioned on the earth path**, not
   moved to a load. The coupling changed, which is why the diverted share is only
   consistent up to 2024-04.
2. **The move onto the branch: bounded to 2024-10 … 2025-12, best guess 2024-10.** What
   is certain is a real discontinuity at 2024-09 → 2024-10 — the raw count collapses from
   +7664 to −60 — which needs a physical cause, and no other discontinuity falls between
   there and the channel carrying signal again in 2025-12. That is parsimony, not
   measurement: from 2024-10 to 2025-06 port 9 is the reference and reads nothing, the
   board is dark 2025-07…2025-11, and **a branch circuit with nothing running is
   indistinguishable from a CT clipped to nothing at all**. HOUSE_POWER's `asym` first
   shows the one-leg signature in 2026-01, but that dates the first observed LOAD, not the
   CT move, so it excludes nothing earlier. ⚠ No data gap and no repo trace at 2024-09/10
   — the board was not powered down, so this move left none of the tells the first one
   did. What he did in the panel that autumn is the one open question on this channel.

⛔ **2026-08 is not a CT event** — do not read it as a re-seat or an onset. It is x570
starting to run: `asym` quadruples there while the channel had already been showing a
steady one-leg asymmetry since 2026-01. ⚠ The identity as x570 specifically is anchored
only at the 2026-08/09 end.

⭐ **Corroboration for the modulation rule from a second direction**: through 2026-01…04
the channel sits at 240–322 counts (12–17 W), squarely "pure residual" by level — yet its
`asym` holds a consistent −0.07. Something small and real was on that circuit months
before x570 came back. The ~25 W threshold governs the LEVEL only; the modulation carries
usable signature well below it, exactly as the hour-of-day swing test says.

### Gain

**Port 9 reads ~1.07 × Main Black, bracket [0.97, 1.18]** — ~1.08 absolute against the
feeders' ~1.01. HOUSE_POWER's plug-derived ×1.12 sits inside it; so does 1.00, so
**per-port gains differing by ~10 % on one board is supported, not established**. The
width is this method's, not theirs: their number is a difference of means across a
controlled 733 W step (±3 %), and its uncertainty lives in the plug's unchecked current
leg.

⛔ **Quote the bracket, never a point.** Selecting minutes on |Δport9| inflates its
variance with its own noise and attenuates the forward slope to 0.846 — which reads as
"port 9 is low" and is an artefact. Selecting on |Δblack| instead picks mostly other loads
on that leg (r = 0.107). Use `[slope(y|x), 1/slope(x|y)]`, both from the port-9-selected
set.

### The GEC finding — CLOSED, do not reopen

⛔ **Do not ask him to move the CT back.** He decided on 2026-09-04 that it stays and that
no alarm is built on the diverted share. The conductor port 9 used to watch is the panel's
**grounding electrode conductor to the water main**, clamp-measured at **3.2–3.3 A**:
parallel-path objectionable current (NEC 250.6), because the metal water service is a
bonded electrode and the utility neutral is multi-grounded, so neutral return divides
between the service neutral and the water main. A copper water main is not much worse a
conductor than an aluminium service neutral, and the divider ratio implies
`Z_gec ≈ 2.7 × Z_neutral`. It is topology, which is why nothing ever tripped on it in six
years — and the parallel path *lowers* effective neutral impedance (`Z_eff = 0.73 × Z_n`)
rather than raising it, so there is no degradation signal being lost.

⛔ **Never open the GEC/water bond while it carries current** — arcing, loss of the ground
reference, and if the utility neutral is compromised the pipe may carry the
neighbourhood's return, so breaking it can energise the plumbing. A "remove the bond
jumper" test applies to a *load-side* bond only, never here. The one safe decisive test if
it is ever reopened: open the main disconnect and re-clamp the GEC — gone means his own
load (look for a load-side N-G bond, 250.142(B)); still there means utility/neighbour
return, and a POCO matter.

**The diverted share** — port 9 regressed on the service-neutral unbalance
`|Main Red − Main Black|`, `scripts/port9_divider.py` — was flat at **0.285–0.298 across
2019–2023**, and the series ends at **2024-04**. ⚠ One residual, recorded and not pursued:
it rose to 0.326 [0.322, 0.330] in 2024-03 and 0.329 [0.326, 0.332] in 2024-04 —
non-overlapping CIs — *before* the panel work, so his answer does not account for it.

⛔ **Use the PAIRED-DIFFERENCE slope, never the level regression** (`port9_paired.py`).
Port 9 and the unbalance both follow the daily household rhythm, so they correlate through
that common mode whether or not they are physically coupled — the level estimator read
0.43 where the truth was 0.045. The two agree to a few percent when a CT is on the
conductor and diverge 6–10× when it is not, and that divergence is itself the diagnostic:

| month | level slope | diff slope |
|---|---|---|
| 2022-06 | 0.315 | 0.319 |
| 2024-04 | 0.350 | 0.336 |
| 2024-08 | 0.064 | 0.063 |
| 2026-01 | 0.001 | −0.001 |
| 2026-08 | 0.204 | 0.034 |
| 2026-09 | 0.431 | 0.045 |

### Instrument facts established while chasing this

- **Crosstalk and a load-dependent GND reference are both ruled out.** Against 106
  compressor steps >50 W the feeders respond at slope 0.31–0.35 as they must, while port 9
  gives 0.0037 (r = 0.023); and 1834 feeder steps >200 W move the GND reference at slope
  −0.005 (r = −0.078). Low-power channels carry no load-scaling floor.
- ⚠ **Citation hygiene.** NEC **250.6** (*Objectionable Current*) and **250.142(B)**
  (*Load-Side Equipment*) were confirmed independently. The IAEI "parallel paths" article
  does not resolve here, so its "large percentage of the neutral current" quote is
  **unverified**; NEC section *text* was only ever seen via a third-party scrape of
  NFPA-copyright material, which is no authority. Verified sources: Mike Holt's N-G
  voltage article, EC&M's "What's Wrong Here" (green bonding screw in a load-side
  subpanel).
- ⭐ **One minute with a clamp meter beat eight years of logs**, and one question to him
  then dated the CT move. The archive cannot separate a real current from a mislabelled
  CT. Reach for the instrument and for the owner before the regression.

**HOUSE_POWER owns the electrical question; this project owns the meter.**

## ⛔ "Exactly 0.0 W" means FLOORED, not disconnected — in any figure predating 2026-09-04

`read_file.m` used to compute `max(|col| - |gnd|, 0)`, so any channel whose raw value
sat below the GND reference reported a flat zero. In 2024-10 **four** main channels read
0.0 every day — A/C compressor, Grounding strip, 28 Air handler, 40 Lida — and the
A/C one is simply "October, no cooling". A zero therefore means *below the noise
reference*, and only a **step in the raw value against a stable reference** says
something physically changed. Read the raw columns before calling a channel dead.

The clamp is **gone** as of the signed-subtraction fix, so such a channel no longer reads
0.0 — it reads the magnitude of its residual against GND, which is an artefact of order
±20 W and not power. Same rule, different symptom: read the raw columns.
See [[read-file-ground-subtraction-sign]].

## ⛔ `conf_main_v0.m` channel LABELS are not reliable

Port 10 is labelled `28 Air handler attic`. The attic air handler runs
continuously and a blower is hundreds of watts, but the channel read **3–20 W from
2018-12 to 2024-09** and then **exactly 0.0 from 2024-10 on** (raw stepped ~360 →
201 against a ~308 GND reference; it now sits below the reference, which floored the
watts at zero under the old estimator). ⭐ **It is dead, confirmed by modulation**: its
hour-of-day swing is 1.3× the GND column's own, i.e. it tracks the reference and carries
no load at all. That also rules out reading its +6.7 W since 2025-12 as an A/C response.

His reading: **3–20 W is far too low for that air handler, so the channel is probably
mislabelled** — it was measuring some other small load, which stopped in 2024-10.

**Consequence: treat every label in `conf_main_v0.m` as a hypothesis, not a fact.** That
is no longer speculative — port 9 spent six years labelled `Grounding strip` while the CT
moved twice, and the label cost this project a long grounding-electrode investigation
before a clamp meter settled it.

Open questions for a panel visit: what port 10's CT is actually clamped on, why it
went dead in 2024-10, and whether the attic air handler is metered at all.

## ⛔ `38 Vova` (port 11) — NOT disturbed by the 2024 work. It swings on its own.

Two sessions asserted a 2024 collapse here and both were wrong, because both read one
high run as a baseline. The full corrected series (monthly means, W, whole archive)
settles it:

| span | port 11 |
|---|---|
| 2018-12 … 2019-08 | 8.5–13.7 |
| 2019-09 … 2020-05 | 18.9 → 34.4, rising |
| 2020-06 … 2021-07 | 36.4–**107.4** |
| 2021-08 | **7.7** — the sharpest step this channel has ever made |
| 2021-10 … 2023-11 | 12–47, variable |
| 2023-12 … 2024-05 | 45.8–64.9 |
| 2024-08 … 2026-01 | 2.0–12.6, except **34.7** in 2025-04 |
| 2026-02, -03, -06 | **38.9, 48.0, 28.2** |

⇒ **There is no discontinuity at the June–July 2024 panel work.** The channel's monthly
mean has ranged 2–107 W for its whole life and returns to 28–48 W repeatedly after 2024.
Its one genuinely sharp, unexplained step is **2021-07 → 2021-08, 107.4 → 7.7 W** — which
nobody has looked at, and which is far larger than anything in 2024.

⚠ **Its low months are not level measurements.** 2–12 W sits inside the ±20 W
per-channel residual, so "port 11 was quiet" and "port 11 read nothing distinguishable
from zero" are the same statement about its LEVEL. The same trap makes channel `24`
appear to step 12 → 26 W at the 2025-12 flash: 270 counts against the GND column's own
250-count step, i.e. the reference, not the circuit — the feeders moved only ×1.105 there.
**Test the modulation instead**: `24` swings 17× the GND column's own diurnal swing and
port 11 19×, so both carry real load regardless of where their levels sit. The rule and
the measured yardstick are in [[read-file-ground-subtraction-sign]].

⇒ The June–July 2024 work disturbed **port 9 only**. `24` and `26` are flat across it
(12 W, and 111–186 W either side), and so, correctly read, is port 11.

## `40 Lida` (port 12) — alive, and the one channel that matches its label

⛔ **The "exactly 0.0 from 2021-10 onward" that this note used to carry was the old
estimator's clamp, not the circuit.** Corrected, port 12 reads a small but unmistakably
real load: 2026-02 mean 11.8 W, median 10.5, p95 24.1. Its level sits inside the ±20 W
residual, so the level alone says nothing — but its **hour-of-day swing is 615.6 counts,
43× the GND column's own**, the largest ratio on the board, running 11.5 → 43.5 W with a
midday peak. That is a daily-use circuit behaving exactly as its door-table label implies,
and it is one of the few main-panel channels that does.

Its level did drop around 2021-09 (15–91 W before), which is real and unexplained, but the
channel was never dead. **Not a fault.**

## ⛔ There is NO constant reference load — do not use `22 heat exchanger`

`22 heat exchanger` looks like the steadiest load in the house (within 2026-02:
p05 109, p95 123 W) and its monthly median moves 122 W (2019-01) → 79 W (2025-05)
→ 119 W (2025-12) → 107 W (2026-08). **That is not instrument drift.** The user
ran it through an **auto-transformer and adjusted it from time to time**, removing
the auto-transformer during 2026. Its level is a record of those adjustments, so it
cannot calibrate anything and a year-over-year change in it is not a finding.

What does stand: the **GND reference channel steps ~300 → ~550** at the 2025-12
firmware rebuild (the `/read` fix; the sampling loop was reworked again in
2026-05). That channel is a grounded mux input, so it carries the additive offset
common to every channel, not a load — and since it is *subtracted* from every
channel, a step in it moves every reading. Not evidence of a gain change, but it is
**epoch-dependent bias**, and it scales the old code's wrong-sign subtraction on a
reversed-CT channel as `2·gnd`: ~85 W per reversed channel before the step, ~140 W
after. `Main Black` is reversed, so pre-fix mains totals are low by that much and the
error nearly doubles across the 2025-12 boundary.
See [[read-file-ground-subtraction-sign]].

**No validated constant load exists in this data**, so gain stability across
firmware epochs is currently unmeasured in either direction. A known resistive load
would settle it, and is the same bench test that settles the two-board question.

## Reading the numbers

Never quote a `max` from this data — at 0.33 % duty an extremum is an aliasing
artefact. Resample on **local** time: a UTC monthly resample relabelled to local
shifts every bucket by a month.

**Feeder arithmetic closes on the main unit, and closes better since the signed-GND fix.**
2026-02 means: Main Red + Black **2401.2 W**, Subpanel Red + Black **2071.5 W**,
difference **329.7 W** against **368.6 W** of metered main branches — of which ~18.6 W is
port 10's pure residual, so ~350 W is real. That is a ~6 % closure, against a 15 % gap
under the old estimator. ⚠ The *difference* is invariant under the fix (each leg pair
contains exactly one reversed CT, so both sides gain the same ~140 W); it is the branch
total that moved.

So the ~40 % gap between the subpanel feed and the sub unit's 14 branches is a sub-side
question — unmetered subpanel circuits, or the two boards' separate calibrations
(`conf.coeff` 385 vs 328.2). A known resistive load read on both units separates them, and
is immune to the PLUG fleet's undocumented 1.08 power trim.

See [[epm-main-unreliable-for-years]].
