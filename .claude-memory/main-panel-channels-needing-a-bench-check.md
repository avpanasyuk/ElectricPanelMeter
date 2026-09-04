---
name: main-panel-channels-needing-a-bench-check
description: "CLOSED by his decision: the GEC carries 3.2-3.3 A of parallel-path current and that is topology, not a defect - the CT stays off it, port 9 is now an ordinary unlabelled load channel; plus unreliable channel labels"
metadata:
  node_type: memory
  type: project
---

Measured 2026-09-04 with `scripts/channel_health.py` and `scripts/gnd_daily.py`
(main unit, `read_file.m`'s formula).

## `Grounding strip` (port 9) — MEASURED, then CLOSED by his decision

⛔ **Do not reopen this as a fault, and do not ask him to move the CT back.** He
decided on 2026-09-04 that the CT stays where it is and no alarm gets built on the
diverted share. A copper water main is not much worse a conductor than an aluminium
service neutral, so a few amps taking it is topology. The measurement agrees: the
divider ratio implies `Z_gec ≈ 2.7 × Z_neutral`, and the parallel path *lowers* the
effective neutral impedance (`Z_eff = 0.73 × Z_n`) rather than raising it.

⇒ **The diverted-share series ends at 2024-04, permanently.** It was flat at
0.285–0.298 across 2019–2023, and his legs are symmetric. **Port 9 is an ordinary
unlabelled load channel** from 2024-04 on — HOUSE_POWER models it on post-2024-04
data and ignores its name. The rest of this section is the evidence, kept because it
is what closed the question, not because anything is still open.

One residual, recorded and not pursued: the share was flat through 2023, then rose to
0.326 [0.322, 0.330] in 2024-03 and 0.329 [0.326, 0.332] in 2024-04 — non-overlapping
CIs — immediately before the CT moved. Small, unexplained, now unobservable. It falls
in the same **2024-05…2024-08 window** as the undocumented panel work in
[[main-board-scale-epochs]], which is the one place left to look.


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

⚠ **Port 9's pre-2024 LEVELS are not comparable with these** — they were mostly
rectification artefact. Under the old `max(|col| - |gnd|, 0)` the channel reported
90–160 W in every month from 2018 to 2023; signed, most of those months collapse to
4–50 W (2019-02: 103.8 → 4.0 W; 2018-12: 89.7 → 10.9 W) while a few barely move
(2019-09: 147.4 → 149.7 W). Where old ≈ new the channel carried real current; where
new ≪ old it was sitting on the offset. So port 9 was **intermittently** live long
before 2024, not steadily so, and the 2024-08 step is larger than the old numbers
suggested. The 2024-02…09 and 2026-08/09 levels quoted above move by under 8 %, so
nothing here rests on the artefact, and every slope is unaffected throughout.
Per-month old-vs-new: `data/gnd_sign_correction_by_month.csv`.
See [[read-file-ground-subtraction-sign]].

⭐⭐ **IDENTIFIED, 2026-09-04: main port 9 is the branch circuit feeding the x570
workstation.** HOUSE_POWER caught x570's smart plug stepping 78 → 811 W and differenced
every channel on both panels across it: `port9 = 1.037 × plug_raw + 4.2 W`, residual
sd 40 W, edge simultaneous to the minute. The ~4 W intercept says that circuit carries
x570 and essentially nothing else.

Confirmed here independently, with no plug in the chain — paired 1-minute differences
over 2026-08/09, selecting the 350 minutes where port 9 steps >3σ: **the step lands in
`Main Black` and nowhere else** (`Main Red` 0.032, `Subpanel Black` 0.007, `Subpanel Red`
0.031, `26` 0.000). A GEC CT would show as a fraction of the neutral *unbalance*, never
1:1 into one leg.

### Dating the move: it is 2024-10 — not 2024-06, and not 2026-08

Scale-free shape test: correlation of the 1-minute difference in port 9 with each feeder,
over **all** minutes, no selection. Selection and slopes both mislead here — port 9 sits
in the residual for most of this span, so its steps are near-noise and slopes blow up to
100+. Correlation is scale-free and survives that.

⚠ In this table `sub_r` and `sub_b` mirror `red` and `black` exactly — an artefact of
signing each channel by its own long-run mean. **The informative comparison is r_red
against r_black**, as magnitudes.

| era | raw p9−gnd median | r_red | r_black | shape |
|---|---|---|---|---|
| 2019-06/07 | −1124 | 0.720 | 0.242 | both legs |
| 2022-06/07 | −866 | 0.807 | 0.359 | both legs |
| 2024-03/04 | −4412 | 0.505 | 0.496 | both legs |
| 2024-08/09 | **+8068** | 0.296 | 0.107 | both legs |
| 2024-10/11 | −60 | 0.211 | **0.694** | **one leg** |
| 2026-02/03 | +252 | 0.075 | 0.239 | one leg |
| 2026-08/09 | +1370 | 0.012 | 0.277 | one leg |

**Two separate events, both now dated:**
1. **June–July 2024** (his panel work): the raw sign flips −4412 → +8068 and the magnitude
   nearly doubles, but the shape stays both-legs. The CT was **reversed or repositioned on
   the earth path**, not moved to a load.
2. **2024-09 → 2024-10**: the magnitude collapses to ~0 *and* `red` drops away while
   `black` becomes dominant. **That is the move onto the black-leg branch** — and it is
   the "second break" this note previously carried as undated and unexplained.

⇒ Port 9 has been a branch channel since **2024-10**, not since 2026-08. Its level stayed
inside the residual until 2026-08 only because x570 was idle; the *shape* was the branch
shape throughout. ⚠ The identity as x570 specifically is anchored only at the 2026-08/09
end, by the plug step. The black-only shape is continuous from 2024-10, so one move is the
simplest reading — a second move inside that window would not show.

⚠ **There is no data gap and no repo trace at 2024-09/10.** Unlike the June–July work the
board was not powered down, so the usual tell is absent and the CT was moved a second time
unrecorded. That is the question if the panel comes up with him.

`conf_main_v0.m` port 9 is relabelled `x570 branch (earth path before 2024-10)` —
era-explicit, because the conf is applied retroactively to all v0 main data.

⚠ **Gain: port 9 reads ~1.07× Main Black, bracket [0.97, 1.18]** — so ~1.08 absolute
against the feeders' ~1.01. HOUSE_POWER's ×1.12 from the plug sits inside that, but so
does 1.00, **so "per-port gains differ by ~10 % on the same board" is supported and not
yet established.** ⛔ The bracket is what matters, not a point estimate: selecting minutes
on |Δport9| inflates its variance and attenuates the forward slope (0.846 raw, an
artefact I nearly quoted), while selecting on |Δblack| picks mostly other loads on that
leg and gives r = 0.107. Use the errors-in-variables bracket
`[slope(y|x), 1/slope(x|y)]`, both from the port-9-selected set.

⭐ **The CT move is confirmed by him, and it happened in June–July 2024** — he added a
240 V garage circuit and another outlet, and "maybe moved ground wire CT" (2026-09-04).
So the CT-position inference above was right, and the date is fixed. See
[[main-board-scale-epochs]].

⚠ **Port 9 has TWO breaks, not one** — do not model it as a single channel after 2024-04.
Corrected monthly means: 116.7 W (2024-05) → **330.8, 351.2** (2024-08/09) → **2.8, 1.4,
5.9** (2024-10/11/12) → ~2–7 W through 2025 → 19.7 (2025-12) → 137–238 (2026-08/09). The
June–July work moved the CT onto something that carried MORE than the GEC, which is why
the level rose while the coupling slope collapsed to 0.063. Then a **second** event in
September–October 2024 took it to zero — that one has no explanation and no gap to date it.

⇒ **The GEC is monitored by nothing, and that is now the accepted state.** A
partially coupled channel reads low, so nothing supported a rising share; my earlier
reading of 0.36→0.43 as possible neutral degradation was the level estimator picking
up common mode, and is withdrawn.

⇒ It also explains the "three-state appliance" signature HOUSE_POWER found in
2026-08/09 (a flat ~87 W plateau held for 8+ hours, 500–1000 W bursts, ~8 W days):
**that is an appliance, because the CT is on an appliance circuit.** Which circuit
is unknown, and — the question being closed — is not worth a panel visit to find out.
The data cannot say whether the CT has been on it since 2024 with the load starting in
August 2026, or was moved again in August.

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
no load at all. That also rules out reading its +6.7 W since 2025-12 as an A/C response. The user's reading: **3–20 W is far too low for that air handler,
so the channel is probably mislabelled** — it is measuring some other small load,
which then stopped in 2024-10.

**Consequence for everything else here: treat every label in `conf_main_v0.m` as a
hypothesis, not a fact.** It also raises the odds on the `Grounding strip`
explanation — a CT that is not on what its label says is a demonstrated failure
mode in this panel, not a speculative one.

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
shifts every bucket by a month. Feeder arithmetic closes on the main unit (Feb
means: Main Red + Black 2251.7 W, Subpanel Red + Black 1922.0 W, difference
329.7 W against 281.7 W of metered main branches), so the 40 % gap between the
subpanel feed and the sub unit's 14 branches is a sub-side question — unmetered
subpanel circuits, or the two boards' separate calibrations (`conf.coeff` 385 vs
328.2). A known resistive load read on both units separates them, and is immune to
the PLUG fleet's undocumented 1.08 power trim.

See [[epm-main-unreliable-for-years]].
