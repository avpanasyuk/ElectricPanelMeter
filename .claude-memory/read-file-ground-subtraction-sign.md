---
name: read-file-ground-subtraction-sign
description: "read_file.m subtracted the GND reference from the MAGNITUDE, understating every reversed-CT channel (incl. Main Black) by ~140 W; fixed. There is NO 2 kW rectification floor - that was solar."
metadata:
  node_type: memory
  type: project
---

**Retracted: there is no ~2.1 kW rectified floor on the feeders.** The constant
`Main Red + Main Black` that looked like one is **gross consumption** — the PV array is
line-side tapped, so the main-panel CTs sit downstream of it and never see the
production, while the bill reads net. HOUSE_POWER owns that identity
(`gross = net + production`) and the resulting per-epoch gains.

**What is really wrong with the old `read_file.m`, in order of size.** Each CSV value is
`cov(V,I)` over one free-running 60 Hz window (`harvest_scan()` in `main_v2.cpp`), so it
is **signed**; the sign is CT orientation, not direction of flow. The last column is a
grounded mux input — no CT, so it carries only the additive offset every channel picks
up (~505 counts on the main board, Sept 2026).

1. **Wrong-sign ground subtraction — the real defect.** The old code did
   `max(|col| − |gnd|, 0)`. For a channel whose CT is reversed (`Subpanel Red`,
   **`Main Black`**, `26`) that subtracts the offset from the magnitude instead of from
   the signed value, understating it by **2 × gnd ≈ 1012 counts ≈ 140 W**. `Main Black`
   is a feeder, so the mains total was low by ~140 W — **+5.6 % on Sept 2026**
   (2533 → 2675 W).
2. **Rectification of near-zero channels — real but small.** `mean(|x|) ≠ |mean(x)|`
   still holds, but consecutive-sample scatter is only ~85 counts on Main Red and
   ~225 on Main Black = **10–30 W**, nowhere near the ~1000 counts a 2 kW floor needs.
   It only bites a channel sitting at the offset, and there the `max(…,0)` clamp
   dominates anyway.

**Measured across the whole archive** (`scripts/gnd_sign_correction.py` →
`data/gnd_sign_correction_by_month.csv`, 79 months, 2018-12…2026-09): the mains
correction is **+3.2 to +5.0 % in every month through 2025-06**, then **+4.7 to +7.2 %
from 2025-12 on** — the GND channel's ~300 → ~550 count step at that firmware rebuild,
showing up exactly where it must. In watts, ~82 W before and ~150 W after. It is a
per-epoch multiplicative-looking bias that is really a fixed additive one, so it hits a
low-consumption month hardest.

**The GND column is a common additive offset — verified, not assumed.** With its breaker
off, `25 - Main A/C compressor` sits at a median of **517 counts** against the GND
column's own **511**. A switched-off load reading exactly the reference is what makes the
signed subtraction correct.

**Residual floor after the fix: ±20 W per channel, not zero.** `28 Air handler attic` and
`40 Lida` average *below* GND (363 and 191 counts vs 505), so the offset is not identical
across mux inputs, and a level of that size is not by itself a measurement.

⛔ **But test the MODULATION, not the level — a residual is constant, so it cannot make a
diurnal shape.** "Under 25 W means nothing" is too blunt and throws away real load. The
GND column's own hour-of-day swing is the yardstick: **14.2 counts** over 2026-03
(357950 rows). Against it, same month:

| channel | swing | × GND | W range |
|---|---|---|---|
| `40 Lida` | 615.6 | **43×** | 11.5 → 43.5 |
| `38 Vova` | 273.9 | **19×** | 43.4 → 57.6 |
| `24` | 238.7 | **17×** | 13.0 → 25.4 |
| port 9 (x570 branch) | 80.6 | 5.7× | 9.9 → 14.1 |
| `28 Air handler attic` | 18.8 | **1.3×** | −18.9 → −17.0 |

⇒ `40 Lida` and `24` carry real diurnal load of tens of watts even though their LEVELS sit
in the residual band. ⇒ `28 Air handler attic` tracks the reference and does not modulate
at all — which confirms it reads nothing far more strongly than the level argument did,
and rules out the "responds slightly to A/C" reading of its +6.7 W since 2025-12.

**The rule: below ~25 W a branch channel's LEVEL is offset; its MODULATION is real when it
is ≳10× the GND column's own swing in the same month.** Compute the yardstick per month —
it scales with the offset, which stepped at the 2025-12 flash.

**How to apply:** `read_file.m` now subtracts GND signed, applies one fixed per-channel
sign taken from the whole-record mean, and no longer clamps at zero — so reverse flow
survives (a future load-side PV, battery or bidirectional EV charger would be visible;
the present line-side array still is not). Any watt figure published before 2026-09-04
for a reversed-CT channel is ~140 W low. See
[[main-panel-channels-needing-a-bench-check]].
