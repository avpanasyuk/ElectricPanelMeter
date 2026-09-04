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
across mux inputs. Their post-fix 14.7 W / 16.4 W is the magnitude of that residual, not
power. Treat anything under ~25 W on a branch channel as indistinguishable from zero —
and note this independently confirms `28 Air handler attic` reads nothing.

**How to apply:** `read_file.m` now subtracts GND signed, applies one fixed per-channel
sign taken from the whole-record mean, and no longer clamps at zero — so reverse flow
survives (a future load-side PV, battery or bidirectional EV charger would be visible;
the present line-side array still is not). Any watt figure published before 2026-09-04
for a reversed-CT channel is ~140 W low. See
[[main-panel-channels-needing-a-bench-check]].
