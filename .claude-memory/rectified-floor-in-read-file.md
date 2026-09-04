---
name: rectified-floor-in-read-file
description: "read_file.m takes abs() per row then averages, rectifying single-cycle scatter into a ~2.1 kW floor on the feeder channels - every low-signal watt figure is suspect"
metadata:
  node_type: memory
  type: project
---

`read_file.m` computes `max(abs(col) - abs(gnd), 0)` **per row**, then averages. But
`mean(|x|) ≠ |mean(x)|`, and each CSV value is a **single-cycle** covariance
estimate — the firmware integrates one free-running ~16.67 ms window per port per
~2.6 s scan (`TimePeriod1<1000000UL/60, micros>`, which truncates to 16666 µs
against a true 16666.67 and is not locked to a zero crossing), so it scatters
widely around the true power. Taking the magnitude first rectifies that scatter
into a **positive constant** whenever the true term is small.

**Measured consequence** (HOUSE_POWER, 26 Eversource bills 2024-05…2026-07): the
billed average swings 6.7× across the year while `Main Red + Main Black` stays at
2080–2571 W, r = +0.06 against the bill. `mains ≈ max(true load, ~2100 W)` fits
every well-covered period: 2330→2268 and 2181→2253 in winter, but **349→2190** and
**274→2099** in spring. The winter agreement is not calibration success — it is the
months where the true load happens to exceed the floor.

Worst on the **feeder** channels: 200 A CTs at port coefficient 18.8, so 2.7× coarser
W/count than a branch and more stray field. Not board-wide.

**The sign survives on the wire** — there is no `abs()` anywhere in
`src/main_v2.cpp`, `samples2csv()` emits a signed float, and the CSVs carry
negatives. So this is fixable **host-side**: average the signed values and take the
magnitude only at the end, or carry a fixed per-channel sign (the negatives are CT
orientation and opposite-leg phase, not net export). Doing so also restores
direction of flow, which the current `abs()` destroys.

**How to apply:** treat every published watt figure for a low-signal channel as
suspect, including both boards' `conf.coeff` — `calibration.m` derived the sub
board's constant at low signal and may have measured the floor. The "777 W of
subpanel feed unaccounted for" gap is probably downstream of this too.
**Do not bench-calibrate a feeder channel below ~2.5 kW.**
See [[main-panel-channels-needing-a-bench-check]].
