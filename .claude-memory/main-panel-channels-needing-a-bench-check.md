---
name: main-panel-channels-needing-a-bench-check
description: "Main-unit channels 40 Lida and 28 Air handler read dead in every month checked, and Grounding strip tracks the A/C - all three need someone at the panel"
metadata:
  node_type: memory
  type: project
---

Measured 2026-09-04 over 2026-02, -04 and -08 with `scripts/channel_health.py`
(main unit, `read_file.m`'s formula). Three things data cannot settle:

- **`40 Lida` (port 12) reads dead in all three months** — median 0 W, and the
  rise in "% of rows above 5 W" (1.1 → 6.5 → 10.2) comes with p95 going 0 → 9 →
  17 W, i.e. the rectified noise floor growing with ambient, not a load. Fallen
  or open CT, or a breaker off since before February.
- **`28 Air handler attic` (port 10) reads 0 W median in February AND August** —
  dead across both the heating and the cooling season, so "wrong season" is out.
- **`Grounding strip` (port 9) tracks the A/C compressor seasonally** — median
  13.8 W in Feb and Apr, 64.5 W in August with p95 811 W, rising in lockstep with
  the compressor (p95 51 → 1340 W). Either real ground/neutral current, which is
  an electrical fault, or mux crosstalk from a high-current channel.

**Why it matters beyond those channels:** if it is crosstalk, every low-power
channel on the main unit carries an additive floor that scales with total house
current — which would also explain the two "dead" channels' non-zero tails. So
settle the grounding-strip question first; it changes how the other two read.

**How to apply:** don't conclude a circuit is unused from these channels, and
don't quote a `max` from this data at all — at 0.33 % duty an extremum is an
aliasing artefact. Feeder arithmetic closes on the main unit (Feb: Main Red +
Black 2251.7 W, Subpanel Red + Black 1922.0 W, difference 329.7 W against 281.7 W
of metered main branches), so the 40 % gap between the subpanel feed and the sub
unit's 14 branches is a sub-side question — unmetered subpanel circuits, or the
two boards' separate calibrations (`conf.coeff` 385 vs 328.2). A known resistive
load read on both units separates them. See [[epm-main-unreliable-for-years]].
