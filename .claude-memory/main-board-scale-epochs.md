---
name: main-board-scale-epochs
description: "Complete record of every date the reported watt scale could have moved - conf edits, firmware, hardware - plus the GND column as the epoch instrument. A conf edit can never make a boundary; the FLASH date does, not the commit."
metadata:
  node_type: memory
  type: project
---

Assembled 2026-09-04 from the repo's own history, for HOUSE_POWER's bill calibration,
which was inferring epochs from the signal. Use this instead of a detector.

⛔ **A `conf.coeff` or port-coefficient edit CANNOT create an epoch boundary.** The
conf is selected by the version digit in the log filename and applied retroactively to
every row read with it, so an edit uniformly rescales that version's whole history — it
never steps the data at the edit date. The only conf-driven boundary is a change of the
*filename version digit* (sub v0→v1→v2), which selects a different file.

⛔ **The commit date is not the boundary — the FLASH date is, per board.** Proof: the
sampling rewrite is 2025-07-03, but main's data steps at 2025-12, because main logged
nothing between 2025-06 and 2025-12 and was reflashed around 2025-12-10. The two boards
do not share boundaries. Bound a flash by the coverage gap.

## Conf edits — complete

- `conf_main_v0.m`: `conf.coeff = 385` never changed since the file was created
  (351866c, 2019-05-11). Port coefficients changed **once**: 2020-01-02 (34e18ec), ports
  6 and 10 `50` → `50/2` "two phases" — a retroactive ×0.5 on `25 - Main A/C compressor`
  and `28 Air handler attic`. 2024-08-13 (5538c31) is **labels only**. The four feeder
  coefficients (18.8) have never moved.
- `conf_sub_v2.m`: `conf.coeff` 376.62 → 328.18 at 2025-07-02 (065f33d) — retroactive
  ×1.148 on all sub-v2 watts. 2026-09-02/03 changed `conf.dir` only.
- `conf_sub_v0.m` / `conf_sub_v1.m`: 376.62, untouched since 2019-05-11.

## Firmware — only three things can move the scale

`LastPower = (Power − Current·Voltage/N)/N` normalizes by sample count, so the sampling
RATE does not change the scale. Only an explicit factor, the mux settling `Delay`, and
the V→I sample phase can.

- `SAMPLES_IN_WF = 600` unchanged since 2018-12-05; `Delay = GetDelay(4)` (160 µs)
  unchanged since 2018-12-09 (`GetDelay(5)` for the four days before).
- **2019-05-10 → 05-11** (a87409d → 1d09284): phase interpolation added, reverted next day.
- **2025-07-02 → 07-03** (065f33d → eda9b90): `NumSamplesPerWL = 8` put an explicit extra
  `/8` on every reported power. Anything logged in that one-day window is **8× low**.
- ⭐ **2025-07-03 (eda9b90)**: rewrote the cadence to `WIFI_timeRatio = 10` — read one
  port flat out, then yield 10 wavelengths to WiFi. Changes the V→I sample spacing, hence
  the phase error, hence the gain. **This is the 2025/2026 boundary**, reaching main only
  at its 2025-12-10 flash.
- **2026-06-08 (c00840b, v5.04)**: fixed a race where `/read` and the bsd push stole each
  other's accumulator, so a value could be a partial scan. **Exclude 2026-05-25…06-08.**
- Everything else 2025-11 → 2026-06 is WiFi/OTA/mDNS/logging/LED, not the ADC path.
- **Nothing between 2024-08-13 and 2025-07-02** — one commit in that whole window, and it
  is the label-only conf edit.

## ⭐ The epoch instrument: the main board's GND column

The last CSV column is a grounded mux input, so it is the additive offset with **no load
in it**. Annual mean ADC counts from `data/gnd_sign_correction_by_month.csv`
(`gnd_counts`): 2019 297.8, 2020 298.4, 2021 295.0, 2022 305.0, 2023 309.0, 2024 306.0,
2025-12 562.8, 2026 528.0. Flat inside ±5 % for seven years, then one clean step, no
drift. **Two epochs on main and nothing else.** It is a board diagnostic, so it is the
check, not the key — but it beats any load channel. See
[[read-file-ground-subtraction-sign]] for why that column matters twice over.

## The 2024-06/07 gap — ANSWERED by him, 2026-09-04

He did the panel work himself: **added a 240 V circuit to the garage, added another
power outlet, and "maybe moved ground wire CT"** — his words, "it should not have
changed anything else." Main is dark through 2024-06/07 because he powered the board
down to work.

That accounts for the GND step (294.8 → 329.3 → 308.2 by 2024-10), port 9 leaving the
GEC, and the collapse of port 11 — the archive could never have supplied any of it.
**The feeders did not move**: HOUSE_POWER's bill-plus-PV chain reads ~0.914 either side,
so the board's scale is unchanged. CT and wiring work, not calibration. Treat
**2024-06-01 … 2024-08-01 as a hard discontinuity for ports 9 and 11 only.**

⛔ **The lesson, worth more than the answer: when a step lands in a window where the
meter also went dark, ASK HIM FIRST.** Those two coincide precisely because he powers the
board down to work on the panel, so the gap is the tell. Two sessions spent a long
exchange inferring a physical event from ADC counts, firmware history and commit logs,
and one question settled it. HOUSE_POWER records the same lesson in its own
`panel-work-june-july-2024.md`.
