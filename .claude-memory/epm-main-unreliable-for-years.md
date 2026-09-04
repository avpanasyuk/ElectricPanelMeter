---
name: epm-main-unreliable-for-years
description: "EPM_main has logged only 9-63% of wall-clock minutes every year since 2020, including a 4.5-month run of payload-free rows in 2025"
metadata:
  node_type: memory
  type: project
---

Coverage (% of wall-clock minutes actually logged), measured 2026-09-04 by the
HOUSE_POWER session across all 173 sink files:

| year | main | sub |   | year | main | sub |
|---|---|---|---|---|---|---|
| 2020 | 62 | 86 | | 2024 | 40 | 80 |
| 2021 | 36 | 59 | | 2025 | **9** | 52 |
| 2022 | 42 | 47 | | 2026 | 59 | 53 |
| 2023 | 63 | 83 | | | | |

The only stretch in eight years where both units were near-complete is
2026-01 … 2026-04. 2026-05 onward reads low for both, but that is
[[esp-log-sink-rotation-truncates-months]], not the devices.

**2025-07-03 → 2025-11-15, EPM_main posted ~975,600 rows containing a timestamp
and one trailing comma — no payload**, at the normal 5 s cadence. It recovered
2025-12-10 15:22:23, mixed with short rows for ~24 minutes around that point.
**Diagnosed and long since fixed.** Commit `4787751` (2025-07-02, "a lot of time
is wasted on String reallocations") made `samples2string()` return a reference to
a static buffer that `/read` passed straight into `send()`; `5e500c6`
(2025-12-10 17:07, "was not formatting /read string properly") fixed it by
assembling into its own buffer. In 2025 the rows came from a host-side poller
reading `/read`, so a malformed response wrote a timestamp and an empty field.
Both boundaries bracket the commits with flash-then-commit ordering (data
recovered 2025-12-10 15:22, commit 17:07). Nothing to look for in current
firmware; treat 2025-07-03 → 2025-11-15 as a known-bad window and exclude it.

**Why it matters beyond the data:** the device was up, on WiFi, posting on
schedule; file mtime, growth rate and rows-per-day were all nominal for 4.5
months. Every liveness check reads green — only parsing the payload catches it.
Treat "posted but empty" as a distinct failure from "did not post".

**How to apply:** don't read a quiet stretch in the main-panel log as a house
event; check the unit first. Before trusting any long-run comparison of main
against sub, check coverage for that window — main's history is thin.
