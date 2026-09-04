---
name: esp-log-sink-rotation-truncates-months
description: "bsd's http_server.py rotates at 10 MiB and overwrites the single .1, so each EPM month keeps only its last ~4-9 days"
metadata:
  node_type: memory
  type: project
---

bsd's `http_server.py` rotates a log to `<file>.1` once it passes
`max_log_bytes` (10 MiB) and **replaces** any existing `.1`. Each EPM unit posts
one ~145-byte row every 5 s ≈ 104 KB/h, so a monthly file crosses 10 MiB every
~4.2 days and rotates ~7 times a month. Only the last two chunks survive:
`PowerMonitor.v0.08.26.main.csv.1` starts 2026-08-25, the `.csv` covers 08-30
onward, and August 1–25 is gone. Same on the sub unit; first `.1` files are
2026-06. Not in ZFS snapshots (written to `/mnt/T`, which is outside the
`POOL/ARCHIVE` snapshot set), so 2026-06 … 2026-08 is unrecoverable.

**Why:** the surviving chunks show unbroken 5 s posting at exactly the rate that
predicts the rotation interval, and the same calendar months in 2019–2025 are
single 40–65 MB files — so this is the sink discarding data, not the meters going
quiet. It is silent: no error anywhere, and a month simply reads short.

**How to apply:** for any month from 2026-06 on, read `<file>.csv.1` as well as
`<file>.csv`, and never treat a month's row count as a measure of uptime. The fix
belongs in C_ESP's `http_server.py` (numbered suffixes or a per-file rate limit,
keeping the disk-fill protection) — reported to the home-servers session
2026-09-04. See [[power-log-data-flow]].
