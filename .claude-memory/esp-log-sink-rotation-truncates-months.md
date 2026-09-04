---
name: esp-log-sink-rotation-truncates-months
description: "The 10 MiB rotation that overwrote its single .1 destroyed most of Jun-Aug 2026; fixed in C_ESP, staged on bsd, awaiting an http_server restart"
metadata:
  node_type: memory
  type: project
---

bsd's `http_server.py` rotated a log to `<file>.1` past 10 MiB and **replaced**
any existing `.1`. At the meters' ~104 KB/h it fired every ~4 days, so only the
last two chunks of a month survived. Roughly the 1st ~10:00 to the 23rd–26th of
2026-06, -07 and -08 is gone on both units — unrecoverable (written to `/mnt/T`,
UFS on a USB stick, outside every snapshot).

**Fixed in C_ESP `development` (9fc4625), and already on bsd's clone** at
`~panasyuk/GIT_REPS/LIBS/C/ESP` — pushing to `HOME` updates that working tree
(`receive.denyCurrentBranch=updateInstead`). **The running process still has the
old code: it needs an `http_server` restart, which drops in-flight POSTs.**
A filename carrying the month (`--no-rotate-pattern`, `MONTHLY_NAME_RE`) is
exempt from size rotation, so the power logs stay one file per month; other names
keep the cap but the over-size file is renamed to `<file>.<YYYYmmdd-HHMMSS>` and
kept. Disk protection is `--max-rows-per-min` plus a `--dir-quota-bytes` alert,
neither of which deletes. `--alert-email` defaults to None — set it on the
service line or the alerts only print. `test_http_server.py` sits beside it.

**Repaired 2026-09-04:** 42,164 rows recovered from ZFS snapshots (the first
~10 h of each month, from the retired monthly rsync's copies), then each month
merged back to a single file — 115784/95224/98066 rows main, 96512/103779/92124
sub. Pre-merge sources kept at `/POOL/ARCHIVE/ESP_LOGS_PRE_MERGE/`, deliberately
**outside** `ESP_LOGS/` so a reader walking the tree cannot double-count them.
`scripts/merge_split_months.py` is the tool and the record of how.

**Why it stayed invisible:** the sink returned 200 and logged only
"Rotated <file>". Retention is no backstop either — POOL/ARCHIVE's weekly
snapshots have not fired since 2026-08-09, so a rotated chunk lived ~7 days in
the dailies and was then gone.

See [[power-log-data-flow]] and [[epm-main-unreliable-for-years]].
