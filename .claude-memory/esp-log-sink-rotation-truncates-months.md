---
name: esp-log-sink-rotation-truncates-months
description: "bsd's http_server.py rotated at 10 MiB and overwrote the single .1, destroying Jun-Aug 2026; fix written, deploy pending"
metadata:
  node_type: memory
  type: project
---

bsd's `http_server.py` rotated a log to `<file>.1` past 10 MiB and **replaced**
any existing `.1`. Each meter posts ~104 KB/h, so the cap fired every ~4 days and
only the last two chunks of a month survived. 2026-06 … 2026-08 lost roughly the
1st ~10:00 to the 23rd–26th on both units — unrecoverable (they were on `/mnt/T`,
UFS on a USB stick, no snapshots).

**Recovered and restored:** `//bsd/ARCHIVE/ESP_LOGS/RECOVERED/` holds 42,164 rows
— the first ~10 h of 2026-06-01, 07-01 and 08-01 for both meters, from the old
monthly rsync's copies preserved in POOL/ARCHIVE snapshots. Kept in a
subdirectory so nothing in the sink dir is a file the sink did not write.

**Fix written, NOT deployed:** C_ESP branch `logsink-nondestructive-rotation`
(57154d0, pushed to GitHub+HOME). Rotation becomes `<file>.<YYYYmmdd-HHMMSS>`,
never overwriting; disk protection moves to a per-file rows/minute limit plus a
directory-quota alert, neither of which deletes. Deploying needs an
`http_server` restart, which drops in-flight POSTs — the user's call.

**Why it stayed invisible:** the sink returned 200 to the device and logged only
"Rotated <file>"; nothing reported data as lost. Retention is no backstop —
POOL/ARCHIVE's weekly snapshots have not fired since 2026-08-09, so a rotated
chunk lives ~7 days in the dailies and is then gone.

**How to apply:** read a month as every `<name>.csv*` plus `RECOVERED/<name>.csv`,
sorted by row timestamp — `MATLAB/month_chunks.m` does this; never glob the bare
`.csv` alone. Watch `EVR_Balance.log` and `OldEVR.log`, both ~80 % of the cap.
See [[power-log-data-flow]] and [[epm-main-unreliable-for-years]].
