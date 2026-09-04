---
name: power-log-data-flow
description: "Where EPM power CSV logs live on bsd, and which dir the MATLAB scripts read"
metadata: 
  node_type: memory
  type: project
  originSessionId: 8b29c43a-0137-4fb7-b923-2893ff129f92
---

The ESP firmware pushes power-meter CSV rows to bsd's `http_server.py` sink,
which writes them into **`/POOL/ARCHIVE/ESP_LOGS`**
(SMB share `//bsd/ARCHIVE/ESP_LOGS`). One directory, live and historical
together: there is no second location and no monthly move job. `MATLAB/show1.m`,
`show.m` and `month_by_month.m` all `cd` there, and `conf_sub_v2.m` sets
`conf.dir` to the same path.

**Why:** the old two-tier flow (live buffer on `/mnt/T` = `//bsd/USB_FLASH`,
month-end move to `/POOL/ARCHIVE/POWER`) was retired in the 2026-09-02 sink move.
`/POOL/ARCHIVE/POWER` no longer exists, and `/mnt/T` is still mounted but frozen
at 2026-09-02 21:11 — it answers reads with pre-move data and **no error**, so
anyone following the old note gets stale rows with no clue anything is wrong.

**How to apply:** every reader → `//bsd/ARCHIVE/ESP_LOGS`. Never `/mnt/T` or
`//bsd/USB_FLASH`. Column 1 of these logs is dual-format (legacy Unix epoch +
`yyyy-MM-dd HH:mm:ss.SS` since 2026-06-23), handled by `read_file.m`.
See [[esp-log-sink-rotation-truncates-months]] before trusting a month's row count.
