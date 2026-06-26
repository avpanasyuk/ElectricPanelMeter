---
name: power-log-data-flow
description: "Where EPM power CSV logs live on bsd, and which dir show1.m must read"
metadata: 
  node_type: memory
  type: project
  originSessionId: 8b29c43a-0137-4fb7-b923-2893ff129f92
---

The ESP firmware pushes power-meter CSV rows **live** to bsd `/mnt/T` (the
`http_server.py` sink; SMB share `//bsd/USB_FLASH`). A **monthly** job then
moves each **finished** month's `PowerMonitor.v*.MM.YY.{main,sub}.csv` into
`/POOL/ARCHIVE/POWER` (SMB share `//bsd/ARCHIVE/POWER`).

`MATLAB/show1.m` (and `show.m` / `month_by_month.m`) plot **completed** months,
so they `cd('//bsd/ARCHIVE/POWER/')`. **Do NOT repoint show1 at `//bsd/USB_FLASH`.**
The in-progress **current** month exists only in `/mnt/T` (`//bsd/USB_FLASH`)
until it is moved at month end — read it there directly for an ad-hoc
current-month check.

**Why:** the archive's current-month file looks stale mid-month (e.g. frozen on
the 1st) only because the move runs at month end — it is NOT broken. I mistook
the live buffer for the source of truth and wrongly repointed show1 at
USB_FLASH; reverted.
**How to apply:** show1/show/month_by_month → `//bsd/ARCHIVE/POWER`; ad-hoc
current-month look → `/mnt/T` (`//bsd/USB_FLASH`). Column 1 of these logs is
dual-format (legacy Unix epoch + `yyyy-MM-dd HH:mm:ss.SS` since 2026-06-23),
handled by `read_file.m`.
