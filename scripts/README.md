# scripts

Data-repair and maintenance drivers for the panel-meter logs on bsd. Run them there
(`python3`), or pipe them over ssh; they take absolute paths and touch nothing else.

| script | what it does |
|---|---|
| `merge_split_months.py` | Rebuilds one file per month for a `PowerMonitor.*.csv` that arrived split. Dry run by default, `--apply` to change anything. Concatenates in time order (the pieces do not overlap), refuses on a row-count mismatch or a non-increasing join boundary, and moves the sources to `/POOL/ARCHIVE/ESP_LOGS_PRE_MERGE/` rather than deleting them. |
| `channel_health.py` | Per-channel median / p95 / mean watts and duty for the main unit over one or more merged months (`python3 channel_health.py 02 04 08`), using `read_file.m`'s formula. For telling a dead CT from an off breaker: a breaker comes back across months, a fallen CT does not. |
| `gnd_daily.py` | Daily medians for every main-unit channel over the months given, plus each channel's hi/lo contrast and correlation against `Grounding strip`'s high days. Written to find which circuit switches with the intermittent ground current; the answer so far is none of the metered ones. |
| `port10_history.py` | Monthly medians for selected main-unit channels across every month in the archive, plus the raw covariance value beside the GND reference. Dates when a channel changed state, which is what separates a CT that came off from one that never read right. |

`merge_split_months.py` was written for the 2026-06/07/08 split, which came from
`http_server.py` rotating past 10 MiB into a single `.1` that it overwrote. That is
fixed at the sink — a filename carrying the month is now exempt from rotation — so this
should not be needed again. Kept because it records exactly how those three months were
reconstructed, and because it is the tool if a month ever splits again: edit `NAMES`.
