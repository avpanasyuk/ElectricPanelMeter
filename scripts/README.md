# scripts

Data-repair and maintenance drivers for the panel-meter logs on bsd. Run them there
(`python3`), or pipe them over ssh; they take absolute paths and touch nothing else.

⛔ **Every script here computes watts the way `read_file.m` does, and that changed on
2026-09-04**: the GND column is the additive offset common to all channels, so it is
subtracted SIGNED and the magnitude is taken last — never `max(|col| - |gnd|, 0)` per
row, which is the wrong sign on a reversed-CT channel (`Subpanel Red`, `Main Black`,
`26`) and understates it by `2*gnd`. Keep any new script consistent with `read_file.m`.
Slopes and shares are invariant under the change; levels are not.

| script | what it does |
|---|---|
| `diurnal_modulation.py` | Which main-unit channels carry real load, by MODULATION rather than level. A residual offset is constant, so it cannot make a diurnal shape: a channel's hour-of-day swing against the GND column's own separates load from offset below ~25 W, where the level says nothing. >~10x the reference = load. Recompute the yardstick per month. |
| `gnd_sign_correction.py` | How much every past main-board figure moves under the signed-GND fix, month by month, for a consumer that has to correct published numbers rather than re-derive them. Output is committed as `data/gnd_sign_correction_by_month.csv`. Reads the whole archive; ~10 min on bsd. |
| `merge_split_months.py` | Rebuilds one file per month for a `PowerMonitor.*.csv` that arrived split. Dry run by default, `--apply` to change anything. Concatenates in time order (the pieces do not overlap), refuses on a row-count mismatch or a non-increasing join boundary, and moves the sources to `/POOL/ARCHIVE/ESP_LOGS_PRE_MERGE/` rather than deleting them. |
| `channel_health.py` | Per-channel median / p95 / mean watts and duty for the main unit over one or more merged months (`python3 channel_health.py 02 04 08`), using `read_file.m`'s formula. For telling a dead CT from an off breaker: a breaker comes back across months, a fallen CT does not. |
| `gnd_daily.py` | Daily medians for every main-unit channel over the months given, plus each channel's hi/lo contrast and correlation against `Grounding strip`'s high days. Written to find which circuit switches with the intermittent ground current; the answer so far is none of the metered ones. |
| `port10_history.py` | Monthly medians for selected main-unit channels across every month in the archive, plus the raw covariance value beside the GND reference. Dates when a channel changed state, which is what separates a CT that came off from one that never read right. |
| `port9_divider.py` | Regresses port 9 (the GEC CT) on the service-neutral unbalance `|Main Red - Main Black|` per file. The SLOPE is the share of neutral current diverted onto the water main, and it is calibration-proof (numerator and denominator share the board coefficient), so it is comparable across firmware epochs. A rising share is the service-neutral degradation signal; a collapsed r means the CT has moved off the conductor. |
| `port9_paired.py` | The same share by BOTH estimators -- level regression and paired differences of 1-minute means. Prefer the difference slope. Two channels that both follow the daily household rhythm correlate through that common mode whether or not they are physically coupled, so the level regression is only trustworthy when the two agree; where they diverge, the coupling is weak and the level figure is the artefact. |

`merge_split_months.py` was written for the 2026-06/07/08 split, which came from
`http_server.py` rotating past 10 MiB into a single `.1` that it overwrote. That is
fixed at the sink — a filename carrying the month is now exempt from rotation — so this
should not be needed again. Kept because it records exactly how those three months were
reconstructed, and because it is the tool if a month ever splits again: edit `NAMES`.
