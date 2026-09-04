#!/usr/local/bin/python3
"""Merge the split 2026-06/07/08 PowerMonitor logs back into one file per month.

Dry run by default; pass --apply to make changes.

The three pieces of each month are internally chronological and do not overlap, so this
concatenates in time order rather than sorting -- which keeps it from having to parse
the mixed epoch/human timestamp formats for anything but the join boundaries. It refuses
on any row-count mismatch or non-increasing boundary, and moves the sources aside rather
than deleting them.
"""
import shutil
import sys
import time
from datetime import datetime
from pathlib import Path

LOG = Path('/POOL/ARCHIVE/ESP_LOGS')
REC = LOG / 'RECOVERED'
PRE = LOG / 'PRE_MERGE'
APPLY = '--apply' in sys.argv

NAMES = [f"PowerMonitor.v{v}.{mm}.26.{panel}.csv"
         for v, panel in (('0', 'main'), ('2', 'sub'))
         for mm in ('06', '07', '08')]


def epoch(ts):
    """Row timestamps come in two formats, sometimes inside one file: legacy Unix epoch
    seconds, and local wall clock written by http_server.py. bsd runs America/New_York,
    so strptime().timestamp() maps the second form to the same scale as the first."""
    ts = ts.strip()
    if '-' in ts:
        return datetime.strptime(ts, '%Y-%m-%d %H:%M:%S.%f').timestamp()
    return float(ts)


def rows_of(p):
    return [l for l in p.read_text().splitlines() if l.strip()]


failed = []
print(f"{'APPLY' if APPLY else 'DRY RUN'} -- merging {len(NAMES)} months in {LOG}\n")

for name in NAMES:
    # chronological order: the recovered head, the rotated middle, then the live tail
    pieces = [REC / name, LOG / f"{name}.1", LOG / name]
    missing = [p for p in pieces if not p.is_file()]
    if missing:
        print(f"SKIP  {name}: missing {[p.name for p in missing]}")
        continue

    age_h = (time.time() - (LOG / name).stat().st_mtime) / 3600
    if age_h < 24:
        print(f"FAIL  {name}: modified {age_h:.1f} h ago -- not a closed month, refusing")
        failed.append(name)
        continue

    chunks = [rows_of(p) for p in pieces]
    counts = [len(c) for c in chunks]

    ok = True
    for i in range(len(chunks) - 1):
        last = epoch(chunks[i][-1].split(',')[0])
        first = epoch(chunks[i + 1][0].split(',')[0])
        if not last < first:
            print(f"FAIL  {name}: piece {i} ends at or after piece {i+1} starts "
                  f"({chunks[i][-1].split(',')[0]} vs {chunks[i+1][0].split(',')[0]})")
            ok = False
    if not ok:
        failed.append(name)
        continue

    merged = [r for c in chunks for r in c]
    total = sum(counts)
    span = (chunks[0][0].split(',')[0], chunks[-1][-1].split(',')[0])
    print(f"OK    {name}: {counts[0]} + {counts[1]} + {counts[2]} = {total} rows,"
          f" {span[0]} .. {span[1]}")

    if not APPLY:
        continue

    staged = LOG / f"{name}.merged"
    staged.write_text('\n'.join(merged) + '\n')
    back = rows_of(staged)
    if len(back) != total or back != merged:
        print(f"FAIL  {name}: staged file does not match ({len(back)} of {total}) -- left in place")
        failed.append(name)
        continue

    PRE.mkdir(exist_ok=True)
    shutil.move(str(pieces[0]), str(PRE / f"{name}.recovered"))
    shutil.move(str(pieces[1]), str(PRE / f"{name}.1"))
    shutil.move(str(pieces[2]), str(PRE / f"{name}.orig"))
    staged.replace(LOG / name)
    print(f"      merged in place; 3 sources moved to PRE_MERGE/")

if APPLY and not failed:
    (PRE / 'README.txt').write_text(
        "Pre-merge sources for the 2026-06/07/08 PowerMonitor logs. Nothing here is\n"
        "unique -- every row is also in ../<name>, which is now one file per month.\n\n"
        "  <name>.recovered  the first ~10 h of the month, from a ZFS snapshot\n"
        "  <name>.1          the chunk http_server rotated aside\n"
        "  <name>.orig       the tail the sink was still appending to\n\n"
        "They were three files because http_server.py's size rotation reused a single\n"
        "'.1' and overwrote it, so a month arrived split and mostly destroyed. The\n"
        "merged file is a plain concatenation in time order; the three pieces did not\n"
        "overlap. Verify with: cat <name>.recovered <name>.1 <name>.orig | cmp - ../<name>\n"
        "Kept for reversibility; safe to delete once the merged months have been used.\n")
    for d in (REC,):
        left = [p for p in d.iterdir()] if d.is_dir() else []
        if left and all(p.name == 'README.txt' for p in left):
            shutil.move(str(d / 'README.txt'), str(PRE / 'RECOVERED-README.txt'))
            d.rmdir()
            print(f"\nRECOVERED/ removed (its rows now live in the month files)")

print(f"\n{'FAILED: ' + ', '.join(failed) if failed else 'all months OK'}")
sys.exit(1 if failed else 0)
