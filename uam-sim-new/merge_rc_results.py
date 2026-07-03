#!/usr/bin/env python3
"""merge_rc_results.py -- merged VIEW of two RC export trees via symlinks.

Builds --out as a K*/C*/scenario tree of symlinks to the per-run CSVs of
--base and --extra, concatenates sweep_summary_all.csv and the per-cell
summary.csv of both. Every existing plot script then works unmodified on the
merged dir (e.g. --dir csv_export_RC8v9), seeing the union of policies.

Rationale: RC9 runs only mw-sw-v8 on the SAME grid/scenario/seed as RC8, so
merging cells is seed-for-seed consistent with the RC8 policies.

Usage:
  python3 merge_rc_results.py                       # RC8 + RC9 -> csv_export_RC8v9
  python3 merge_rc_results.py --base csv_export_RC8 --extra csv_export_RC9 \
                              --out csv_export_RC8v9
Re-run after new RC9 results: refreshes links and re-concatenates summaries.
"""
import os, glob, argparse
import pandas as pd

ap = argparse.ArgumentParser()
ap.add_argument("--base",  default="csv_export_RC8")
ap.add_argument("--extra", default="csv_export_RC9")
ap.add_argument("--out",   default="csv_export_RC8v8")
args = ap.parse_args()

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE  = os.path.join(SCRIPT_DIR, args.base)
EXTRA = os.path.join(SCRIPT_DIR, args.extra)
OUT   = os.path.join(SCRIPT_DIR, args.out)

n_link = 0
cells = set()
for root in (BASE, EXTRA):
    if not os.path.isdir(root):
        print(f"WARNING: {root} not found, skipping")
        continue
    # per-run CSVs: K*/C*/<scenario>/<policy>_*_{slot_series,drone_summary,manifest}.csv
    for f in glob.glob(os.path.join(root, "K*", "C*", "*", "*_*.csv")):
        rel = os.path.relpath(f, root)
        if os.path.basename(f) == "summary.csv":
            continue
        dst = os.path.join(OUT, rel)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        cells.add(os.path.dirname(rel))
        if os.path.islink(dst) or os.path.exists(dst):
            os.remove(dst)
        os.symlink(os.path.abspath(f), dst)
        n_link += 1
print(f"linked {n_link} per-run CSVs into {OUT}")

# per-cell summary.csv: concat rows of base+extra
n_cell = 0
for cell in sorted(cells):
    parts = []
    for root in (BASE, EXTRA):
        p = os.path.join(root, cell, "summary.csv")
        if os.path.exists(p):
            parts.append(pd.read_csv(p))
    if parts:
        pd.concat(parts, ignore_index=True).to_csv(
            os.path.join(OUT, cell, "summary.csv"), index=False)
        n_cell += 1
print(f"merged {n_cell} per-cell summary.csv")

# master sweep summary: concat
parts = []
for root in (BASE, EXTRA):
    p = os.path.join(root, "sweep_summary_all.csv")
    if os.path.exists(p):
        parts.append(pd.read_csv(p))
if parts:
    allrows = pd.concat(parts, ignore_index=True)
    allrows.to_csv(os.path.join(OUT, "sweep_summary_all.csv"), index=False)
    print(f"merged sweep_summary_all.csv: {len(allrows)} rows, "
          f"policies: {sorted(allrows.policy.unique())}")
