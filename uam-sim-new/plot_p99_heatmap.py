#!/usr/bin/env python3
"""plot_p99_heatmap.py -- P99 companion to the TLS envelope.

Cell metric = max_u P99(R_u(t)): per UAV, the 99th percentile of the
instantaneous normalized risk r_tot over its flight; then the MAX over the
fleet. More demanding than maxRbar (time-average) and quantile-interpretable:
"for 99% of the time, every UAV stays within x of the envelope".

Heatmap axes K x C identical to tls_companion; diverging colormap centered at
the TLS (=1, log color); cell values printed. Console: P99 acceptance frontier
= largest C with max_u P99 <= 1, per K.

Usage:
  python3 plot_p99_heatmap.py --dir csv_export_RC11 \
      --pol round-robin-sw,pf-classic-sw,mw-sw-v9,mw-v9-nsw
Output: <dir>/plots/p99_heatmap.{pdf,png} + p99_stats.csv (cache)
"""
import os, glob, argparse
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import TwoSlopeNorm

ap = argparse.ArgumentParser()
ap.add_argument("--dir", default="csv_export_RC11")
ap.add_argument("--scenario", default="4r_b7")
ap.add_argument("--q", type=float, default=0.99, help="quantile (default 0.99)")
ap.add_argument("--pol", default="round-robin-sw,pf-classic-sw,mw-sw-v9,mw-v9-nsw")
ap.add_argument("--min-slots", type=int, default=10)
ap.add_argument("--exclude-ramps", action="store_true",
                help="mask BOTH 500 m ramps (climb-out + landing outage spikes)")
ap.add_argument("--exclude-landing", action="store_true",
                help="mask each UAV's landing ramp (final 500 m of its route, "
                     "outage-driven risk spike); output files get _noland suffix")
ap.add_argument("--cached", action="store_true")
args = ap.parse_args()

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR   = os.path.join(SCRIPT_DIR, args.dir)
OUT_DIR    = os.path.join(DATA_DIR, "plots")
os.makedirs(OUT_DIR, exist_ok=True)
LTAG       = "_noramps" if args.exclude_ramps else ("_noland" if args.exclude_landing else "")
STATS_CSV  = os.path.join(OUT_DIR, f"p99_stats{LTAG}.csv")

K_VALUES = [1, 2, 4, 8, 16, 32]
C_VALUES = [5, 10, 20, 40, 60, 80, 100, 120]
POL       = [p for p in args.pol.split(",") if p]
POL_LABEL = {"round-robin-sw": "RR", "pf-classic-sw": "PF", "mr-mw-ch-sw": "Risk-Aware",
             "max-risk-pf-sw": "Risk-PF", "mw-sw-otimo-ch": "MW-opt (sw)",
             "mw-otimo-ch": "MW-opt", "mw-sw-v8": "MW-V8", "mw-sw-v9": "MW-V9",
             "mw-v9-nsw": "MW-V9-nsw"}

plt.rcParams.update({
    "font.family": "serif", "font.serif": ["Times New Roman", "Nimbus Roman", "DejaVu Serif"],
    "mathtext.fontset": "stix",
    "font.size": 8, "axes.titlesize": 9, "axes.labelsize": 8,
    "xtick.labelsize": 7, "ytick.labelsize": 7, "figure.dpi": 150,
})

def cell_p99(K, C, pol):
    fs = sorted(glob.glob(os.path.join(DATA_DIR, f"K{K}", f"C{C:02d}", args.scenario,
                                        f"{pol}_*_slot_series.csv")))
    if not fs:
        return np.nan
    mask_on = args.exclude_landing or args.exclude_ramps
    cols = ["drone_id", "r_tot"] + (["slot"] if mask_on else [])
    df = pd.read_csv(fs[-1], usecols=cols,
                     dtype={"drone_id": np.int32, "r_tot": np.float32})
    if mask_on:
        dsf = fs[-1].replace("_slot_series.csv", "_drone_summary.csv")
        meta = pd.read_csv(dsf, usecols=["drone_id", "entry_slot", "exit_slot"])
        meta["dur"] = meta.exit_slot - meta.entry_slot
        fr = np.where(meta.dur > meta.dur.min() * 1.5, 500.0/8000.0, 500.0/4264.4)
        meta["hi"] = meta.entry_slot + ((1.0 - fr) * meta.dur).astype(int)
        meta["lo"] = meta.entry_slot + (fr * meta.dur).astype(int)
        df = df.merge(meta[["drone_id", "lo", "hi"]], on="drone_id")
        df = df[df.slot < df.hi]
        if args.exclude_ramps:
            df = df[df.slot > df.lo]
    g = df.groupby("drone_id").r_tot
    q = g.quantile(args.q)
    keep = g.size() >= args.min_slots
    q = q[keep]
    return float(q.max()) if len(q) else np.nan

if args.cached and os.path.exists(STATS_CSV):
    stats = pd.read_csv(STATS_CSV)
    print("loaded", STATS_CSV)
else:
    rows = []
    for K in K_VALUES:
        for C in C_VALUES:
            for pol in POL:
                v = cell_p99(K, C, pol)
                if np.isnan(v):
                    print(f"  MISSING K{K} C{C} {pol}")
                    continue
                rows.append(dict(K=K, C=C, policy=pol, max_p99=v))
        print(f"  done K{K}")
    stats = pd.DataFrame(rows)
    stats.to_csv(STATS_CSV, index=False)
    print("saved", STATS_CSV, f"({len(stats)} cells)")

def matrix(pol):
    M = np.full((len(K_VALUES), len(C_VALUES)), np.nan)
    for i, K in enumerate(K_VALUES):
        for j, C in enumerate(C_VALUES):
            r = stats[(stats.K == K) & (stats.C == C) & (stats.policy == pol)]
            if len(r):
                M[i, j] = r.max_p99.values[0]
    return M

allv = stats.max_p99.values
lo, hi = np.log10(np.nanmin(allv[allv > 0])), np.log10(np.nanmax(allv))
norm = TwoSlopeNorm(vmin=min(2.5 * lo, -0.5), vcenter=0.0, vmax=max(hi, 1e-3))
cmap = plt.get_cmap("RdBu_r")

fig, axes = plt.subplots(1, len(POL), figsize=(7.16, 1.35 if len(POL) > 3 else 1.6),
                         constrained_layout=True, squeeze=False)
axes = axes.flatten()
for pi, pol in enumerate(POL):
    ax = axes[pi]
    M = matrix(pol)
    ax.imshow(np.log10(M), aspect="auto", origin="lower", cmap=cmap, norm=norm)
    ax.set_xticks(range(len(C_VALUES))); ax.set_xticklabels(C_VALUES)
    ax.set_yticks(range(len(K_VALUES))); ax.set_yticklabels(K_VALUES)
    ax.set_title(POL_LABEL.get(pol, pol), fontweight="bold")
    ax.set_xlabel("Simultaneous UAVs")
    if pi == 0:
        ax.set_ylabel(r"$K$")
    for i in range(len(K_VALUES)):
        for j in range(len(C_VALUES)):
            v = M[i, j]
            if np.isfinite(v):
                txt = f"{v:.2f}" if v < 1 else f"{v:.3g}"
                ax.text(j, i, txt, ha="center", va="center", fontsize=5.0,
                        color="black", fontweight="bold")
fig.text(1.005, 0.5, rf"$\max_u P_{{{int(args.q*100)}}}(R_u)$", rotation=90,
         va="center", ha="center", fontsize=9)
base = os.path.join(OUT_DIR, f"p99_heatmap{LTAG}")
for ext in ("pdf", "png"):
    fig.savefig(f"{base}.{ext}", bbox_inches="tight")
print("saved", base)

print(f"\n=== P{int(args.q*100)} acceptance frontier: largest C with max_u P99 <= 1 (per K) ===")
print("  K  " + "".join(f"{POL_LABEL.get(p, p):>14s}" for p in POL))
for K in K_VALUES:
    cells = []
    for pol in POL:
        r = stats[(stats.K == K) & (stats.policy == pol)].sort_values("C")
        acc = r[r.max_p99 <= 1.0]
        cells.append(f"{int(acc.C.max()):>14d}" if len(acc) else f"{'none':>14s}")
    print(f"  {K:<3d}" + "".join(cells))
