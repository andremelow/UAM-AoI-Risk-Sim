#!/usr/bin/env python3
"""plot_ccdf_cells.py -- CCDF of the instantaneous risk R_u(t) for selected
(K,C) cells, policies overlaid, log-log. P99 = crossing of the 1e-2 line.

CCDF pooled over (u,t) within the cell; vertical dashed line at the TLS (x=1),
horizontal dotted line at 1e-2 (P99) — the x where each curve crosses it is
that policy's pooled P99. Console prints pooled P99 and max_u P99 per policy.

Usage:
  python3 plot_ccdf_cells.py --dir csv_export_RC11 --cells 4:40,16:80 \
      --pol round-robin-sw,pf-classic-sw,mw-sw-v9,mw-v9-nsw
Output: <dir>/plots/fig_ccdf_cells_<tags>.{pdf,png}
"""
import os, glob, argparse
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

ap = argparse.ArgumentParser()
ap.add_argument("--dir", default="csv_export_RC11")
ap.add_argument("--scenario", default="4r_b7")
ap.add_argument("--cells", default="4:40,16:80", help="comma list of K:C")
ap.add_argument("--pol", default="round-robin-sw,pf-classic-sw,mw-sw-v9,mw-v9-nsw")
args = ap.parse_args()

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(SCRIPT_DIR, args.dir)
CELLS = [tuple(int(v) for v in c.split(":")) for c in args.cells.split(",")]

POL    = [p for p in args.pol.split(",") if p]
LABEL  = {"round-robin-sw": "RR", "pf-classic-sw": "PF", "mr-mw-ch-sw": "Risk-Aware",
          "max-risk-pf-sw": "Risk-PF", "mw-sw-otimo-ch": "MW-opt (sw)",
          "mw-otimo-ch": "MW-opt", "mw-sw-v8": "MW-V8", "mw-sw-v9": "MW-V9",
          "mw-v9-nsw": "MW-V9-nsw"}
COLORS = {"round-robin-sw": "#F9A825", "pf-classic-sw": "#E53935", "mr-mw-ch-sw": "#8E24AA",
          "max-risk-pf-sw": "#1E88E5", "mw-sw-otimo-ch": "#3949AB",
          "mw-otimo-ch": "#6D4C41", "mw-sw-v8": "#00897B", "mw-sw-v9": "#7CB342",
          "mw-v9-nsw": "#33691E"}

def load(K, C, pol):
    fs = sorted(glob.glob(os.path.join(DATA, f"K{K}", f"C{C:02d}", args.scenario,
                                        f"{pol}_*_slot_series.csv")))
    if not fs:
        return None
    return pd.read_csv(fs[-1], usecols=["drone_id", "r_tot"],
                       dtype={"drone_id": np.int32, "r_tot": np.float32})

def ccdf_xy(r, npts=1500):
    xs = np.sort(r[r > 0]); n = xs.size
    if n == 0:
        return np.array([]), np.array([])
    p = (n - np.arange(n)) / n
    idx = np.unique(np.linspace(0, n - 1, npts).astype(int))
    return xs[idx], p[idx]

plt.rcParams.update({"font.family": "serif",
                     "font.serif": ["Times New Roman", "Nimbus Roman", "DejaVu Serif"],
                     "mathtext.fontset": "stix", "font.size": 8,
                     "xtick.labelsize": 7, "ytick.labelsize": 7, "pdf.fonttype": 42})
PW = 3.55 if len(CELLS) <= 3 else 2.05          # narrower panels for long diagonals
fig, axes = plt.subplots(1, len(CELLS), figsize=(PW * len(CELLS), 2.6),
                         sharey=True, constrained_layout=True, squeeze=False)
axes = axes.flatten()

for ci, (K, C) in enumerate(CELLS):
    ax = axes[ci]
    print(f"\n=== K={K} C={C} ===")
    for p in POL:
        df = load(K, C, p)
        if df is None:
            print(f"  MISSING {p}")
            continue
        r = df.r_tot.values
        x, y = ccdf_xy(r)
        ax.loglog(x, y, color=COLORS[p], lw=1.7, label=LABEL.get(p, p))
        p99_pooled = float(np.quantile(r[r > 0], 0.99))
        p99_worst  = float(df.groupby("drone_id").r_tot.quantile(0.99).max())
        print(f"  {LABEL.get(p,p):11}: P99 pooled={p99_pooled:8.3f}   max_u P99={p99_worst:8.3f}")
    ax.axvline(1.0, ls=(0, (4, 3)), lw=0.9, color="0.35")
    if ci == len(CELLS) - 1:
        ax.text(1.0, 0.97, " TLS", transform=ax.get_xaxis_transform(),
                ha="left", va="top", fontsize=7, color="0.35")
    ax.axhline(1e-2, ls=":", lw=0.9, color="0.35")
    ax.text(0.02, 1.15e-2, "P99", transform=ax.get_yaxis_transform(),
            ha="left", va="bottom", fontsize=7, color="0.35")
    ax.set_ylim(1e-5, 1.1)
    ax.set_title(f"K = {K},  C = {C}", fontweight="bold")
    ax.set_xlabel(r"$x$  (normalized risk $R_u$)")
    if ci == 0:
        ax.set_ylabel(r"$P(R_u(t) > x)$")
    ax.grid(True, which="both", ls=":", lw=0.4, alpha=0.6)
axes[0].legend(frameon=False, fontsize=7.5, loc="lower left")

OUT = os.path.join(DATA, "plots")
os.makedirs(OUT, exist_ok=True)
tags = "_".join(f"K{K}C{C}" for K, C in CELLS)
base = os.path.join(OUT, f"fig_ccdf_cells_{tags}")
for ext in ("pdf", "png"):
    fig.savefig(f"{base}.{ext}", dpi=180, bbox_inches="tight")
    print("saved", f"{base}.{ext}")
