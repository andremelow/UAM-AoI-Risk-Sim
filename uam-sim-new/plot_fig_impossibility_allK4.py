#!/usr/bin/env python3
"""plot_fig_impossibility_allK4.py -- impossibility figure across ALL K:
4 metric rows x 6 K columns, x = C, split color convention.
  row 1: fleet-mean C2 AoI     mean_u mean_h1        (log y)
  row 2: fleet-mean video AoI  mean_u mean_h2        (log y)
  row 3: worst-case risk       max_u R̄_u             (log y, TLS line at 1)
  row 4: unsafe-fleet fraction P(R̄_u > 1)  [%]       (linear)

All metrics from drone_summary (latest run per cell).

Usage:
  python3 plot_fig_impossibility_allK4.py --dir csv_export_RC11 \
      --pol round-robin-sw,pf-classic-sw,mw-sw-v9,mw-v9-nsw
Output: <dir>/plots/fig_impossibility_allK4.{pdf,png}
"""
import os, glob, argparse
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
from matplotlib.lines import Line2D

ap = argparse.ArgumentParser()
ap.add_argument("--dir", default="csv_export_RC11")
ap.add_argument("--scenario", default="4r_b7")
ap.add_argument("--thr", type=float, default=1.0)
ap.add_argument("--pol", default="round-robin-sw,pf-classic-sw,mw-sw-v9,mw-v9-nsw")
args = ap.parse_args()

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(SCRIPT_DIR, args.dir)
SCEN = args.scenario
K_VALUES = [1, 2, 4, 8, 16, 32]
C_VALUES = [5, 10, 20, 40, 60, 80, 100, 120]

POL    = [p for p in args.pol.split(",") if p]
LABEL  = {"round-robin-sw": "RR", "pf-classic-sw": "PF", "mr-mw-ch-sw": "Risk-Aware",
          "max-risk-pf-sw": "Risk-PF", "mw-sw-otimo-ch": "MW-opt (sw)",
          "mw-otimo-ch": "MW-opt", "mw-sw-v8": "MW-V8", "mw-sw-v9": "MW-V9",
          "mw-v9-nsw": "MW-V9-nsw"}
COLORS = {"round-robin-sw": "#F9A825", "pf-classic-sw": "#E53935", "mr-mw-ch-sw": "#8E24AA",
          "max-risk-pf-sw": "#1E88E5", "mw-sw-otimo-ch": "#3949AB",
          "mw-otimo-ch": "#6D4C41", "mw-sw-v8": "#00897B", "mw-sw-v9": "#7CB342",
          "mw-v9-nsw": "#33691E"}
MARKERS = {"round-robin-sw": "o", "pf-classic-sw": "s", "mr-mw-ch-sw": "*",
           "max-risk-pf-sw": "D", "mw-sw-otimo-ch": "v", "mw-otimo-ch": "P",
           "mw-sw-v8": "X", "mw-sw-v9": "^", "mw-v9-nsw": "v"}

def cell(K, C, pol):
    fs = sorted(glob.glob(os.path.join(DATA, f"K{K}", f"C{C:02d}", SCEN,
                                        f"{pol}_*_drone_summary.csv")))
    if not fs:
        return (np.nan,) * 4
    d = pd.read_csv(fs[-1], usecols=["mean_h1", "mean_h2", "mean_r_tot"])
    return (float(d.mean_h1.mean()), float(d.mean_h2.mean()),
            float(d.mean_r_tot.max()), 100.0 * float((d.mean_r_tot > args.thr).mean()))

# M[pol][row][ki] = list over C
M = {p: [[[np.nan]*len(C_VALUES) for _ in K_VALUES] for _ in range(4)] for p in POL}
for p in POL:
    for ki, K in enumerate(K_VALUES):
        for ci, C in enumerate(C_VALUES):
            vals = cell(K, C, p)
            for r in range(4):
                M[p][r][ki][ci] = vals[r]

ROW_LABEL = ["Mean C2 AoI (slots)", "Mean video AoI (slots)",
             r"$\max_u \bar R_u$", r"$P(\bar R_u > 1)$  [%]"]

plt.rcParams.update({"font.family": "serif",
                     "font.serif": ["Times New Roman", "Nimbus Roman", "DejaVu Serif"],
                     "mathtext.fontset": "stix", "font.size": 8,
                     "axes.labelsize": 8, "xtick.labelsize": 6.5, "ytick.labelsize": 7,
                     "pdf.fonttype": 42})
fig, axes = plt.subplots(4, len(K_VALUES), figsize=(2.0*len(K_VALUES), 8.0),
                         constrained_layout=True, sharex=True, sharey="row")

for ki, K in enumerate(K_VALUES):
    for r in range(4):
        ax = axes[r][ki]
        for p in POL:
            ax.plot(C_VALUES, M[p][r][ki], color=COLORS[p], marker=MARKERS[p],
                    ms=4, lw=1.5)
        if r == 0:
            ax.set_title(f"K = {K}", fontweight="bold")
        if r < 3:
            ax.set_yscale("log")
        if r == 2:
            ax.axhline(1.0, ls=(0, (4, 3)), lw=0.8, color="0.35")
            if ki == len(K_VALUES) - 1:
                ax.text(0.97, 1.0, "TLS", transform=ax.get_yaxis_transform(),
                        ha="right", va="bottom", fontsize=6.5, color="0.35")
        if r == 3:
            ax.set_ylim(-4, 104)
            ax.set_xlabel("Simultaneous UAVs")
        if ki == 0:
            ax.set_ylabel(ROW_LABEL[r])
        ax.set_xscale("log")
        ax.set_xticks(C_VALUES)
        ax.xaxis.set_major_formatter(mticker.FixedFormatter([str(c) for c in C_VALUES]))
        ax.xaxis.set_minor_formatter(mticker.NullFormatter())
        ax.tick_params(axis="x", which="minor", length=0, rotation=90)
        ax.grid(True, which="both", ls=":", lw=0.4, alpha=0.6)

handles = [Line2D([0], [0], color=COLORS[p], marker=MARKERS[p], lw=1.6, ms=4.5,
                  label=LABEL.get(p, p)) for p in POL]
fig.legend(handles=handles, ncol=len(POL), loc="lower center",
           bbox_to_anchor=(0.5, -0.03), frameon=False, fontsize=9,
           columnspacing=1.8, handlelength=2.4)

OUT = os.path.join(DATA, "plots")
os.makedirs(OUT, exist_ok=True)
base = os.path.join(OUT, "fig_impossibility_allK4")
for ext in ("pdf", "png"):
    fig.savefig(f"{base}.{ext}", dpi=170, bbox_inches="tight")
    print("saved", f"{base}.{ext}")
