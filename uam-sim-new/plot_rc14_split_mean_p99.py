#!/usr/bin/env python3
"""plot_rc14_split_mean_p99.py -- risk split over the (K, N) grid:
  top row    : fleet-mean risk  mean_u R_bar_u
  bottom row : fleet p99        p99_u [R_bar_u]   (the TLS target metric)
One column per K; both rows from drone_summary.mean_r_tot only.

Usage:
  python3 plot_rc14_split_mean_p99.py --dir csv_export_RC14 \
      --pol round-robin-sw,pf-classic-sw,mw-sw-v8,mw-z,mw-sw-v9,mw-z-v9
Output: <dir>/plots/risk_split_mean_p99.{pdf,png}
"""
import os, glob, argparse
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

ap = argparse.ArgumentParser()
ap.add_argument("--dir", default="csv_export_RC14")
ap.add_argument("--scenario", default="4r_b7")
ap.add_argument("--q", type=float, default=0.99)
ap.add_argument("--pol", default="round-robin-sw,pf-classic-sw,mw-sw-v8,mw-z,"
                               "mw-sw-v9,mw-z-v9")
args = ap.parse_args()

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(SCRIPT_DIR, args.dir)
SCEN = args.scenario
K_VALUES = [1, 2, 4, 8, 16, 32]
N_VALUES = [5, 10, 20, 40, 60, 80, 100, 120]

POL    = [p for p in args.pol.split(",") if p]
LABEL  = {"round-robin-sw": "RR", "pf-classic-sw": "PF", "mw-sw-v8": "MW-V8",
          "mw-z": "MW-Z", "mw-sw-v9": "MW-V9", "mw-z-v9": "MW-Z-V9"}
COLORS = {"round-robin-sw": "#F9A825", "pf-classic-sw": "#E53935",
          "mw-sw-v8": "#00897B", "mw-z": "#1E88E5",
          "mw-sw-v9": "#7CB342", "mw-z-v9": "#6A1B9A"}
MARKERS = {"round-robin-sw": "o", "pf-classic-sw": "s", "mw-sw-v8": "X",
           "mw-z": "D", "mw-sw-v9": "^", "mw-z-v9": "P"}

def cell(K, C, pol):
    fs = sorted(glob.glob(os.path.join(DATA, f"K{K}", f"C{C:02d}", SCEN,
                                        f"{pol}_*_drone_summary.csv")))
    if not fs:
        return np.nan, np.nan
    d = pd.read_csv(fs[-1], usecols=["mean_r_tot"])
    return float(d.mean_r_tot.mean()), float(d.mean_r_tot.quantile(args.q))

plt.rcParams.update({"font.family": "serif",
                     "font.serif": ["Times New Roman", "Nimbus Roman", "DejaVu Serif"],
                     "mathtext.fontset": "stix", "font.size": 9, "pdf.fonttype": 42})
fig, axes = plt.subplots(2, len(K_VALUES), figsize=(2.05*len(K_VALUES), 5.4),
                         sharex=True, sharey="row", constrained_layout=True)

for kc, K in enumerate(K_VALUES):
    axm, axq = axes[0, kc], axes[1, kc]
    for p in POL:
        vals = [cell(K, N, p) for N in N_VALUES]
        axm.plot(N_VALUES, [v[0] for v in vals], color=COLORS[p],
                 marker=MARKERS[p], ms=4.5, lw=1.4)
        axq.plot(N_VALUES, [v[1] for v in vals], color=COLORS[p],
                 marker=MARKERS[p], ms=4.5, lw=1.4)
    for ax in (axm, axq):
        ax.set_yscale("log")
        ax.set_xscale("log")
        ax.set_xticks(N_VALUES)
        ax.set_xticklabels([str(n) for n in N_VALUES], fontsize=6.5, rotation=0)
        ax.tick_params(axis="x", which="minor", bottom=False)
        ax.axhline(1.0, ls=(0, (4, 3)), lw=0.9, color="0.35")
        ax.grid(True, which="both", ls=":", lw=0.35, alpha=0.55)
    axm.set_title(f"K = {K}", fontsize=10, fontweight="bold")
    axq.set_xlabel("Simultaneous UAVs $N$", fontsize=8)

axes[0, 0].set_ylabel(r"Fleet-mean risk  $\mathrm{mean}_u\,\bar R_u$")
axes[1, 0].set_ylabel(rf"$p_{{{int(round(100*args.q))}}}\,[\bar R_u]$")
axes[0, -1].text(N_VALUES[-1], 1.0, " envelope", fontsize=6.5, color="0.35",
                 va="bottom", ha="right")

handles = [Line2D([0], [0], color=COLORS[p], marker=MARKERS[p], lw=1.6, ms=6,
                  label=LABEL.get(p, p)) for p in POL]
handles.append(Line2D([0], [0], ls=(0, (4, 3)), color="0.35", lw=1,
                      label="design envelope (=1)"))
fig.legend(handles=handles, ncol=len(handles), loc="lower center",
           bbox_to_anchor=(0.5, -0.05), frameon=False, fontsize=8.5)

OUT = os.path.join(DATA, "plots")
os.makedirs(OUT, exist_ok=True)
base = os.path.join(OUT, "risk_split_mean_p99")
for ext in ("pdf", "png"):
    fig.savefig(f"{base}.{ext}", dpi=200, bbox_inches="tight")
    print("saved", f"{base}.{ext}")
