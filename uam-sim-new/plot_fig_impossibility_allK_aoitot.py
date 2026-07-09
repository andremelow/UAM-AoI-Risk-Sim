#!/usr/bin/env python3
"""plot_fig_impossibility_allK_aoitot.py -- freshness-vs-tail-risk across the
FULL (K, N) grid, split layout: 2 rows x 6 K columns.
  top    = fleet-mean TOTAL AoI  mean_u (mean_h1 + mean_h2)/2  [slots], log y
  bottom = fleet 99th percentile p99_u R̄_u, log y, envelope line at 1

Both from drone_summary (mean_h1, mean_h2, mean_r_tot).

Usage:
  python3 plot_fig_impossibility_allK_aoitot.py --dir csv_export_RC8v8 \
      --pol round-robin-sw,pf-classic-sw,mw-sw-v8,mw-sw-v9
Output: <dir>/plots/fig_impossibility_allK_aoitot_p99.{pdf,png}
"""
import os, glob, argparse
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
from matplotlib.lines import Line2D

ap = argparse.ArgumentParser()
ap.add_argument("--dir", default="csv_export_RC8v8")
ap.add_argument("--scenario", default="4r_b7")
ap.add_argument("--q", type=float, default=0.99)
ap.add_argument("--pol", default="round-robin-sw,pf-classic-sw,mw-sw-v8,mw-sw-v9")
args = ap.parse_args()

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(SCRIPT_DIR, args.dir)
SCEN = args.scenario
K_VALUES = [1, 2, 4, 8, 16, 32]
N_VALUES = [5, 10, 20, 40, 60, 80, 100, 120]

POL    = [p for p in args.pol.split(",") if p]
LABEL  = {"round-robin-sw": "RR", "pf-classic-sw": "PF", "mr-mw-ch-sw": "Risk-Aware",
          "max-risk-pf-sw": "Risk-PF", "mw-sw-otimo-ch": "MW-opt (sw)",
          "mw-otimo-ch": "MW-opt", "mw-sw-v8": "MW-V8", "mw-sw-v9": "MW-V9", "mw-v9-nsw": "MW-V9-nsw"}
COLORS = {"round-robin-sw": "#F9A825", "pf-classic-sw": "#E53935", "mr-mw-ch-sw": "#8E24AA",
          "max-risk-pf-sw": "#1E88E5", "mw-sw-otimo-ch": "#3949AB",
          "mw-otimo-ch": "#6D4C41", "mw-sw-v8": "#00897B", "mw-sw-v9": "#7CB342", "mw-v9-nsw": "#33691E"}
MARKERS = {"round-robin-sw": "o", "pf-classic-sw": "s", "mr-mw-ch-sw": "*",
           "max-risk-pf-sw": "D", "mw-sw-otimo-ch": "v", "mw-otimo-ch": "P", "mw-sw-v9": "^", "mw-v9-nsw": "v",
           "mw-sw-v8": "X"}

QPCT = int(round(100 * args.q))

def cell(K, C, pol):
    fs = sorted(glob.glob(os.path.join(DATA, f"K{K}", f"C{C:02d}", SCEN,
                                        f"{pol}_*_drone_summary.csv")))
    if not fs:
        print(f"  missing: K{K}/C{C:02d} {pol}")
        return np.nan, np.nan
    d = pd.read_csv(fs[-1], usecols=["mean_h1", "mean_h2", "mean_r_tot"])
    aoi_tot = float(((d.mean_h1 + d.mean_h2) / 2.0).mean())
    return aoi_tot, float(d.mean_r_tot.quantile(args.q))

# A[pol][K] / Q[pol][K] = list over N
A = {p: {} for p in POL}
Q = {p: {} for p in POL}
for p in POL:
    for K in K_VALUES:
        A[p][K] = []; Q[p][K] = []
        for C in N_VALUES:
            a, q = cell(K, C, p)
            A[p][K].append(a); Q[p][K].append(q)

plt.rcParams.update({"font.family": "serif",
                     "font.serif": ["Times New Roman", "Nimbus Roman", "DejaVu Serif"],
                     "mathtext.fontset": "stix", "font.size": 9, "pdf.fonttype": 42})
fig, axes = plt.subplots(2, len(K_VALUES), figsize=(3.0 * len(K_VALUES), 6.4),
                         sharex=True, constrained_layout=True, squeeze=False)

for ci, K in enumerate(K_VALUES):
    axa, axb = axes[0][ci], axes[1][ci]
    for p in POL:
        axa.plot(N_VALUES, A[p][K], color=COLORS[p], marker=MARKERS[p], ms=6, lw=2.0)
        axb.plot(N_VALUES, Q[p][K], color=COLORS[p], marker=MARKERS[p], ms=6, lw=2.0)
    axa.set_title(f"K = {K}", fontweight="bold")
    axa.set_yscale("log")
    axb.set_yscale("log")
    axb.axhline(1.0, ls=(0, (4, 3)), lw=1.0, color="0.4")
    for ax in (axa, axb):
        ax.set_xscale("log")
        ax.set_xlim(N_VALUES[0] * 0.9, N_VALUES[-1] * 1.08)
        ax.set_xticks(N_VALUES)
        ax.xaxis.set_major_formatter(mticker.FixedFormatter([str(n) for n in N_VALUES]))
        ax.xaxis.set_minor_formatter(mticker.NullFormatter())
        ax.tick_params(axis="x", which="minor", length=0)
        ax.tick_params(axis="x", labelsize=7)
        ax.grid(True, which="both", ls=":", lw=0.4, alpha=0.6)
    axb.set_xlabel(r"Simultaneous UAVs $N$")
    if ci == 0:
        axa.set_ylabel(r"Mean total AoI $\frac{1}{2}(\bar h_1 + \bar h_2)$ (slots)")
        axb.set_ylabel(rf"$p_{{{QPCT}}}\,[\bar R_u]$")
    if ci == len(K_VALUES) - 1:
        axb.text(0.98, 1.0, "design envelope", transform=axb.get_yaxis_transform(),
                 ha="right", va="bottom", fontsize=7, color="0.4")

handles = [Line2D([0], [0], color=COLORS[p], marker=MARKERS[p], lw=2,
                  label=LABEL.get(p, p)) for p in POL]
handles.append(Line2D([0], [0], color="0.4", ls=(0, (4, 3)), lw=1.0,
                      label="design envelope (=1)"))
fig.legend(handles=handles, ncol=len(handles), loc="lower center",
           bbox_to_anchor=(0.5, -0.03), frameon=False, fontsize=10)

OUT = os.path.join(DATA, "plots")
os.makedirs(OUT, exist_ok=True)
base = os.path.join(OUT, f"fig_impossibility_allK_aoitot_p{QPCT}")
for ext in ("pdf", "png"):
    fig.savefig(f"{base}.{ext}", dpi=150, bbox_inches="tight")
    print("saved", f"{base}.{ext}")
