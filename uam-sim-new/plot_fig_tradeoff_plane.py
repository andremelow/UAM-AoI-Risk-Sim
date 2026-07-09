#!/usr/bin/env python3
"""plot_fig_tradeoff_plane.py -- freshness-vs-safety trade-off plane.
  x = fleet-mean TOTAL AoI  mean_u (mean_h1 + mean_h2)/2  [slots]
  y = fleet 99th percentile p99_u R̄_u
Each policy is a trajectory over the DIAGONAL of the (K, N) grid
(K=1,N=5 ... K=32,N=120); faint dots show every (K, N) cell of the full
grid. Shaded band: TLS-feasible region p99 <= 1.

Usage:
  python3 plot_fig_tradeoff_plane.py --dir csv_export_RC8v8 \
      --pol round-robin-sw,pf-classic-sw,mw-sw-v8,mw-sw-v9
Output: <dir>/plots/fig_tradeoff_plane.{pdf,png}
"""
import os, glob, argparse
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

ap = argparse.ArgumentParser()
ap.add_argument("--dir", default="csv_export_RC8v8")
ap.add_argument("--scenario", default="4r_b7")
ap.add_argument("--q", type=float, default=0.99)
ap.add_argument("--diag", default="1:5,2:10,4:20,8:40,16:80,32:120")
ap.add_argument("--pol", default="round-robin-sw,pf-classic-sw,mw-sw-v8,mw-sw-v9")
args = ap.parse_args()

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(SCRIPT_DIR, args.dir)
SCEN = args.scenario
DIAG = [tuple(int(v) for v in kc.split(":")) for kc in args.diag.split(",") if kc]
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

def cell(K, C, pol):
    fs = sorted(glob.glob(os.path.join(DATA, f"K{K}", f"C{C:02d}", SCEN,
                                        f"{pol}_*_drone_summary.csv")))
    if not fs:
        return np.nan, np.nan
    d = pd.read_csv(fs[-1], usecols=["mean_h1", "mean_h2", "mean_r_tot"])
    return (float(((d.mean_h1 + d.mean_h2) / 2.0).mean()),
            float(d.mean_r_tot.quantile(args.q)))

plt.rcParams.update({"font.family": "serif",
                     "font.serif": ["Times New Roman", "Nimbus Roman", "DejaVu Serif"],
                     "mathtext.fontset": "stix", "font.size": 9, "pdf.fonttype": 42})
fig, ax = plt.subplots(figsize=(4.2, 3.4), constrained_layout=True)

# faint cloud: every cell of the full grid
for p in POL:
    xs, ys = [], []
    for K in K_VALUES:
        for C in N_VALUES:
            a, q = cell(K, C, p)
            xs.append(a); ys.append(q)
    ax.scatter(xs, ys, s=8, color=COLORS[p], alpha=0.18, linewidths=0, zorder=1)

# diagonal trajectories: marker size grows with scale
sizes = np.linspace(28, 110, len(DIAG))
for p in POL:
    xs, ys = [], []
    for (K, N) in DIAG:
        a, q = cell(K, N, p)
        xs.append(a); ys.append(q)
    ax.plot(xs, ys, color=COLORS[p], lw=1.6, alpha=0.9, zorder=2)
    ax.scatter(xs, ys, s=sizes, color=COLORS[p], marker=MARKERS[p],
               edgecolors="white", linewidths=0.6, zorder=3)

# TLS-feasible band
ax.set_xscale("log"); ax.set_yscale("log")
ax.autoscale(); ax.set_xmargin(0.06)
ylo, yhi = ax.get_ylim()
ax.axhspan(ylo, 1.0, color="#7CB342", alpha=0.10, zorder=0)
ax.axhline(1.0, ls=(0, (4, 3)), lw=1.0, color="0.35")
ax.text(0.985, 0.985, "TLS-feasible ($p_{99} \\leq 1$)",
        transform=ax.get_yaxis_transform(), ha="right", va="top",
        fontsize=7.5, color="0.30")
ax.set_ylim(ylo, yhi)

ax.set_xlabel(r"Fleet-mean total AoI $\frac{1}{2}(\bar h_1 + \bar h_2)$ (slots)")
ax.set_ylabel(rf"$p_{{{int(round(100*args.q))}}}\,[\bar R_u]$")
ax.grid(True, which="both", ls=":", lw=0.4, alpha=0.5)

handles = [Line2D([0], [0], color=COLORS[p], marker=MARKERS[p], lw=1.6, ms=6,
                  label=LABEL.get(p, p)) for p in POL]
handles.append(Line2D([0], [0], marker="o", color="0.5", lw=0, ms=3, alpha=0.5,
                      label="all $(K,N)$ cells"))
ax.legend(handles=handles, loc="upper left", fontsize=7.5, frameon=False,
          handlelength=1.8, borderaxespad=0.3)

ax.annotate("scale $\\uparrow$ ($K{=}1..32$, $N{=}5..120$)",
            xy=(0.03, 0.03), xycoords="axes fraction", fontsize=7, color="0.35")

OUT = os.path.join(DATA, "plots")
os.makedirs(OUT, exist_ok=True)
base = os.path.join(OUT, "fig_tradeoff_plane")
fig.savefig(base + ".pdf", bbox_inches="tight")
fig.savefig(base + ".png", dpi=200, bbox_inches="tight")
print("saved", base + ".pdf")
