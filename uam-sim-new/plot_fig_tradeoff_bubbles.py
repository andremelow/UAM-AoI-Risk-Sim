#!/usr/bin/env python3
"""plot_fig_tradeoff_bubbles.py -- simplified freshness-vs-safety figure.
One panel over the DIAGONAL of the (K, N) grid:
  y        = fleet 99th percentile p99_u R̄_u  (position = safety)
  bubble   = fleet-mean TOTAL AoI (mean_h1+mean_h2)/2  (size = freshness cost)
Shaded band: TLS-feasible region p99 <= 1.

Usage:
  python3 plot_fig_tradeoff_bubbles.py --dir csv_export_RC8v8 \
      --pol round-robin-sw,pf-classic-sw,mw-sw-v8,mw-sw-v9
Output: <dir>/plots/fig_tradeoff_bubbles.{pdf,png}
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
ap.add_argument("--diag", default="1:5,2:10,4:20,8:40,16:80,32:120")
ap.add_argument("--pol", default="round-robin-sw,pf-classic-sw,mw-sw-v8,mw-sw-v9")
args = ap.parse_args()

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(SCRIPT_DIR, args.dir)
SCEN = args.scenario
DIAG = [tuple(int(v) for v in kc.split(":")) for kc in args.diag.split(",") if kc]

POL    = [p for p in args.pol.split(",") if p]
LABEL  = {"round-robin-sw": "RR", "pf-classic-sw": "PF", "mr-mw-ch-sw": "Risk-Aware",
          "max-risk-pf-sw": "Risk-PF", "mw-sw-otimo-ch": "MW-opt (sw)",
          "mw-otimo-ch": "MW-opt", "mw-sw-v8": "MW-V8", "mw-sw-v9": "MW-V9", "mw-v9-nsw": "MW-V9-nsw"}
COLORS = {"round-robin-sw": "#F9A825", "pf-classic-sw": "#E53935", "mr-mw-ch-sw": "#8E24AA",
          "max-risk-pf-sw": "#1E88E5", "mw-sw-otimo-ch": "#3949AB",
          "mw-otimo-ch": "#6D4C41", "mw-sw-v8": "#00897B", "mw-sw-v9": "#7CB342", "mw-v9-nsw": "#33691E"}

def cell(K, C, pol):
    fs = sorted(glob.glob(os.path.join(DATA, f"K{K}", f"C{C:02d}", SCEN,
                                        f"{pol}_*_drone_summary.csv")))
    if not fs:
        return np.nan, np.nan
    d = pd.read_csv(fs[-1], usecols=["mean_h1", "mean_h2", "mean_r_tot"])
    return (float(((d.mean_h1 + d.mean_h2) / 2.0).mean()),
            float(d.mean_r_tot.quantile(args.q)))

AOI_TO_PTS2 = 0.55          # bubble area (pt^2) per slot of mean total AoI

plt.rcParams.update({"font.family": "serif",
                     "font.serif": ["Times New Roman", "Nimbus Roman", "DejaVu Serif"],
                     "mathtext.fontset": "stix", "font.size": 9, "pdf.fonttype": 42})
fig, ax = plt.subplots(figsize=(4.6, 3.2), constrained_layout=True)

X = [k for k, _ in DIAG]
# small multiplicative x-offset per policy so bubbles at the same scale don't overlap
offsets = np.linspace(-0.10, 0.10, len(POL))

for p, off in zip(POL, offsets):
    xs, ys, ss = [], [], []
    for (K, N) in DIAG:
        a, q = cell(K, N, p)
        xs.append(K * (2 ** off)); ys.append(q)
        ss.append(max(a * AOI_TO_PTS2, 8))
    ax.plot(xs, ys, color=COLORS[p], lw=1.0, alpha=0.45, zorder=2)
    ax.scatter(xs, ys, s=ss, color=COLORS[p], alpha=0.85,
               edgecolors="white", linewidths=0.6, zorder=3)

ax.set_xscale("log", base=2)
ax.set_yscale("log")
ax.set_xticks(X)
ax.xaxis.set_major_formatter(mticker.FixedFormatter([f"{k}\n{n}" for k, n in DIAG]))
ax.xaxis.set_minor_formatter(mticker.NullFormatter())
ax.tick_params(axis="x", which="minor", length=0)
ax.set_xlim(X[0] * 0.7, X[-1] * 1.45)
ax.set_xlabel(r"Channels $K$  /  simultaneous UAVs $N$")
ax.set_ylabel(rf"$p_{{{int(round(100*args.q))}}}\,[\bar R_u]$")

# TLS-feasible band
ylo, yhi = ax.get_ylim()
ax.axhspan(ylo, 1.0, color="#7CB342", alpha=0.10, zorder=0)
ax.axhline(1.0, ls=(0, (4, 3)), lw=1.0, color="0.35")
ax.text(0.02, 0.97, "TLS-feasible ($p_{99} \\leq 1$)",
        transform=ax.get_yaxis_transform(), ha="left", va="top",
        fontsize=7.5, color="0.30")
ax.set_ylim(ylo, yhi)
ax.grid(True, which="both", ls=":", lw=0.4, alpha=0.5)

# color legend (policies)
pol_handles = [Line2D([0], [0], marker="o", color=COLORS[p], lw=0, ms=7,
                      label=LABEL.get(p, p)) for p in POL]
leg1 = ax.legend(handles=pol_handles, loc="lower left", fontsize=7.5,
                 frameon=False, handletextpad=0.4, borderaxespad=0.3)
ax.add_artist(leg1)

# size legend (mean total AoI)
size_vals = [25, 100, 400]
size_handles = [plt.scatter([], [], s=v * AOI_TO_PTS2, color="0.55", alpha=0.8,
                            edgecolors="white", linewidths=0.6,
                            label=f"{v}") for v in size_vals]
ax.legend(handles=size_handles, title="mean total AoI (slots)",
          loc="upper right", fontsize=7, title_fontsize=7.5, frameon=False,
          labelspacing=1.1, handletextpad=0.6, borderaxespad=0.3, scatterpoints=1)

OUT = os.path.join(DATA, "plots")
os.makedirs(OUT, exist_ok=True)
base = os.path.join(OUT, "fig_tradeoff_bubbles")
fig.savefig(base + ".pdf", bbox_inches="tight")
fig.savefig(base + ".png", dpi=200, bbox_inches="tight")
print("saved", base + ".pdf")
