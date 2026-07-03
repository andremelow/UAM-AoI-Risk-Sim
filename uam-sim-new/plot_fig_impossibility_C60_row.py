#!/usr/bin/env python3
"""plot_fig_impossibility_C60_row.py -- page-top impossibility strip at fixed C,
x = K: ONE ROW x FIVE COLUMNS, split-figure color convention (IEEE double column).
  (a) fleet-mean C2 AoI     mean_u mean_h1
  (b) fleet-mean video AoI  mean_u mean_h2
  (c) fleet-mean risk       mean_u R̄_u = mean_u mean_r_tot
  (d) worst-case per-UAV risk  max_u R̄_u = max_u mean_r_tot, envelope line at 1
  (e) unsafe-fleet fraction P(R̄_u > 1)  [%]  (same metric as split_pexceed top row)

All metrics from drone_summary (verified == raw per-UAV means).

Usage:
  python3 plot_fig_impossibility_C60_row.py --dir csv_export_RC8v8 --C 60 \
      --pol round-robin-sw,pf-classic-sw,mr-mw-ch-sw,mw-sw-v8
Output: <dir>/plots/fig_impossibility_C{C}_row.{pdf,png}
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
ap.add_argument("--C", type=int, default=60)
ap.add_argument("--thr", type=float, default=1.0)
ap.add_argument("--pol", default="round-robin-sw,pf-classic-sw,mr-mw-ch-sw,mw-sw-v8")
args = ap.parse_args()

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(SCRIPT_DIR, args.dir)
SCEN, C = args.scenario, args.C
K_VALUES = [1, 2, 4, 8, 16, 32]

POL    = [p for p in args.pol.split(",") if p]
LABEL  = {"round-robin-sw": "RR", "pf-classic-sw": "PF", "mr-mw-ch-sw": "Risk-Aware",
          "max-risk-pf-sw": "Risk-PF", "mw-sw-otimo-ch": "MW-opt (sw)",
          "mw-otimo-ch": "MW-opt", "mw-sw-v8": "MW-V8"}
COLORS = {"round-robin-sw": "#F9A825", "pf-classic-sw": "#E53935", "mr-mw-ch-sw": "#8E24AA",
          "max-risk-pf-sw": "#1E88E5", "mw-sw-otimo-ch": "#3949AB",
          "mw-otimo-ch": "#6D4C41", "mw-sw-v8": "#00897B"}
MARKERS = {"round-robin-sw": "o", "pf-classic-sw": "s", "mr-mw-ch-sw": "*",
           "max-risk-pf-sw": "D", "mw-sw-otimo-ch": "v", "mw-otimo-ch": "P",
           "mw-sw-v8": "X"}

def cell(K, pol):
    fs = sorted(glob.glob(os.path.join(DATA, f"K{K}", f"C{C:02d}", SCEN,
                                        f"{pol}_*_drone_summary.csv")))
    if not fs:
        return np.nan, np.nan, np.nan, np.nan
    d = pd.read_csv(fs[-1], usecols=["mean_h1", "mean_h2", "mean_r_tot"])
    return (float(d.mean_h1.mean()), float(d.mean_h2.mean()),
            float(d.mean_r_tot.mean()), float(d.mean_r_tot.max()),
            100.0 * float((d.mean_r_tot > args.thr).mean()))

M = {p: {k: [] for k in "abcde"} for p in POL}
for p in POL:
    for K in K_VALUES:
        vals = cell(K, p)
        for k, v in zip("abcde", vals):
            M[p][k].append(v)

plt.rcParams.update({"font.family": "serif",
                     "font.serif": ["Times New Roman", "Nimbus Roman", "DejaVu Serif"],
                     "mathtext.fontset": "stix", "font.size": 8,
                     "axes.labelsize": 8, "xtick.labelsize": 7, "ytick.labelsize": 7,
                     "pdf.fonttype": 42})
fig, axes = plt.subplots(1, 5, figsize=(7.16, 1.75), constrained_layout=True)
axa, axb, axm, axc, axd = axes

# INVERTED axes: metric on x, K on y (shared fleet of horizontal profiles)
for p in POL:
    kw = dict(color=COLORS[p], marker=MARKERS[p], ms=4, lw=1.5)
    axa.plot(M[p]["a"], K_VALUES, **kw)
    axb.plot(M[p]["b"], K_VALUES, **kw)
    axm.plot(M[p]["c"], K_VALUES, **kw)
    axc.plot(M[p]["d"], K_VALUES, **kw)
    axd.plot(M[p]["e"], K_VALUES, **kw)

axa.set_xscale("log")
axa.set_xlabel("Mean C2 AoI (slots)")
axa.text(0.05, 0.94, "(a)", transform=axa.transAxes, fontweight="bold", va="top")

axb.set_xscale("log")
axb.set_xlabel("Mean video AoI (slots)")
axb.text(0.05, 0.94, "(b)", transform=axb.transAxes, fontweight="bold", va="top")

axm.set_xscale("log")
axm.set_xlabel(r"$\mathrm{mean}_u\, \bar R_u$")
axm.axvline(1.0, ls=(0, (4, 3)), lw=0.8, color="0.35")
axm.text(0.05, 0.94, "(c)", transform=axm.transAxes, fontweight="bold", va="top")

axc.set_xscale("log")
axc.set_xlabel(r"$\max_u \bar R_u$")
axc.axvline(1.0, ls=(0, (4, 3)), lw=0.8, color="0.35")
axc.text(1.0, 0.97, " TLS", transform=axc.get_xaxis_transform(),
         ha="left", va="top", fontsize=6.5, color="0.35")
axc.text(0.05, 0.94, "(d)", transform=axc.transAxes, fontweight="bold", va="top")

axd.set_xlabel(r"$P(\bar R_u > 1)$  [%]")
axd.set_xlim(-4, 104)
axd.text(0.05, 0.94, "(e)", transform=axd.transAxes, fontweight="bold", va="top")

for ax in axes:
    ax.set_yscale("log", base=2)
    ax.set_yticks(K_VALUES)
    ax.yaxis.set_major_formatter(mticker.FixedFormatter([str(k) for k in K_VALUES]))
    ax.yaxis.set_minor_formatter(mticker.NullFormatter())
    ax.tick_params(axis="y", which="minor", length=0)
    ax.grid(True, which="both", ls=":", lw=0.4, alpha=0.6)
axa.set_ylabel(r"Channels $K$")
for ax in axes[1:]:
    ax.tick_params(labelleft=False)
for ax in axes:
    ax.invert_yaxis()          # K=1 at the top


handles = [Line2D([0], [0], color=COLORS[p], marker=MARKERS[p], lw=1.6, ms=4.5,
                  label=LABEL.get(p, p)) for p in POL]
fig.legend(handles=handles, ncol=len(POL), loc="lower center",
           bbox_to_anchor=(0.5, -0.06), frameon=False, fontsize=8,
           columnspacing=1.6, handlelength=2.2)

OUT = os.path.join(DATA, "plots")
os.makedirs(OUT, exist_ok=True)
base = os.path.join(OUT, f"fig_impossibility_C{C}_1x5_inv")
for ext in ("pdf", "png"):
    fig.savefig(f"{base}.{ext}", dpi=200, bbox_inches="tight")
    print("saved", f"{base}.{ext}")
