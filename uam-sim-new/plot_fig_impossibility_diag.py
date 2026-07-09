#!/usr/bin/env python3
"""plot_fig_impossibility_diag.py -- "impossibility" figure along the DIAGONAL of
the (K, N) grid: K and N scale together (N/K ~= 5), same color/marker convention
as the split figures.
  (a) fleet-mean C2 AoI  mean_u mean_h1
  (b) worst-case per-UAV risk max_u R̄_u = max_u mean_r_tot, envelope line at 1

Default diagonal: (K=1,N=5) (2,10) (4,20) (8,40) (16,80) (32,120)
(last cell is the grid corner, N/K=3.75 instead of 5 -- N=160 was not simulated).

All metrics from drone_summary (verified == raw per-UAV means).

Usage:
  python3 plot_fig_impossibility_diag.py --dir csv_export_RC8v8 \
      --pol round-robin-sw,pf-classic-sw,mr-mw-ch-sw,mw-sw-v8
  python3 plot_fig_impossibility_diag.py --diag 1:5,2:10,4:20,8:40,16:80
Output: <dir>/plots/fig_impossibility_diag.{pdf,png}
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
ap.add_argument("--diag", default="1:5,2:10,4:20,8:40,16:80,32:120",
                help="comma-separated K:N cells along the diagonal")
ap.add_argument("--pol", default="round-robin-sw,pf-classic-sw,mr-mw-ch-sw,mw-sw-v8")
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
MARKERS = {"round-robin-sw": "o", "pf-classic-sw": "s", "mr-mw-ch-sw": "*",
           "max-risk-pf-sw": "D", "mw-sw-otimo-ch": "v", "mw-otimo-ch": "P", "mw-sw-v9": "^", "mw-v9-nsw": "v",
           "mw-sw-v8": "X"}

def cell(K, N, pol):
    fs = sorted(glob.glob(os.path.join(DATA, f"K{K}", f"C{N:02d}", SCEN,
                                        f"{pol}_*_drone_summary.csv")))
    if not fs:
        print(f"  missing: K{K}/C{N:02d} {pol}")
        return np.nan, np.nan
    d = pd.read_csv(fs[-1], usecols=["mean_h1", "mean_r_tot"])
    return float(d.mean_h1.mean()), float(d.mean_r_tot.max())

A = {p: [] for p in POL}
R = {p: [] for p in POL}
for p in POL:
    for K, N in DIAG:
        a, r = cell(K, N, p)
        A[p].append(a); R[p].append(r)

X = [k for k, _ in DIAG]          # position along the diagonal, indexed by K

plt.rcParams.update({"font.family": "serif",
                     "font.serif": ["Times New Roman", "Nimbus Roman", "DejaVu Serif"],
                     "mathtext.fontset": "stix", "font.size": 9, "pdf.fonttype": 42})
fig, (axa, axb) = plt.subplots(2, 1, figsize=(4.2, 5.2), sharex=True,
                               constrained_layout=True)

for p in POL:
    axa.plot(X, A[p], color=COLORS[p], marker=MARKERS[p], ms=6, lw=2.0)
    axb.plot(X, R[p], color=COLORS[p], marker=MARKERS[p], ms=6, lw=2.0)

# ---- panel (a): mean C2 AoI, log y ----
axa.set_yscale("log")
axa.set_ylabel("Mean AoI (slots)")
axa.text(0.03, 0.92, "(a)", transform=axa.transAxes, fontweight="bold", va="top")
axa.grid(True, which="both", ls="--", lw=0.5, alpha=0.4)

# ---- panel (b): max_u R̄_u, log y, design-envelope line at 1 ----
axb.set_yscale("log")
axb.set_ylabel(r"$\max_u \bar R_u$")
axb.axhline(1.0, ls=(0, (4, 3)), lw=1.0, color="0.35")
axb.text(0.985, 1.0, "design envelope", transform=axb.get_yaxis_transform(),
         ha="right", va="bottom", fontsize=8, color="0.35")
axb.text(0.03, 0.92, "(b)", transform=axb.transAxes, fontweight="bold", va="top")
axb.grid(True, which="both", ls="--", lw=0.5, alpha=0.4)

# ---- shared x: log2 in K, two-line ticks "K / N" ----
axb.set_xscale("log", base=2)
axb.set_xlim(X[0] * 0.85, X[-1] * 1.18)
axb.set_xticks(X)
axb.xaxis.set_major_formatter(
    mticker.FixedFormatter([f"{k}\n{n}" for k, n in DIAG]))
axb.xaxis.set_minor_formatter(mticker.NullFormatter())
axb.tick_params(axis="x", which="minor", length=0)
axb.set_xlabel(r"Channels $K$  /  simultaneous UAVs $N$")

# ---- one legend for the whole figure, above panel (a) ----
handles = [Line2D([0], [0], color=COLORS[p], marker=MARKERS[p], lw=2,
                  label=LABEL.get(p, p)) for p in POL]
axa.legend(handles=handles, ncol=min(len(POL), 4), loc="lower center",
           bbox_to_anchor=(0.5, 1.0), columnspacing=1.2, handlelength=2.2,
           borderaxespad=0.2, frameon=False)

OUT = os.path.join(DATA, "plots")
os.makedirs(OUT, exist_ok=True)
base = os.path.join(OUT, "fig_impossibility_diag")
fig.savefig(base + ".pdf", bbox_inches="tight")
fig.savefig(base + ".png", dpi=200, bbox_inches="tight")
print("saved", base + ".pdf")
