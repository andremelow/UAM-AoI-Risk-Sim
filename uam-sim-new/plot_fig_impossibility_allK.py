#!/usr/bin/env python3
"""plot_fig_impossibility_allK.py -- impossibility figure across ALL K, split layout,
colored in the split palette. 2 rows x 6 K columns:
  top    = fleet-mean time-average status AoI A(N) = mean_u (mean_t h_{u,s})
  bottom = worst-case per-UAV risk max_u R̄_u = max_u (mean_t R_u), with the
           design-envelope reference line at 1.
A(N) from drone_summary.mean_h1 (verified == raw per-UAV mean of h1_slots);
max_u R̄_u from drone_summary.mean_r_tot (verified == raw). Both flight-window.

Output: csv_export_RC7/plots/fig_impossibility_allK.{pdf,png}
"""
import os, glob
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
from matplotlib.lines import Line2D

DATA, SCEN = "csv_export_RC7", "4r_b7"
K_VALUES = [1, 2, 4, 8, 16, 32]
N_VALUES = [5, 10, 20, 40, 60, 80, 100, 120]
POL = ["round-robin-sw", "pf-classic-sw", "mr-mw-ch-sw"]
LABEL  = {"round-robin-sw": "RR", "pf-classic-sw": "PF", "mr-mw-ch-sw": "Risk-Aware"}
COLORS = {"round-robin-sw": "#F9A825", "pf-classic-sw": "#E53935", "mr-mw-ch-sw": "#8E24AA"}
MARKERS = {"round-robin-sw": "o", "pf-classic-sw": "s", "mr-mw-ch-sw": "*"}

def cell(K, C, pol):
    ds = sorted(glob.glob(os.path.join(DATA, f"K{K}", f"C{C:02d}", SCEN, f"{pol}_*_drone_summary.csv")))
    if not ds:
        return np.nan, np.nan
    d = pd.read_csv(ds[0], usecols=["mean_h1", "mean_r_tot"])
    return float(d.mean_h1.mean()), float(d.mean_r_tot.max())

# A[pol][K] = list over N ; R[pol][K] = list over N
A = {p: {} for p in POL}; R = {p: {} for p in POL}
for p in POL:
    for K in K_VALUES:
        A[p][K] = []; R[p][K] = []
        for C in N_VALUES:
            a, r = cell(K, C, p)
            A[p][K].append(a); R[p][K].append(r)

fig, axes = plt.subplots(2, len(K_VALUES), figsize=(3.0 * len(K_VALUES), 6.4),
                         sharex=True, constrained_layout=True, squeeze=False)

for ci, K in enumerate(K_VALUES):
    axa, axb = axes[0][ci], axes[1][ci]
    for p in POL:
        axa.plot(N_VALUES, A[p][K], color=COLORS[p], marker=MARKERS[p], ms=6, lw=2.0)
        axb.plot(N_VALUES, R[p][K], color=COLORS[p], marker=MARKERS[p], ms=6, lw=2.0)
    axa.set_title(f"K = {K}", fontweight="bold")
    axa.set_yscale("log"); axb.set_yscale("log")
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
        axa.set_ylabel("Mean AoI (slots)")
        axb.set_ylabel(r"$\max_u \bar R_u$")
    if ci == len(K_VALUES) - 1:
        axb.text(0.98, 1.0, "design envelope", transform=axb.get_yaxis_transform(),
                 ha="right", va="bottom", fontsize=7, color="0.4")

handles = [Line2D([0], [0], color=COLORS[p], marker=MARKERS[p], lw=2, label=LABEL[p]) for p in POL]
handles.append(Line2D([0], [0], color="0.4", ls=(0, (4, 3)), lw=1.0, label="design envelope (=1)"))
fig.legend(handles=handles, ncol=len(handles), loc="lower center",
           bbox_to_anchor=(0.5, -0.03), frameon=False, fontsize=10)

OUT = os.path.join(DATA, "plots")
os.makedirs(OUT, exist_ok=True)
base = os.path.join(OUT, "fig_impossibility_allK")
for ext in ("pdf", "png"):
    fig.savefig(f"{base}.{ext}", dpi=150, bbox_inches="tight")
    print("saved", f"{base}.{ext}")
