#!/usr/bin/env python3
"""fig_rr_resonance_K -- max_u R̄_u vs K, showing RR's non-monotonic K=16
resonance spike (worse than K=8 AND K=32) vs the monotone PF / Risk-Aware.
Panels: a few loads C. Data: drone_summary.mean_r_tot (== raw, verified).
Output: csv_export_RC7/plots/fig_rr_resonance_K.{pdf,png}
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
C_PANELS = [40, 60, 100]
POL = ["round-robin-sw", "pf-classic-sw", "mr-mw-ch-sw"]
LABEL  = {"round-robin-sw": "RR", "pf-classic-sw": "PF", "mr-mw-ch-sw": "Risk-Aware"}
COLORS = {"round-robin-sw": "#F9A825", "pf-classic-sw": "#E53935", "mr-mw-ch-sw": "#8E24AA"}
MARKERS = {"round-robin-sw": "o", "pf-classic-sw": "s", "mr-mw-ch-sw": "*"}

def maxbar(K, C, pol):
    fs = sorted(glob.glob(os.path.join(DATA, f"K{K}", f"C{C:02d}", SCEN, f"{pol}_*_drone_summary.csv")))
    return pd.read_csv(fs[0]).mean_r_tot.max() if fs else np.nan

plt.rcParams.update({"font.family": "serif", "font.serif": ["Times New Roman", "Nimbus Roman", "DejaVu Serif"],
                     "mathtext.fontset": "stix", "font.size": 9, "pdf.fonttype": 42})
fig, axes = plt.subplots(1, len(C_PANELS), figsize=(9.0, 3.0), sharey=True,
                         constrained_layout=True, squeeze=False)
for ci, C in enumerate(C_PANELS):
    ax = axes[0][ci]
    for p in POL:
        y = [maxbar(K, C, p) for K in K_VALUES]
        ax.plot(K_VALUES, y, color=COLORS[p], marker=MARKERS[p], ms=8, lw=2.0, label=LABEL[p])
    # highlight RR's K=16 non-monotonic spike
    rr16 = maxbar(16, C, "round-robin-sw")
    ax.annotate("RR spike\n(K=16)", xy=(16, rr16), xytext=(6, rr16 * 2.2),
                fontsize=8, color=COLORS["round-robin-sw"],
                arrowprops=dict(arrowstyle="->", color=COLORS["round-robin-sw"], lw=1.2))
    ax.set_xscale("log", base=2); ax.set_yscale("log")
    ax.set_xticks(K_VALUES); ax.xaxis.set_major_formatter(mticker.ScalarFormatter()); ax.minorticks_off()
    ax.set_title(f"C = {C}", fontweight="bold")
    ax.set_xlabel(r"Slots per frame  $K$")
    if ci == 0:
        ax.set_ylabel(r"$\max_u \bar R_u$")
    ax.grid(True, which="both", ls=":", lw=0.4, alpha=0.6)

handles = [Line2D([0], [0], color=COLORS[p], marker=MARKERS[p], lw=2, label=LABEL[p]) for p in POL]
fig.legend(handles=handles, ncol=3, loc="lower center", bbox_to_anchor=(0.5, -0.06), frameon=False)

OUT = os.path.join(DATA, "plots")
base = os.path.join(OUT, "fig_rr_resonance_K")
for ext in ("pdf", "png"):
    fig.savefig(f"{base}.{ext}", dpi=150, bbox_inches="tight")
    print("saved", f"{base}.{ext}")
