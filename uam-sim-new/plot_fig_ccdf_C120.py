#!/usr/bin/env python3
"""fig_ccdf_C120.pdf -- double-column CCDF at C=120, one panel per K.
CCDF P(R_u(t) > x) pooled over (u,t) for RR / PF / Risk-Aware, log-log, common
axes, vertical dashed at x=1 (design envelope). Shared fig_style (grayscale).

Provenance: csv_export_RC7/K{K}/C120/4r_b7/<policy>_*_slot_series.csv, column r_tot.
Output: csv_export_RC7/plots/fig_ccdf_C120.{pdf,png}
"""
import os, glob
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
from matplotlib.lines import Line2D
import fig_style

import sys
DATA, SCEN = "csv_export_RC7", "4r_b7"
C = int(sys.argv[1]) if len(sys.argv) > 1 else 120
K_VALUES = [1, 2, 4, 8, 16, 32]
POL = fig_style.POLICIES

def pooled(K, pol):
    fs = sorted(glob.glob(os.path.join(DATA, f"K{K}", f"C{C:02d}", SCEN, f"{pol}_*_slot_series.csv")))
    return pd.read_csv(fs[0], usecols=["r_tot"]).r_tot.values.astype(np.float32) if fs else None

def ccdf_xy(r, npts=1500):
    xs = np.sort(r[r > 0]); n = xs.size
    if n == 0:
        return np.array([]), np.array([])
    p = (n - np.arange(n)) / n
    idx = np.unique(np.linspace(0, n - 1, npts).astype(int))
    return xs[idx], p[idx]

COLORS = {"round-robin-sw": "#F9A825", "pf-classic-sw": "#E53935", "mr-mw-ch-sw": "#8E24AA"}
MARKERS = {"round-robin-sw": "o", "pf-classic-sw": "s", "mr-mw-ch-sw": "*"}

series = {(K, p): pooled(K, p) for K in K_VALUES for p in POL}
xmax = max(v.max() for v in series.values() if v is not None and v.size)

fig_style.apply()
fig, axes = plt.subplots(1, 6, figsize=(7.16, 2.3), sharex=True, sharey=True,
                         constrained_layout=True, squeeze=False)
axes = axes.flatten()
for ki, K in enumerate(K_VALUES):
    ax = axes[ki]
    for p in POL:
        r = series[(K, p)]
        if r is None or r.size == 0:
            continue
        x, y = ccdf_xy(r)
        ax.plot(x, y, color=COLORS[p], marker=MARKERS[p], markevery=0.16, ms=3.0,
                lw=1.4, markerfacecolor=COLORS[p], label=fig_style.LABEL[p])
    ax.axvline(1.0, ls=(0, (3, 2)), lw=0.8, color="0.45")
    ax.set_xscale("log"); ax.set_yscale("log")
    ax.set_xlim(0.1, xmax * 1.4); ax.set_ylim(8e-7, 1.6)
    ax.set_title(f"K = {K}", fontweight="bold")
    ax.grid(True, which="major", ls=":", lw=0.35, alpha=0.55)
    ax.set_xlabel(r"$R_u(t)$")
    if ki == 0:
        ax.set_ylabel(r"$P(R_u > x)$")
    ax.xaxis.set_major_locator(mticker.LogLocator(numticks=4))
    ax.tick_params(labelsize=6)

handles = [Line2D([0], [0], color=COLORS[p], marker=MARKERS[p], lw=1.8, label=fig_style.LABEL[p]) for p in POL]
handles.append(Line2D([0], [0], color="0.45", ls=(0, (3, 2)), lw=0.9, label="design envelope ($x=1$)"))
fig.legend(handles=handles, ncol=len(handles), loc="lower center",
           bbox_to_anchor=(0.5, -0.16), frameon=False, fontsize=8)
fig.suptitle(f"CCDF of per-UAV risk at C = {C} (pooled over u,t)", fontsize=10, fontweight="bold")

OUT = os.path.join(DATA, "plots")
base = os.path.join(OUT, f"fig_ccdf_C{C}")
for ext in ("pdf", "png"):
    fig.savefig(f"{base}.{ext}", dpi=150, bbox_inches="tight")
    print("saved", f"{base}.{ext}")
