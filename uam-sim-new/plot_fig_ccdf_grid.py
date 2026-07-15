#!/usr/bin/env python3
"""fig_ccdf_grid.pdf -- double-column CCDF grid over ALL (K,N) tuples.
6 rows (K) x 8 cols (N); each panel = CCDF P(R_u(t) > x) pooled over (u,t) for
RR / PF / Risk-Aware, log-log, common axes, vertical dashed at x=1 (design envelope).

Provenance: csv_export_RC7/K{K}/C{N:02d}/4r_b7/<policy>_*_slot_series.csv, column r_tot.
Output: csv_export_RC7/plots/fig_ccdf_grid.{pdf,png}
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

def pooled(K, C, pol):
    fs = sorted(glob.glob(os.path.join(DATA, f"K{K}", f"C{C:02d}", SCEN, f"{pol}_*_slot_series.csv")))
    if not fs:
        return None
    return pd.read_csv(fs[0], usecols=["r_tot"]).r_tot.values.astype(np.float32)

def ccdf_xy(r, npts=700):
    xs = np.sort(r[r > 0]); n = xs.size
    if n == 0:
        return np.array([]), np.array([])
    p = (n - np.arange(n)) / n
    idx = np.unique(np.linspace(0, n - 1, npts).astype(int))
    return xs[idx], p[idx]

# preload + global x-range
series = {}
xmax = 1.0
for K in K_VALUES:
    for C in N_VALUES:
        for p in POL:
            r = pooled(K, C, p)
            series[(K, C, p)] = r
            if r is not None and r.size:
                xmax = max(xmax, float(r.max()))

plt.rcParams.update({
    "font.family": "serif", "font.serif": ["Times New Roman", "Nimbus Roman", "DejaVu Serif"],
    "mathtext.fontset": "stix", "font.size": 7.5, "axes.titlesize": 8,
    "xtick.labelsize": 6, "ytick.labelsize": 6, "pdf.fonttype": 42,
})
nrows, ncols = len(K_VALUES), len(N_VALUES)
fig, axes = plt.subplots(nrows, ncols, figsize=(7.16, 8.4),
                         sharex=True, sharey=True, constrained_layout=True, squeeze=False)

for ri, K in enumerate(K_VALUES):
    for ci, C in enumerate(N_VALUES):
        ax = axes[ri][ci]
        for p in POL:
            r = series[(K, C, p)]
            if r is None or r.size == 0:
                continue
            x, y = ccdf_xy(r)
            ax.plot(x, y, color=COLORS[p], lw=1.1)
        ax.axvline(1.0, ls=(0, (3, 2)), lw=0.7, color="0.45")
        ax.set_xscale("log"); ax.set_yscale("log")
        ax.set_xlim(0.1, xmax * 1.4)
        ax.set_ylim(8e-7, 1.6)
        ax.grid(True, which="major", ls=":", lw=0.3, alpha=0.5)
        if ri == 0:
            ax.set_title(f"N = {C}", fontsize=8, fontweight="bold")
        if ci == 0:
            ax.set_ylabel(f"K = {K}", fontsize=8, fontweight="bold")
        ax.xaxis.set_major_locator(mticker.LogLocator(numticks=4))
        ax.yaxis.set_major_locator(mticker.LogLocator(numticks=4))

# shared axis labels
fig.supxlabel(r"Per-UAV risk $R_u(t)$   (dashed line = design envelope, $x=1$)", fontsize=9)
fig.supylabel(r"$P(R_u > x)$", fontsize=9)

handles = [Line2D([0], [0], color=COLORS[p], lw=2, label=LABEL[p]) for p in POL]
handles.append(Line2D([0], [0], color="0.45", ls=(0, (3, 2)), lw=0.9, label="design envelope"))
fig.legend(handles=handles, ncol=len(handles), loc="upper center",
           bbox_to_anchor=(0.5, 1.03), frameon=False, fontsize=9)

OUT = os.path.join(DATA, "plots")
base = os.path.join(OUT, "fig_ccdf_grid")
for ext in ("pdf", "png"):
    fig.savefig(f"{base}.{ext}", dpi=150, bbox_inches="tight")
    print("saved", f"{base}.{ext}")
