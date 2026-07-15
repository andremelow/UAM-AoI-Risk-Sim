#!/usr/bin/env python3
"""plot_paper_cdf_grid.py -- inverted quantile-view CDF of the worst UAV's risk
R_{u*}(t) for ALL cells: grid of K rows x N columns, one panel per (K, N),
split-color palette, risk on y (log), P(R_{u*} <= x) on x.

Per cell and policy: u* = argmax_u temporal mean of R_u (t > burnin, UAVs with
n >= min-n); curve = empirical quantile function of the pooled post-burn-in
samples of R_{u*}(t). Horizontal envelope line at R = 1 in every panel.
Reuses the per-run uav_stats cache of the paper figures.

Usage:
  python3 plot_paper_cdf_grid.py --dir csv_export_RC11 --burnin 10000
Output: <dir>/plots/fig_cdf_Ru_grid.{pdf,png}
"""
import os, argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import paper_data_checks as pdc

ap = argparse.ArgumentParser()
ap.add_argument("--dir", default="csv_export_RC11")
ap.add_argument("--scenario", default="4r_b7")
ap.add_argument("--burnin", type=float, default=1e4)
ap.add_argument("--min-n", type=int, default=500)
ap.add_argument("--cells", default=None,
                help="comma list of K:N pairs (e.g. 1:5,2:10) — single-row layout "
                     "over the given diagonal instead of the full grid")
ap.add_argument("--std-axes", action="store_true",
                help="standard CDF orientation like fig_cdf_Ru_N100_K16: "
                     "x = risk (log), y = P(R_{u*} <= x); default is inverted")
args = ap.parse_args()

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(SCRIPT_DIR, args.dir)
OUT  = os.path.join(DATA, "plots")
CACHE = os.path.join(OUT, "paper_cache")
BURNIN = int(args.burnin)

K_VALUES = [1, 2, 4, 8, 16, 32]
N_VALUES = [5, 10, 20, 40, 60, 80, 100, 120]
POL    = pdc.REQUIRED_POLICIES
LABEL  = {"round-robin-sw": "RR", "pf-classic-sw": "PF",
          "mw-sw-v9": "Risk-Aware", "mw-v9-nsw": "Risk-Aware (non-preempt.)"}
COLORS = {"round-robin-sw": "#F9A825", "pf-classic-sw": "#E53935",
          "mw-sw-v9": "#7CB342", "mw-v9-nsw": "#33691E"}

def ustar_curve(K, N, pol):
    runs = pdc.discover_runs(DATA, args.scenario, K, N, pol)
    if not runs:
        return None
    run = runs[-1]                                  # latest (deduped by content anyway)
    st = pdc.uav_stats(run, BURNIN, CACHE)
    st = st[st.n >= args.min_n]
    if not len(st):
        return None
    u_star = int(st.loc[st.mean_r.idxmax(), "drone_id"])
    r = pdc.uav_series(run, u_star, BURNIN)
    xs = np.sort(r[r > 0])
    if not xs.size:
        return None
    p = (np.arange(xs.size) + 1) / xs.size          # P(R <= x)
    idx = np.unique(np.linspace(0, xs.size - 1, 400).astype(int))
    return p[idx], xs[idx]

plt.rcParams.update({"font.family": "serif",
                     "font.serif": ["Times New Roman", "Nimbus Roman", "DejaVu Serif"],
                     "mathtext.fontset": "stix", "font.size": 7,
                     "xtick.labelsize": 5.5, "ytick.labelsize": 5.5,
                     "pdf.fonttype": 42})
if args.cells:
    CELLS = [tuple(int(v) for v in c.split(":")) for c in args.cells.split(",")]
    fig, axes = plt.subplots(1, len(CELLS), figsize=(2.05*len(CELLS), 2.6),
                             sharex=True, sharey=True, constrained_layout=True,
                             squeeze=False)
    for ci, (K, N) in enumerate(CELLS):
        ax = axes[0][ci]
        for pol in POL:
            c = ustar_curve(K, N, pol)
            if c is not None:
                if args.std_axes:
                    ax.plot(c[1], c[0], color=COLORS[pol], lw=1.6)
                else:
                    ax.plot(c[0], c[1], color=COLORS[pol], lw=1.6)
        if args.std_axes:
            ax.set_xscale("log")
            ax.axvline(1.0, ls=(0, (4, 3)), lw=0.8, color="0.35")
            if ci == len(CELLS) - 1:
                ax.text(1.0, 0.05, " TLS", transform=ax.get_xaxis_transform(),
                        ha="left", va="bottom", fontsize=6.5, color="0.35")
            ax.set_ylim(0, 1.02)
            ax.set_xlabel(r"$x$ (risk $R_{u^*}$)", fontsize=7)
            if ci == 0:
                ax.set_ylabel(r"$P(R_{u^*}(t) \leq x)$")
        else:
            ax.set_yscale("log")
            ax.axhline(1.0, ls=(0, (4, 3)), lw=0.8, color="0.35")
            ax.set_xlim(0, 1.02)
            ax.set_xlabel(r"$P(R_{u^*} \leq x)$", fontsize=7)
            if ci == 0:
                ax.set_ylabel("$x$ (risk)")
        ax.set_title(f"K = {K},  N = {N}", fontsize=8, fontweight="bold")
        ax.grid(True, which="both", ls=":", lw=0.4, alpha=0.6)
        print(f"done K={K} N={N}", flush=True)
    axes = None
else:
    fig, axes = plt.subplots(len(K_VALUES), len(N_VALUES),
                             figsize=(1.55*len(N_VALUES), 1.35*len(K_VALUES)),
                             sharex=True, sharey=True, constrained_layout=True)

    for ki, K in enumerate(K_VALUES):
        for ni, N in enumerate(N_VALUES):
            ax = axes[ki][ni]
            for pol in POL:
                c = ustar_curve(K, N, pol)
                if c is not None:
                    ax.plot(c[0], c[1], color=COLORS[pol], lw=1.1)
            ax.set_yscale("log")
            ax.axhline(1.0, ls=(0, (4, 3)), lw=0.6, color="0.35")
            ax.set_xlim(0, 1.02)
            ax.grid(True, which="both", ls=":", lw=0.3, alpha=0.5)
            if ki == 0:
                ax.set_title(f"N = {N}", fontsize=7, fontweight="bold")
            if ni == 0:
                ax.set_ylabel(f"K = {K}\n$x$ (risk)", fontsize=6.5)
            if ki == len(K_VALUES) - 1:
                ax.set_xlabel(r"$P(R_{u^*} \leq x)$", fontsize=6.5)
        print(f"done K={K}", flush=True)

handles = [Line2D([0], [0], color=COLORS[p], lw=1.8, label=LABEL[p]) for p in POL]
fig.legend(handles=handles, ncol=4, loc="lower center",
           bbox_to_anchor=(0.5, -0.025), frameon=False, fontsize=8)

os.makedirs(OUT, exist_ok=True)
base = os.path.join(OUT, "fig_cdf_Ru_grid" if not args.cells else
                    "fig_cdf_Ru_diag" + ("_std" if args.std_axes else ""))
for ext in ("pdf", "png"):
    fig.savefig(f"{base}.{ext}", dpi=160, bbox_inches="tight")
    print("saved", f"{base}.{ext}")
