#!/usr/bin/env python3
"""plot_risk_boxplot.py -- boxplot of the per-UAV time-average risk R̄_u
(drone_summary.mean_r_tot), one box per policy, one panel per K, fixed C.

Box = fleet distribution of R̄_u in that cell: median, IQR, whiskers (1.5 IQR),
outliers = the worst UAVs the TLS frontier is driven by. TLS line at 1.

Usage:
  python3 plot_risk_boxplot.py --dir csv_export_RC11 --C 80 \
      --pol round-robin-sw,pf-classic-sw,mw-sw-v9,mw-v9-nsw
Output: <dir>/plots/fig_risk_boxplot_C{C}.{pdf,png}
"""
import os, glob, argparse
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

ap = argparse.ArgumentParser()
ap.add_argument("--dir", default="csv_export_RC11")
ap.add_argument("--scenario", default="4r_b7")
ap.add_argument("--C", type=int, default=80)
ap.add_argument("--pol", default="round-robin-sw,pf-classic-sw,mw-sw-v9,mw-v9-nsw")
ap.add_argument("--violin", action="store_true",
                help="violin plot (log-space KDE) instead of boxplot")
ap.add_argument("--grid", action="store_true",
                help="full 6x8 grid: rows = K, cols = N, one mini-boxplot panel per cell")
ap.add_argument("--cells", default=None,
                help="comma list of K:C pairs (e.g. 1:5,2:10,4:20) — panels follow "
                     "the given diagonal instead of all K at fixed --C")
ap.add_argument("--route", choices=["all", "A", "B"], default="all",
                help="restrict fleet to corridor A (8 km) or B (4.3 km, hotspots); "
                     "classified by flight duration (exit-entry vs 1.5x min)")
args = ap.parse_args()

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(SCRIPT_DIR, args.dir)
SCEN, C = args.scenario, args.C
K_VALUES = [1, 2, 4, 8, 16, 32]

POL    = [p for p in args.pol.split(",") if p]
LABEL  = {"round-robin-sw": "RR", "pf-classic-sw": "PF", "mr-mw-ch-sw": "Risk-Aware",
          "max-risk-pf-sw": "Risk-PF", "mw-sw-otimo-ch": "MW-opt (sw)",
          "mw-otimo-ch": "MW-opt", "mw-sw-v8": "MW-V8", "mw-sw-v9": "MW-V9",
          "mw-v9-nsw": "MW-V9-nsw"}
COLORS = {"round-robin-sw": "#F9A825", "pf-classic-sw": "#E53935", "mr-mw-ch-sw": "#8E24AA",
          "max-risk-pf-sw": "#1E88E5", "mw-sw-otimo-ch": "#3949AB",
          "mw-otimo-ch": "#6D4C41", "mw-sw-v8": "#00897B", "mw-sw-v9": "#7CB342",
          "mw-v9-nsw": "#33691E"}

def rbar(K, pol, C=None):
    C = C if C is not None else args.C
    fs = sorted(glob.glob(os.path.join(DATA, f"K{K}", f"C{C:02d}", SCEN,
                                        f"{pol}_*_drone_summary.csv")))
    if not fs:
        return None
    d = pd.read_csv(fs[-1], usecols=["entry_slot", "exit_slot", "mean_r_tot"])
    if args.route != "all":
        dur = d.exit_slot - d.entry_slot
        is_A = dur > dur.min() * 1.5          # A (8 km) ~2x longer than B (4.3 km)
        d = d[is_A] if args.route == "A" else d[~is_A]
    return d.mean_r_tot.values

plt.rcParams.update({"font.family": "serif",
                     "font.serif": ["Times New Roman", "Nimbus Roman", "DejaVu Serif"],
                     "mathtext.fontset": "stix", "font.size": 8,
                     "xtick.labelsize": 7.5, "ytick.labelsize": 7, "pdf.fonttype": 42})
C_VALUES = [5, 10, 20, 40, 60, 80, 100, 120]
if args.grid:
    CELLS = [(K, Cc) for K in K_VALUES for Cc in C_VALUES]
    fig, axes = plt.subplots(len(K_VALUES), len(C_VALUES),
                             figsize=(1.55*len(C_VALUES), 1.5*len(K_VALUES)),
                             sharey=True, constrained_layout=True, squeeze=False)
    axes = axes.flatten()
elif args.cells:
    CELLS = [tuple(int(v) for v in c.split(":")) for c in args.cells.split(",")]
    fig, axes = plt.subplots(1, len(CELLS), figsize=(2.0*len(CELLS), 3.0),
                             sharey=True, constrained_layout=True, squeeze=False)
    axes = axes.flatten()
else:
    CELLS = [(K, args.C) for K in K_VALUES]
    fig, axes = plt.subplots(1, len(CELLS), figsize=(2.0*len(CELLS), 3.0),
                             sharey=True, constrained_layout=True, squeeze=False)
    axes = axes.flatten()

for ki, (K, Cc) in enumerate(CELLS):
    ax = axes[ki]
    data, colors = [], []
    for p in POL:
        v = rbar(K, p, Cc)
        data.append(v if v is not None else [])
        colors.append(COLORS[p])
    if args.violin:
        # KDE in log10 space (heavy-tailed data); ticks relabelled as 10^k.
        # Floor at 1e-3: route-A UAVs with R̄~0 would otherwise stretch the axis.
        dlog = [np.log10(np.clip(np.asarray(v, float), 1e-3, None)) for v in data]
        vp = ax.violinplot(dlog, showmedians=True, showextrema=False, widths=0.8)
        for body, c in zip(vp["bodies"], colors):
            body.set_facecolor(c); body.set_alpha(0.75); body.set_edgecolor("0.25")
        vp["cmedians"].set_color("black"); vp["cmedians"].set_linewidth(1.2)
        ax.axhline(0.0, ls=(0, (4, 3)), lw=0.9, color="0.35")   # log10(TLS)=0
        ax.set_xticks(np.arange(1, len(POL) + 1))
    else:
        bp = ax.boxplot(data, patch_artist=True, widths=0.62, showfliers=True,
                        flierprops=dict(marker=".", ms=3, alpha=0.55),
                        medianprops=dict(color="black", lw=1.3),
                        whiskerprops=dict(lw=0.9), capprops=dict(lw=0.9))
        for patch, c in zip(bp["boxes"], colors):
            patch.set_facecolor(c); patch.set_alpha(0.75); patch.set_edgecolor("0.25")
        ax.axhline(1.0, ls=(0, (4, 3)), lw=0.9, color="0.35")
    if ki == len(K_VALUES) - 1:
        ax.text(0.97, 0.0 if args.violin else 1.0, " TLS",
                transform=ax.get_yaxis_transform(),
                ha="left", va="bottom", fontsize=7, color="0.35", clip_on=False)
    if args.violin:
        yt = np.arange(-3, 3)
        ax.set_yticks(yt)
        ax.set_yticklabels([r"$\leq 10^{-3}$" if k == -3 else f"$10^{{{int(k)}}}$"
                            for k in yt])
        ax.set_ylim(-3.3, 2.2)
    else:
        ax.set_yscale("log")
    ax.set_title(f"K = {K},  N = {Cc}" if args.cells else f"K = {K}", fontweight="bold")
    ax.set_xticklabels([LABEL.get(p, p) for p in POL], rotation=45, ha="right")
    if ki == 0:
        ax.set_ylabel(r"Per-UAV risk  $\bar R_u$")
    ax.grid(True, axis="y", which="both", ls=":", lw=0.4, alpha=0.6)

if args.grid:
    handles = [Patch(facecolor=COLORS[p], edgecolor="0.25", label=LABEL.get(p, p)) for p in POL]
    fig.legend(handles=handles, ncol=len(POL), loc="lower center",
               bbox_to_anchor=(0.5, -0.02), frameon=False, fontsize=9)
RTAG = "" if args.route == "all" else f" — Route {args.route} only"
HDR = ("all K x N cells" if args.grid else
       "diagonal N/K constant" if args.cells else f"C = {C} (steady/N=C)")
fig.suptitle(f"Fleet distribution of per-UAV risk — {HDR}{RTAG}",
             fontsize=9, fontweight="bold")

OUT = os.path.join(DATA, "plots")
os.makedirs(OUT, exist_ok=True)
VTAG = "violin" if args.violin else "boxplot"
CTAG = ("grid" if args.grid
        else "_".join(f"K{k}C{c}" for k, c in CELLS) if args.cells and len(CELLS) <= 3
        else "diag" if args.cells else f"C{C}")
base = os.path.join(OUT, f"fig_risk_{VTAG}_{CTAG}"
                    + ("" if args.route == "all" else f"_route{args.route}"))
for ext in ("pdf", "png"):
    fig.savefig(f"{base}.{ext}", dpi=170, bbox_inches="tight")
    print("saved", f"{base}.{ext}")
