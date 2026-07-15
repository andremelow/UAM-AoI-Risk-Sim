#!/usr/bin/env python3
"""plot_fig_risk_components.py -- component-wise policy evaluation, NO summing.

R = w_unc*R_nav + w_map*R_gnd + w_vid*R_vid is the SCHEDULING index; for
evaluation each component is measured separately in its own normalized scale
(paper eq. 10-11 design points -- R_bar_nav, R_bar_gnd, H_nom), so "= 1" is a
per-component design envelope with its own physical reading:
  R_nav : pairwise uncertainty-disk overlap (mid-air conflict pressure)
  R_gnd : people expected inside the uncertainty disk (ground exposure)
  R_vid : video staleness beyond coherence window (DAA availability)

Metric semantics identical to the TLS target: per-UAV flight mean of each raw
component (all UAVs, whole flight, from slot_series), then fleet mean (top
row) and p99 across UAVs (bottom row). 2x3 grid vs N at fixed K.

Usage:
  python3 plot_fig_risk_components.py --dir csv_export_RC14 --K 8
Output: <dir>/plots/fig_risk_components_K<K>.{pdf,png} + printed table.
"""
import os, glob, argparse
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

ap = argparse.ArgumentParser()
ap.add_argument("--dir", default="csv_export_RC14")
ap.add_argument("--scenario", default="4r_b7")
ap.add_argument("--K", type=int, default=8)
ap.add_argument("--q", type=float, default=0.99)
ap.add_argument("--N-list", default="")
ap.add_argument("--pol", default="round-robin-sw,pf-classic-sw,mw-sw-v8,mw-sw-v9")
ap.add_argument("--table-N", type=int, default=60, help="cell for printed table")
args = ap.parse_args()

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(SCRIPT_DIR, args.dir)
POL = [p for p in args.pol.split(",") if p]
LABEL  = {"round-robin-sw": "RR", "pf-classic-sw": "PF",
          "mw-sw-v8": "MW-V8", "mw-sw-v9": "MW-V9",
          "mw-z": "MW-Z", "mw-z-v9": "MW-Z-V9"}
COLORS = {"round-robin-sw": "#F9A825", "pf-classic-sw": "#E53935",
          "mw-sw-v8": "#00897B", "mw-sw-v9": "#7CB342",
          "mw-z": "#1E88E5", "mw-z-v9": "#6A1B9A"}
MARKERS = {"round-robin-sw": "o", "pf-classic-sw": "s", "mw-sw-v8": "X",
           "mw-sw-v9": "^", "mw-z": "D", "mw-z-v9": "P"}
COMP = [("r_unc", r"$\tilde R_{\mathrm{Nav}}$ (disk overlap)"),
        ("r_map", r"$\tilde R_{\mathrm{Ground}}$ (people in disk)"),
        ("r_vid", r"$\tilde R_{\mathrm{Video}}$ (staleness)")]

if args.N_list:
    N_VALUES = [int(x) for x in args.N_list.split(",")]
else:
    N_VALUES = sorted(int(d[1:]) for d in os.listdir(os.path.join(DATA, f"K{args.K}"))
                      if d.startswith("C"))

def per_uav_means(pol, C):
    """Per-UAV whole-flight mean of each raw component (same semantics as
    drone_summary.mean_r_tot, but component-wise and weight-free)."""
    base = os.path.join(DATA, f"K{args.K}", f"C{C:02d}", args.scenario)
    fs = sorted(glob.glob(os.path.join(base, f"{pol}_*_slot_series.csv")))
    if not fs:
        return None
    ss = pd.read_csv(fs[-1], usecols=["drone_id", "r_unc", "r_map", "r_vid"])
    return ss.groupby("drone_id")[["r_unc", "r_map", "r_vid"]].mean()

stats = {p: {c: {"mean": [], "q": []} for c, _ in COMP} for p in POL}
table = {}
for p in POL:
    for C in N_VALUES:
        gu = per_uav_means(p, C)
        for c, _ in COMP:
            if gu is None:
                stats[p][c]["mean"].append(np.nan)
                stats[p][c]["q"].append(np.nan)
            else:
                stats[p][c]["mean"].append(float(gu[c].mean()))
                stats[p][c]["q"].append(float(gu[c].quantile(args.q)))
        if C == args.table_N and gu is not None:
            table[p] = gu
    print(f"done {LABEL.get(p, p)}")

plt.rcParams.update({"font.family": "serif",
                     "font.serif": ["Times New Roman", "Nimbus Roman", "DejaVu Serif"],
                     "mathtext.fontset": "stix", "font.size": 9, "pdf.fonttype": 42})
fig, axes = plt.subplots(2, 3, figsize=(8.6, 5.0), sharex=True,
                         constrained_layout=True)
for cc, (c, title) in enumerate(COMP):
    axm, axq = axes[0, cc], axes[1, cc]
    for p in POL:
        axm.plot(N_VALUES, stats[p][c]["mean"], color=COLORS[p],
                 marker=MARKERS[p], ms=4.5, lw=1.4, label=LABEL.get(p, p))
        axq.plot(N_VALUES, stats[p][c]["q"], color=COLORS[p],
                 marker=MARKERS[p], ms=4.5, lw=1.4)
    for ax in (axm, axq):
        ax.set_xscale("log")
        ax.set_yscale("log")
        ax.set_xticks(N_VALUES)
        ax.set_xticklabels([str(n) for n in N_VALUES], fontsize=6.5)
        ax.tick_params(axis="x", which="minor", bottom=False)
        ax.axhline(1.0, ls=(0, (4, 3)), lw=0.9, color="0.35")
        ax.grid(True, which="both", ls=":", lw=0.35, alpha=0.55)
    axm.set_title(title, fontsize=9.5)
    axq.set_xlabel("Simultaneous UAVs $N$", fontsize=8.5)
axes[0, 0].set_ylabel(r"Fleet mean of per-UAV $\bar R_{c,u}$")
axes[1, 0].set_ylabel(rf"$p_{{{int(100*args.q)}}}$ over UAVs of $\bar R_{{c,u}}$")
axes[0, 2].text(N_VALUES[-1], 1.0, " component design point",
                fontsize=6, color="0.35", va="bottom", ha="right")
handles = [Line2D([0], [0], color=COLORS[p], marker=MARKERS[p], lw=1.6, ms=6,
                  label=LABEL.get(p, p)) for p in POL]
handles.append(Line2D([0], [0], ls=(0, (4, 3)), color="0.35", lw=1,
                      label="component envelope (=1)"))
fig.suptitle(f"Component-wise evaluation (no weighting) -- K = {args.K}",
             fontsize=11, fontweight="bold")
fig.legend(handles=handles, ncol=len(handles), loc="lower center",
           bbox_to_anchor=(0.5, -0.06), frameon=False, fontsize=8.5)

OUT = os.path.join(DATA, "plots")
os.makedirs(OUT, exist_ok=True)
base = os.path.join(OUT, f"fig_risk_components_K{args.K}")
for ext in ("pdf", "png"):
    fig.savefig(f"{base}.{ext}", dpi=200, bbox_inches="tight")
    print("saved", f"{base}.{ext}")

QP = int(100 * args.q)
print(f"\nComponent table -- K={args.K}, N={args.table_N} "
      f"(per-UAV flight mean; fleet mean | p{QP} over UAVs):")
print(f"{'policy':9}" + "".join(f" {c:>21}" for c, _ in COMP))
for p in POL:
    if p not in table:
        continue
    gu = table[p]
    row = f"{LABEL.get(p, p):9}"
    for c, _ in COMP:
        row += f"  {gu[c].mean():8.3f} | {gu[c].quantile(args.q):8.3f}"
    print(row)
