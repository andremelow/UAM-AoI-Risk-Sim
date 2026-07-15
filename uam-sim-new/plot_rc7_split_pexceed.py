#!/usr/bin/env python3
"""
plot_rc7_split_pexceed.py -- split figure where the top row is the PROBABILITY
that a drone is unsafe, P(R̄_u > 1) = fraction of the fleet whose per-UAV
time-average risk exceeds the normalized acceptance threshold 1; the bottom row
keeps the peak system risk (as in risk_top_avg --split).

R̄_u = drone_summary.mean_r_tot (per-UAV time mean of the phase-aware normalized
risk r_tot, matching the tls_companion flight-window definition). One panel per K.

Usage:
  python3 plot_rc7_split_pexceed.py --dir csv_export_RC7
"""
import os, glob, argparse
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

ap = argparse.ArgumentParser()
ap.add_argument("--dir", default="csv_export_RC7")
ap.add_argument("--scenario", default="4r_b7")
ap.add_argument("--thr", type=float, default=1.0, help="per-UAV risk threshold (default 1)")
ap.add_argument("--logy-peak", action="store_true", default=True)
ap.add_argument("--pol", default="round-robin-sw,pf-classic-sw,mr-mw-ch-sw,max-risk-pf-sw",
                help="comma-separated policy list (e.g. add mw-sw-v8 after merging RC9)")
args = ap.parse_args()

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR   = os.path.join(SCRIPT_DIR, args.dir)
OUT_DIR    = os.path.join(DATA_DIR, "plots")
os.makedirs(OUT_DIR, exist_ok=True)

K_VALUES = [1, 2, 4, 8, 16, 32]
C_VALUES = [5, 10, 20, 40, 60, 80, 100, 120]
POL       = [p for p in args.pol.split(",") if p]
POL_LABEL = {"round-robin-sw": "RR", "pf-classic-sw": "PF", "mr-mw-ch-sw": "Risk-Aware",
             "max-risk-pf-sw": "Risk-PF", "mw-sw-otimo-ch": "MW-opt (sw)",
             "mw-otimo-ch": "MW-opt", "mw-sw-v8": "MW-V8", "mw-sw-v9": "MW-V9", "mw-v9-nsw": "MW-V9-nsw"}
COLORS    = {"round-robin-sw": "#F9A825", "pf-classic-sw": "#E53935", "mr-mw-ch-sw": "#8E24AA",
             "max-risk-pf-sw": "#1E88E5", "mw-sw-otimo-ch": "#3949AB",
             "mw-otimo-ch": "#6D4C41", "mw-sw-v8": "#00897B", "mw-sw-v9": "#7CB342", "mw-v9-nsw": "#33691E"}
MARKERS   = {"round-robin-sw": "o", "pf-classic-sw": "s", "mr-mw-ch-sw": "*",
             "max-risk-pf-sw": "D", "mw-sw-otimo-ch": "v", "mw-otimo-ch": "P", "mw-sw-v9": "^", "mw-v9-nsw": "v",
             "mw-sw-v8": "X"}

def _first(pat):
    fs = sorted(glob.glob(pat))
    return fs[0] if fs else None

def p_unsafe(K, C, pol):
    """Fraction of UAVs with R̄_u = mean_r_tot > thr, in [0,1]. NaN if missing."""
    ds = _first(os.path.join(DATA_DIR, f"K{K}", f"C{C:02d}", args.scenario,
                             f"{pol}_*_drone_summary.csv"))
    if ds is None:
        return np.nan
    m = pd.read_csv(ds, usecols=["mean_r_tot"])["mean_r_tot"].values
    return float((m > args.thr).mean())

# peak system risk from the master summary
summ = pd.read_csv(os.path.join(DATA_DIR, "sweep_summary_all.csv"))
summ = summ[summ["scenario"] == args.scenario]

fig, axes = plt.subplots(2, len(K_VALUES), figsize=(3.0*len(K_VALUES), 6.6),
                         constrained_layout=True, squeeze=False)

for ci, K in enumerate(K_VALUES):
    axP = axes[0][ci]
    axK = axes[1][ci]
    for pol in POL:
        # ---- top: P(drone R̄_u > 1) ----
        ys = np.array([p_unsafe(K, C, pol) for C in C_VALUES]) * 100.0
        axP.plot(C_VALUES, ys, color=COLORS[pol], marker=MARKERS[pol], ms=6, lw=2.0)
        # ---- bottom: peak system risk ----
        d = summ[(summ.K == K) & (summ.policy == pol)].sort_values("capacity")
        axK.plot(d.capacity, d.peak_r_sys.values.astype(float),
                 color=COLORS[pol], marker=MARKERS[pol], ms=5, lw=2.0, ls="--")
    axP.set_title(f"K = {K}", fontweight="bold")
    axP.set_ylim(-4, 104)
    axP.set_xticks(C_VALUES); axP.tick_params(axis="x", labelsize=8)
    axK.set_xticks(C_VALUES); axK.tick_params(axis="x", labelsize=8)
    axK.set_xlabel("Simultaneous UAVs")
    axK.set_yscale("log")
    if ci == 0:
        axP.set_ylabel(r"$P(\bar R_u > 1)$  [%]")
        axK.set_ylabel(r"Peak risk  $R_{sys}$")
    axP.grid(True, ls="--", alpha=0.4)
    axK.grid(True, which="both", ls="--", alpha=0.4)

handles = [Line2D([0], [0], color=COLORS[p], marker=MARKERS[p], lw=2, label=POL_LABEL[p]) for p in POL]
fig.legend(handles=handles, ncol=len(handles), loc="lower center",
           bbox_to_anchor=(0.5, -0.04), frameon=False, fontsize=10)

base = os.path.join(OUT_DIR, "risk_split_pexceed_logy")
for ext in ("pdf", "png"):
    fig.savefig(f"{base}.{ext}", dpi=150, bbox_inches="tight")
    print("Saved:", f"{base}.{ext}")

# console: P(drone>1) grid per policy
print("\n=== P(R̄_u > 1) [%] per (K, C) ===")
for pol in POL:
    print(f"\n{POL_LABEL[pol]}:")
    print("K\\C  " + "".join(f"{c:>7}" for c in C_VALUES))
    for K in K_VALUES:
        print(f"{K:<4} " + "".join(f"{100*p_unsafe(K,C,pol):7.1f}" for C in C_VALUES))
