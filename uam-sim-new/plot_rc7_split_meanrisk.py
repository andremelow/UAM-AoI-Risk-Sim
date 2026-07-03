#!/usr/bin/env python3
"""plot_rc7_split_meanrisk.py -- split figure of MEAN risk: the top row is the
fleet-mean per-UAV time-average risk, mean_u R̄_u (R̄_u = drone_summary.mean_r_tot,
same flight-window definition as tls_companion/split_pexceed); the bottom row is
the MEAN system risk mean_r_sys from the master summary (steady window), replacing
the peak of split_pexceed. One panel per K.

Usage:
  python3 plot_rc7_split_meanrisk.py --dir csv_export_RC8v8 \
      --pol round-robin-sw,pf-classic-sw,mr-mw-ch-sw,mw-sw-v8
"""
import os, glob, argparse
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

ap = argparse.ArgumentParser()
ap.add_argument("--dir", default="csv_export_RC8v8")
ap.add_argument("--scenario", default="4r_b7")
ap.add_argument("--pol", default="round-robin-sw,pf-classic-sw,mr-mw-ch-sw,mw-sw-v8",
                help="comma-separated policy list")
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
             "mw-otimo-ch": "MW-opt", "mw-sw-v8": "MW-V8"}
COLORS    = {"round-robin-sw": "#F9A825", "pf-classic-sw": "#E53935", "mr-mw-ch-sw": "#8E24AA",
             "max-risk-pf-sw": "#1E88E5", "mw-sw-otimo-ch": "#3949AB",
             "mw-otimo-ch": "#6D4C41", "mw-sw-v8": "#00897B"}
MARKERS   = {"round-robin-sw": "o", "pf-classic-sw": "s", "mr-mw-ch-sw": "*",
             "max-risk-pf-sw": "D", "mw-sw-otimo-ch": "v", "mw-otimo-ch": "P",
             "mw-sw-v8": "X"}

def _first(pat):
    fs = sorted(glob.glob(pat))
    return fs[-1] if fs else None      # latest run per cell

def mean_rbar(K, C, pol):
    """Fleet-mean of R̄_u = mean over UAVs of drone_summary.mean_r_tot. NaN if missing."""
    ds = _first(os.path.join(DATA_DIR, f"K{K}", f"C{C:02d}", args.scenario,
                             f"{pol}_*_drone_summary.csv"))
    if ds is None:
        return np.nan
    return float(pd.read_csv(ds, usecols=["mean_r_tot"])["mean_r_tot"].mean())

# mean system risk from the master summary
summ = pd.read_csv(os.path.join(DATA_DIR, "sweep_summary_all.csv"))
summ = summ[summ["scenario"] == args.scenario]

fig, axes = plt.subplots(2, len(K_VALUES), figsize=(3.0*len(K_VALUES), 6.6),
                         constrained_layout=True, squeeze=False)

for ci, K in enumerate(K_VALUES):
    axU = axes[0][ci]
    axS = axes[1][ci]
    for pol in POL:
        # ---- top: fleet-mean R̄_u ----
        ys = np.array([mean_rbar(K, C, pol) for C in C_VALUES])
        axU.plot(C_VALUES, ys, color=COLORS[pol], marker=MARKERS[pol], ms=6, lw=2.0)
        # ---- bottom: mean system risk ----
        d = summ[(summ.K == K) & (summ.policy == pol)].sort_values("capacity")
        axS.plot(d.capacity, d.mean_r_sys.values.astype(float),
                 color=COLORS[pol], marker=MARKERS[pol], ms=5, lw=2.0, ls="--")
    axU.set_title(f"K = {K}", fontweight="bold")
    axU.set_yscale("log")
    axU.axhline(1.0, ls=(0, (4, 3)), lw=0.9, color="0.35")
    axU.set_xticks(C_VALUES); axU.tick_params(axis="x", labelsize=8)
    axS.set_xticks(C_VALUES); axS.tick_params(axis="x", labelsize=8)
    axS.set_xlabel("Simultaneous UAVs")
    axS.set_yscale("log")
    if ci == 0:
        axU.set_ylabel(r"Fleet-mean risk  $\overline{R}_u$")
        axS.set_ylabel(r"Mean system risk  $\bar R_{sys}$")
    axU.grid(True, which="both", ls="--", alpha=0.4)
    axS.grid(True, which="both", ls="--", alpha=0.4)

handles = [Line2D([0], [0], color=COLORS[p], marker=MARKERS[p], lw=2, label=POL_LABEL[p]) for p in POL]
fig.legend(handles=handles, ncol=len(handles), loc="lower center",
           bbox_to_anchor=(0.5, -0.04), frameon=False, fontsize=10)

base = os.path.join(OUT_DIR, "risk_split_mean")
for ext in ("pdf", "png"):
    fig.savefig(f"{base}.{ext}", dpi=150, bbox_inches="tight")
    print("Saved:", f"{base}.{ext}")

# console: fleet-mean R̄_u grid per policy
print("\n=== mean_u R̄_u per (K, C) ===")
for pol in POL:
    print(f"\n{POL_LABEL.get(pol, pol)}:")
    print("K\\C  " + "".join(f"{c:>8}" for c in C_VALUES))
    for K in K_VALUES:
        print(f"{K:<4} " + "".join(f"{mean_rbar(K,C,pol):8.2f}" for C in C_VALUES))
