#!/usr/bin/env python3
"""
plot_fig_impossibility_color.py -- "impossibility" figure (K=1 slice) in the SAME
color/marker convention as the split figures (split_pexceed / split_meanrisk):
  (a) fleet-mean time-average status AoI A(N)  [per-UAV time mean, then mean over u]
  (b) worst-case per-UAV risk max_u R̄_u        [per-UAV time mean of r_tot, then max over u]

Computed from RAW slot_series traces (same metric as plot_fig_impossibility.py).

Usage:
  python3 plot_fig_impossibility_color.py --dir csv_export_RC8v8 \
      --pol round-robin-sw,pf-classic-sw,mr-mw-ch-sw,mw-sw-v8
Output: <dir>/plots/fig_impossibility_K1_color.{pdf,png}
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
ap.add_argument("--pol", default="round-robin-sw,pf-classic-sw,mr-mw-ch-sw,mw-sw-v8",
                help="comma-separated policy list")
args = ap.parse_args()

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR   = os.path.join(SCRIPT_DIR, args.dir)
SCEN = args.scenario
K = 1
N_VALUES = [5, 10, 20, 40, 60, 80, 100, 120]

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

def _runs(C, pol):
    return sorted(glob.glob(os.path.join(DATA_DIR, f"K{K}", f"C{C:02d}", SCEN,
                                          f"{pol}_*_slot_series.csv")))

def cell(C, pol):
    """A(N), max_u R̄_u averaged over runs (runs are byte-identical here)."""
    fs = _runs(C, pol)
    if not fs:
        return np.nan, np.nan
    As, Rs = [], []
    for f in fs:
        df = pd.read_csv(f, usecols=["drone_id", "h1_slots", "r_tot"],
                         dtype={"drone_id": np.int32, "h1_slots": np.float32, "r_tot": np.float32})
        g = df.groupby("drone_id")
        As.append(g.h1_slots.mean().mean())     # per-UAV time-mean, then mean over u
        Rs.append(g.r_tot.mean().max())          # per-UAV time-mean, then MAX over u
    return float(np.mean(As)), float(np.mean(Rs))

A = {p: [] for p in POL}
R = {p: [] for p in POL}
for p in POL:
    for C in N_VALUES:
        a, r = cell(C, p)
        A[p].append(a); R[p].append(r)

plt.rcParams.update({"font.family": "serif",
                     "font.serif": ["Times New Roman", "Nimbus Roman", "DejaVu Serif"],
                     "mathtext.fontset": "stix", "font.size": 9, "pdf.fonttype": 42})
fig, (axa, axb) = plt.subplots(2, 1, figsize=(4.2, 5.2), sharex=True,
                               constrained_layout=True)

for p in POL:
    axa.plot(N_VALUES, A[p], color=COLORS[p], marker=MARKERS[p], ms=6, lw=2.0)
    axb.plot(N_VALUES, R[p], color=COLORS[p], marker=MARKERS[p], ms=6, lw=2.0)

# ---- panel (a): mean AoI, log y ----
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

# ---- shared x: log, ticks exactly at N_VALUES ----
axb.set_xscale("log")
axb.set_xlim(N_VALUES[0] * 0.9, N_VALUES[-1] * 1.08)
axb.set_xticks(N_VALUES)
axb.xaxis.set_major_formatter(mticker.FixedFormatter([str(n) for n in N_VALUES]))
axb.xaxis.set_minor_formatter(mticker.NullFormatter())
axb.tick_params(axis="x", which="minor", length=0)
axb.set_xlabel(r"Simultaneous UAVs $N$")

# ---- one legend for the whole figure, above panel (a) ----
handles = [Line2D([0], [0], color=COLORS[p], marker=MARKERS[p], lw=2,
                  label=POL_LABEL.get(p, p)) for p in POL]
axa.legend(handles=handles, ncol=min(len(POL), 4), loc="lower center",
           bbox_to_anchor=(0.5, 1.0), columnspacing=1.2, handlelength=2.2,
           borderaxespad=0.2, frameon=False)

OUT_DIR = os.path.join(DATA_DIR, "plots")
os.makedirs(OUT_DIR, exist_ok=True)
base = os.path.join(OUT_DIR, "fig_impossibility_K1_color")
fig.savefig(base + ".pdf", bbox_inches="tight")
fig.savefig(base + ".png", dpi=200, bbox_inches="tight")
print("saved", base + ".pdf")
