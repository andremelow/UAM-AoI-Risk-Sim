#!/usr/bin/env python3
"""
plot_fig_impossibility_mw.py -- IEEE single-column "impossibility" figure (K=1 slice),
Max-Weight family only: mr-mw-ch-sw (Risk-Aware) vs mw-sw-otimo-ch vs mw-otimo-ch.
  (a) fleet-mean time-average status AoI A(N)  [per-UAV time mean, then mean over u]
  (b) worst-case per-UAV risk max_u R̄_u        [per-UAV time mean of r_tot, then max over u]

Computed from RAW slot_series traces only (no aggregated tables). Column mapping,
verified in the MATLAB source:
  h1_slots = h_{u,s}(t) status AoI  (init_state.m:5, step_transmit.m:82)
  r_tot    = R_u(t) composite risk  (step_risk.m:106)

Usage:  python3 plot_fig_impossibility_mw.py [csv_export_RC8]
Output: <dir>/plots/fig_impossibility_K1_mw.{pdf,png}
"""
import os, glob, sys
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
import fig_style

DATA_DIR = sys.argv[1] if len(sys.argv) > 1 else "csv_export_RC8"
SCEN = "4r_b7"
K = 1
N_VALUES = [5, 10, 20, 40, 60, 80, 100, 120]
POL = ["round-robin-sw", "pf-classic-sw", "mr-mw-ch-sw", "mw-sw-otimo-ch", "mw-otimo-ch",
       "mw-sw-v8"]
# keep only policies with data on disk (e.g. mw-sw-v8 appears after merging RC9)
POL = [p for p in POL
       if glob.glob(os.path.join(DATA_DIR, f"K{K}", "C*", SCEN, f"{p}_*_slot_series.csv"))]
print("policies:", POL)

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

fig_style.apply()
fig, (axa, axb) = plt.subplots(2, 1, figsize=(3.5, 4.2), sharex=True,
                               constrained_layout=True)

for p in POL:
    kw = fig_style.line_kwargs(p)
    axa.plot(N_VALUES, A[p], **kw)
    axb.plot(N_VALUES, R[p], **{k: v for k, v in kw.items() if k != "label"})

# ---- panel (a): mean AoI, log y ----
axa.set_yscale("log")
axa.set_ylabel("Mean AoI (slots)")
axa.text(0.03, 0.90, "(a)", transform=axa.transAxes, fontweight="bold", va="top")
axa.grid(True, which="both", ls=":", lw=0.4, alpha=0.6)

# ---- panel (b): max_u R̄_u, log y, design-envelope line at 1 ----
axb.set_yscale("log")
axb.set_ylabel(r"$\max_u \bar R_u$")
axb.axhline(1.0, ls=(0, (4, 3)), lw=0.9, color="0.35")
axb.text(0.985, 1.0, "design envelope", transform=axb.get_yaxis_transform(),
         ha="right", va="bottom", fontsize=7, color="0.35")
axb.text(0.03, 0.90, "(b)", transform=axb.transAxes, fontweight="bold", va="top")
axb.grid(True, which="both", ls=":", lw=0.4, alpha=0.6)

# ---- shared x: log, ticks exactly at N_VALUES ----
axb.set_xscale("log")
axb.set_xlim(N_VALUES[0] * 0.9, N_VALUES[-1] * 1.08)
axb.set_xticks(N_VALUES)
axb.xaxis.set_major_formatter(mticker.FixedFormatter([str(n) for n in N_VALUES]))
axb.xaxis.set_minor_formatter(mticker.NullFormatter())
axb.tick_params(axis="x", which="minor", length=0)
axb.set_xlabel(r"Simultaneous UAVs $N$")

# ---- one legend for the whole figure, above panel (a) ----
h, l = axa.get_legend_handles_labels()
axa.legend(h, l, ncol=3 if len(POL) > 4 else len(POL), loc="lower center", bbox_to_anchor=(0.5, 1.0),
           columnspacing=1.2, handlelength=2.4, borderaxespad=0.2)

OUT_DIR = os.path.join(DATA_DIR, "plots")
os.makedirs(OUT_DIR, exist_ok=True)
base = os.path.join(OUT_DIR, "fig_impossibility_K1_mw")
fig.savefig(base + ".pdf", bbox_inches="tight")
fig.savefig(base + ".png", dpi=200, bbox_inches="tight")
print("saved", base + ".pdf")
