#!/usr/bin/env python3
"""fig_impossibility3_C100.pdf -- impossibility figure at fixed C=100 (x=K) with
a THIRD panel for the fleet exceedance probability.
(a) fleet-mean status AoI A(K) = mean_u (mean_t h_{u,s});
(b) worst-case per-UAV risk max_u R̄_u, design-envelope line at 1;
(c) P(R̄_u > 1) = fraction of the fleet whose per-UAV mean risk violates the
    design envelope.
Three policies (RR, PF, Risk-Aware), shared fig_style (grayscale IEEE).
Output: csv_export_RC7/plots/fig_impossibility3_C100.{pdf,png}
"""
import os, glob
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
import fig_style

import sys
DATA, SCEN = "csv_export_RC7", "4r_b7"
C = int(sys.argv[1]) if len(sys.argv) > 1 else 100
K_VALUES = [1, 2, 4, 8, 16, 32]
POL = fig_style.POLICIES

def cell(K, pol):
    fs = sorted(glob.glob(os.path.join(DATA, f"K{K}", f"C{C:02d}", SCEN, f"{pol}_*_drone_summary.csv")))
    if not fs:
        return np.nan, np.nan, np.nan
    m = pd.read_csv(fs[0], usecols=["mean_h1", "mean_r_tot"])
    return float(m.mean_h1.mean()), float(m.mean_r_tot.max()), float((m.mean_r_tot > 1).mean() * 100)

A = {p: [] for p in POL}; R = {p: [] for p in POL}; P = {p: [] for p in POL}
for p in POL:
    for K in K_VALUES:
        a, r, pe = cell(K, p)
        A[p].append(a); R[p].append(r); P[p].append(pe)

allA = [v for p in POL for v in A[p]]
useLogA = np.log10(max(allA) / min(allA)) > 1.5

fig_style.apply()
fig, (axa, axb, axc) = plt.subplots(3, 1, figsize=(3.5, 5.9), sharex=True, constrained_layout=True)

for p in POL:
    kw = fig_style.line_kwargs(p)
    kw2 = {k: v for k, v in kw.items() if k != "label"}
    axa.plot(K_VALUES, A[p], **kw)
    axb.plot(K_VALUES, R[p], **kw2)
    axc.plot(K_VALUES, P[p], **kw2)

axa.set_yscale("log" if useLogA else "linear")
axa.set_ylabel("Mean AoI (slots)")
axa.text(0.03, 0.90, "(a)", transform=axa.transAxes, fontweight="bold", va="top")
axa.grid(True, which="both", ls=":", lw=0.4, alpha=0.6)

axb.set_yscale("log")
axb.set_ylabel(r"$\max_u \bar R_u$")
axb.axhline(1.0, ls=(0, (4, 3)), lw=0.9, color="0.35")
axb.text(0.985, 1.0, "design envelope", transform=axb.get_yaxis_transform(),
         ha="right", va="bottom", fontsize=7, color="0.35")
axb.text(0.03, 0.90, "(b)", transform=axb.transAxes, fontweight="bold", va="top")
axb.grid(True, which="both", ls=":", lw=0.4, alpha=0.6)

axc.set_ylim(-4, 104)
axc.set_ylabel(r"$P(\bar R_u > 1)$  [%]")
axc.text(0.03, 0.90, "(c)", transform=axc.transAxes, fontweight="bold", va="top")
axc.grid(True, ls=":", lw=0.4, alpha=0.6)

axc.set_xscale("log", base=2)
axc.set_xticks(K_VALUES)
axc.xaxis.set_major_formatter(mticker.FixedFormatter([str(k) for k in K_VALUES]))
axc.xaxis.set_minor_formatter(mticker.NullFormatter())
axc.tick_params(axis="x", which="minor", length=0)
axc.set_xlabel(r"Slots per frame  $K$")

h, l = axa.get_legend_handles_labels()
axa.legend(h, l, ncol=3, loc="lower center", bbox_to_anchor=(0.5, 1.0),
           columnspacing=1.2, handlelength=2.4, borderaxespad=0.2)

OUT = os.path.join(DATA, "plots")
base = os.path.join(OUT, f"fig_impossibility3_C{C}")
for ext in ("pdf", "png"):
    fig.savefig(f"{base}.{ext}", dpi=200, bbox_inches="tight")
print("saved", base + ".pdf")
