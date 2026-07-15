#!/usr/bin/env python3
"""fig_impossibility_C100.pdf -- impossibility figure at fixed C=100, x = K.
(a) fleet-mean status AoI A(K) = mean_u (mean_t h_{u,s});
(b) worst-case per-UAV risk max_u R̄_u = max_u (mean_t R_u), design-envelope line at 1.
Three policies (RR, PF, Risk-Aware), shared fig_style (grayscale IEEE, single column).

A from drone_summary.mean_h1, max_u R̄_u from drone_summary.mean_r_tot
(both verified == raw per-UAV means).
Output: csv_export_RC7/plots/fig_impossibility_C100.{pdf,png}
"""
import os, glob
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
import fig_style

DATA, SCEN, C = "csv_export_RC7", "4r_b7", 100
K_VALUES = [1, 2, 4, 8, 16, 32]
POL = fig_style.POLICIES

def cell(K, pol):
    fs = sorted(glob.glob(os.path.join(DATA, f"K{K}", f"C{C:02d}", SCEN, f"{pol}_*_drone_summary.csv")))
    if not fs:
        return np.nan, np.nan
    d = pd.read_csv(fs[0], usecols=["mean_h1", "mean_r_tot"])
    return float(d.mean_h1.mean()), float(d.mean_r_tot.max())

A = {p: [] for p in POL}; R = {p: [] for p in POL}
for p in POL:
    for K in K_VALUES:
        a, r = cell(K, p)
        A[p].append(a); R[p].append(r)

# log/linear choice for panel (a)
allA = [v for p in POL for v in A[p]]
dec = np.log10(max(allA) / min(allA))
useLogA = dec > 1.5

fig_style.apply()
fig, (axa, axb) = plt.subplots(2, 1, figsize=(3.5, 4.2), sharex=True, constrained_layout=True)

for p in POL:
    kw = fig_style.line_kwargs(p)
    axa.plot(K_VALUES, A[p], **kw)
    axb.plot(K_VALUES, R[p], **{k: v for k, v in kw.items() if k != "label"})

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

axb.set_xscale("log", base=2)
axb.set_xticks(K_VALUES)
axb.xaxis.set_major_formatter(mticker.FixedFormatter([str(k) for k in K_VALUES]))
axb.xaxis.set_minor_formatter(mticker.NullFormatter())
axb.tick_params(axis="x", which="minor", length=0)
axb.set_xlabel(r"Slots per frame  $K$")

h, l = axa.get_legend_handles_labels()
axa.legend(h, l, ncol=3, loc="lower center", bbox_to_anchor=(0.5, 1.0),
           columnspacing=1.2, handlelength=2.4, borderaxespad=0.2)

OUT = os.path.join(DATA, "plots")
base = os.path.join(OUT, "fig_impossibility_C100")
for ext in ("pdf", "png"):
    fig.savefig(f"{base}.{ext}", dpi=200, bbox_inches="tight")
print("saved", base + ".pdf")
print(f"panel (a) y-scale: {'log' if useLogA else 'linear'} (AoI spans {dec:.2f} decades)")
