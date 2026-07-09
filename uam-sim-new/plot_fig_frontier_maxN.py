#!/usr/bin/env python3
"""plot_fig_frontier_maxN.py -- TLS-feasible capacity frontier.
For each K, the LARGEST simulated N whose fleet p99_u R̄_u <= thr.
  x = channels K, y = max feasible N (0 if even N=5 violates).
Censored points (feasible at the largest simulated N) get an up-arrow.

Usage:
  python3 plot_fig_frontier_maxN.py --dir csv_export_RC8v8 \
      --pol round-robin-sw,pf-classic-sw,mw-sw-v8,mw-sw-v9
Output: <dir>/plots/fig_frontier_maxN.{pdf,png}
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
ap.add_argument("--q", type=float, default=0.99)
ap.add_argument("--thr", type=float, default=1.0)
ap.add_argument("--pol", default="round-robin-sw,pf-classic-sw,mw-sw-v8,mw-sw-v9")
args = ap.parse_args()

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(SCRIPT_DIR, args.dir)
SCEN = args.scenario
K_VALUES = [1, 2, 4, 8, 16, 32]
N_VALUES = [5, 10, 20, 40, 60, 80, 100, 120]

POL    = [p for p in args.pol.split(",") if p]
LABEL  = {"round-robin-sw": "RR", "pf-classic-sw": "PF", "mr-mw-ch-sw": "Risk-Aware",
          "max-risk-pf-sw": "Risk-PF", "mw-sw-otimo-ch": "MW-opt (sw)",
          "mw-otimo-ch": "MW-opt", "mw-sw-v8": "MW-V8", "mw-sw-v9": "MW-V9", "mw-v9-nsw": "MW-V9-nsw"}
COLORS = {"round-robin-sw": "#F9A825", "pf-classic-sw": "#E53935", "mr-mw-ch-sw": "#8E24AA",
          "max-risk-pf-sw": "#1E88E5", "mw-sw-otimo-ch": "#3949AB",
          "mw-otimo-ch": "#6D4C41", "mw-sw-v8": "#00897B", "mw-sw-v9": "#7CB342", "mw-v9-nsw": "#33691E"}
MARKERS = {"round-robin-sw": "o", "pf-classic-sw": "s", "mr-mw-ch-sw": "*",
           "max-risk-pf-sw": "D", "mw-sw-otimo-ch": "v", "mw-otimo-ch": "P", "mw-sw-v9": "^", "mw-v9-nsw": "v",
           "mw-sw-v8": "X"}

def p99(K, C, pol):
    fs = sorted(glob.glob(os.path.join(DATA, f"K{K}", f"C{C:02d}", SCEN,
                                        f"{pol}_*_drone_summary.csv")))
    if not fs:
        return np.nan
    d = pd.read_csv(fs[-1], usecols=["mean_r_tot"])
    return float(d.mean_r_tot.quantile(args.q))

frontier = {p: [] for p in POL}     # max feasible N per K (0 = none)
censored = {p: [] for p in POL}     # True if feasible at largest simulated N
for p in POL:
    for K in K_VALUES:
        feas = [N for N in N_VALUES if p99(K, N, p) <= args.thr]
        nmax = max(feas) if feas else 0
        frontier[p].append(nmax)
        censored[p].append(nmax == N_VALUES[-1])
        vals = {N: p99(K, N, p) for N in N_VALUES}
        holes = [N for N in N_VALUES if N < nmax and vals[N] > args.thr]
        if holes:
            print(f"  note: K={K} {p}: non-contiguous frontier, violations below "
                  f"N={nmax} at {holes}")

print("\nmax N with p99<=%.1f per K:" % args.thr)
for p in POL:
    print(f"  {p:16s} " + " ".join(f"K{k}:{n:>3d}" for k, n in zip(K_VALUES, frontier[p])))

plt.rcParams.update({"font.family": "serif",
                     "font.serif": ["Times New Roman", "Nimbus Roman", "DejaVu Serif"],
                     "mathtext.fontset": "stix", "font.size": 9, "pdf.fonttype": 42})
fig, ax = plt.subplots(figsize=(4.2, 3.0), constrained_layout=True)

offsets = np.linspace(-0.06, 0.06, len(POL))
for p, off in zip(POL, offsets):
    xs = [K * (2 ** off) for K in K_VALUES]
    ax.plot(xs, frontier[p], color=COLORS[p], marker=MARKERS[p], ms=6, lw=1.8,
            label=LABEL.get(p, p), clip_on=False, zorder=3)
    for x, n, c in zip(xs, frontier[p], censored[p]):
        if c:   # feasible at the sweep ceiling: true frontier is >= this
            ax.annotate("", xy=(x, n + 11), xytext=(x, n + 2),
                        arrowprops=dict(arrowstyle="-|>", color=COLORS[p], lw=1.2))

ax.set_xscale("log", base=2)
ax.set_xticks(K_VALUES)
ax.xaxis.set_major_formatter(mticker.FixedFormatter([str(k) for k in K_VALUES]))
ax.xaxis.set_minor_formatter(mticker.NullFormatter())
ax.tick_params(axis="x", which="minor", length=0)
ax.set_xlim(K_VALUES[0] * 0.8, K_VALUES[-1] * 1.25)
ax.set_xlabel(r"Channels $K$")

ax.set_ylim(-5, 138)
ax.set_yticks(N_VALUES)
ax.set_ylabel(rf"Max $N$ with $p_{{{int(round(100*args.q))}}}[\bar R_u] \leq {args.thr:g}$")
ax.grid(True, which="both", ls=":", lw=0.4, alpha=0.6)
ax.axhline(N_VALUES[-1], ls=":", lw=0.6, color="0.6")
ax.text(K_VALUES[0] * 0.82, N_VALUES[-1] + 3, "sweep ceiling",
        fontsize=6.5, color="0.45", va="bottom")

ax.legend(loc="upper left", fontsize=8, frameon=False, handlelength=2.0,
          borderaxespad=0.3)

OUT = os.path.join(DATA, "plots")
os.makedirs(OUT, exist_ok=True)
base = os.path.join(OUT, "fig_frontier_maxN")
fig.savefig(base + ".pdf", bbox_inches="tight")
fig.savefig(base + ".png", dpi=200, bbox_inches="tight")
print("saved", base + ".pdf")
