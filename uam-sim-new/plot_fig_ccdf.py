#!/usr/bin/env python3
"""fig_ccdf.pdf -- CCDF of instantaneous per-UAV risk R_u(t) at the crossover
cell (K=1, N=100), pooled over (u, t). log y, log x (support > 2 decades).
Vertical dashed line at x=1 = design envelope. Three policies, shared fig_style.

Provenance: raw csv_export_RC7/K1/C100/4r_b7/<policy>_*_slot_series.csv, column r_tot.
Output: csv_export_RC7/plots/fig_ccdf.{pdf,png} + fig_ccdf_numbers.md
"""
import os, glob
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
from matplotlib.lines import Line2D
import fig_style

DATA, SCEN, K, N = "csv_export_RC7", "4r_b7", 1, 100
POL = fig_style.POLICIES  # RR, PF, Risk-Aware

def pooled(K, C, pol):
    f = sorted(glob.glob(os.path.join(DATA, f"K{K}", f"C{C:02d}", SCEN, f"{pol}_*_slot_series.csv")))[0]
    return pd.read_csv(f, usecols=["r_tot"]).r_tot.values.astype(float)

def ccdf_xy(r, npts=2500):
    xs = np.sort(r[r > 0])                       # log x -> positive support
    n = xs.size
    p = (n - np.arange(n)) / n                   # P(R >= xs) ~ CCDF
    idx = np.unique(np.linspace(0, n - 1, npts).astype(int))
    return xs[idx], p[idx]

series = {p: pooled(K, N, p) for p in POL}

# ---- crossover x where PF CCDF rises above RR (heavier tail) ----
grid = np.logspace(0, np.log10(max(series["pf-classic-sw"].max(),
                                   series["round-robin-sw"].max())), 4000)
ccdf_rr = np.array([(series["round-robin-sw"] > x).mean() for x in grid])
ccdf_pf = np.array([(series["pf-classic-sw"] > x).mean() for x in grid])
# The curves cross several times (PF a hair above at x~1, RR above mid-range,
# PF heavier in the UPPER TAIL, then both vanish). Report the tail crossover:
# the upward crossing (RR>=PF -> PF>RR) within the region where the CCDFs are
# still non-negligible (ignore the far tail where both -> 0).
diff = ccdf_pf - ccdf_rr
valid = np.maximum(ccdf_pf, ccdf_rr) > 1e-4
up = [i for i in range(len(grid) - 1) if diff[i] <= 0 < diff[i + 1] and valid[i + 1]]
x_cross = grid[up[-1] + 1] if up else None

fig_style.apply()
fig, ax = plt.subplots(figsize=(3.5, 2.7), constrained_layout=True)
for p in POL:
    c, ls, mk = fig_style.STYLE[p]
    x, y = ccdf_xy(series[p])
    ax.plot(x, y, color=c, linestyle=ls, marker=mk, markevery=0.12, ms=3.5,
            markerfacecolor=c, label=fig_style.LABEL[p])
ax.axvline(1.0, ls=(0, (4, 3)), lw=0.9, color="0.4")
ax.text(1.0, 0.55, "design envelope", rotation=90, va="center", ha="right", fontsize=7, color="0.4")
ax.set_xscale("log"); ax.set_yscale("log")
xmax = max(series[p].max() for p in POL)
ax.set_xlim(0.1, xmax * 1.4)          # focus on the informative tail (x<0.1 is flat at P=1)
ax.set_xlabel(r"Per-UAV risk $R_u(t)$")
ax.set_ylabel(r"$P(R_u > x)$")
ax.grid(True, which="both", ls=":", lw=0.4, alpha=0.6)
ax.legend(loc="lower left")

OUT = os.path.join(DATA, "plots")
base = os.path.join(OUT, "fig_ccdf")
for ext in ("pdf", "png"):
    fig.savefig(f"{base}.{ext}", dpi=200, bbox_inches="tight")
print("saved", base + ".pdf")

# ================= numbers report =================
def s4(x):
    import math
    if not np.isfinite(x): return "—"
    if x == 0: return "0.000"
    d = max(0, 3 - int(math.floor(math.log10(abs(x)))))
    return f"{x:.{d}f}"

def cellnums(C):
    out = {}
    for p in POL:
        r = pooled(K, C, p)
        out[p] = dict(Pg1=100 * (r > 1).mean(), Pg10=100 * (r > 10).mean(),
                      p99=np.percentile(r, 99), p999=np.percentile(r, 99.9))
    return out

n100, n120 = cellnums(100), cellnums(120)
lines = ["# fig_ccdf numbers (pooled R_u(t) over (u,t))\n",
         "Provenance: `csv_export_RC7/K1/C{100,120}/4r_b7/<policy>_*_slot_series.csv`, column `r_tot`.\n",
         "## K=1, N=100 (crossover cell)\n",
         "| policy | P(R>1) % | P(R>10) % | p99 | p99.9 |",
         "|---|---|---|---|---|"]
for p in POL:
    d = n100[p]
    lines.append(f"| {fig_style.LABEL[p]} | {s4(d['Pg1'])} | {s4(d['Pg10'])} | {s4(d['p99'])} | {s4(d['p999'])} |")
lines += ["\n## K=1, N=120 (comparison)\n",
          "| policy | P(R>1) % | P(R>10) % | p99 | p99.9 |",
          "|---|---|---|---|---|"]
for p in POL:
    d = n120[p]
    lines.append(f"| {fig_style.LABEL[p]} | {s4(d['Pg1'])} | {s4(d['Pg10'])} | {s4(d['p99'])} | {s4(d['p999'])} |")
lines.append(f"\n- **PF tail crosses above RR at x ≈ {s4(x_cross) if x_cross else 'n/a'}** "
             f"(for R_u > this value, P_PF(R>x) > P_RR(R>x) — PF has the heavier upper tail; "
             f"consistent with PF p99 {s4(n100['pf-classic-sw']['p99'])} > RR p99 {s4(n100['round-robin-sw']['p99'])} at N=100).")
lines.append(f"- **x support:** ~8 decades → log x. Inset (starved-UAV h_s trace) OMITTED: "
             f"illegible at 3.5 in single-column; available on request as a separate panel.")

with open(os.path.join(OUT, "fig_ccdf_numbers.md"), "w") as fh:
    fh.write("\n".join(lines) + "\n")
print("wrote", os.path.join(OUT, "fig_ccdf_numbers.md"))
print(f"crossover x ≈ {x_cross}")
