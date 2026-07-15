#!/usr/bin/env python3
"""
plot_fig_impossibility_aoi_split.py -- "impossibility" figure (K=1 slice) with the
mean AoI SPLIT by stream, split-figure color convention:
  (a) fleet-mean C2 AoI    A_s(N)  [per-UAV time mean of h1_slots, then mean over u]
  (b) fleet-mean video AoI A_v(N)  [per-UAV time mean of h2_slots, then mean over u]
  (c) risk: --risk maxu (default) worst-case per-UAV risk max_u R̄_u
            [per-UAV time mean of r_tot, then max over u];
      --risk sys mean SYSTEM risk R̄_sys (mean_r_sys from the master summary,
            steady window) -> output suffix _rsys.

Computed from RAW slot_series traces (same metrics as plot_fig_impossibility.py;
h2_slots = AoI of the latest completed video frame, step_transmit.m:97,145).

Usage:
  python3 plot_fig_impossibility_aoi_split.py --dir csv_export_RC8v8 \
      --pol round-robin-sw,pf-classic-sw,mr-mw-ch-sw,mw-sw-v8 [--risk sys]
Output: <dir>/plots/fig_impossibility_K1_aoi_split[_rsys].{pdf,png}
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
ap.add_argument("--risk", choices=["maxu", "meanu", "sys"], default="maxu",
                help="panel (c): maxu = max_u R̄_u; meanu = fleet-mean R̄_u "
                     "(both from slot_series); sys = mean R̄_sys (summary)")
args = ap.parse_args()

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR   = os.path.join(SCRIPT_DIR, args.dir)
SCEN = args.scenario
K = 1
N_VALUES = [5, 10, 20, 40, 60, 80, 100, 120]

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

def _runs(C, pol):
    return sorted(glob.glob(os.path.join(DATA_DIR, f"K{K}", f"C{C:02d}", SCEN,
                                          f"{pol}_*_slot_series.csv")))

def cell(C, pol):
    """A_s(N), A_v(N), max_u R̄_u averaged over runs (runs are byte-identical here)."""
    fs = _runs(C, pol)
    if not fs:
        return np.nan, np.nan, np.nan
    As, Av, Rs = [], [], []
    for f in fs:
        df = pd.read_csv(f, usecols=["drone_id", "h1_slots", "h2_slots", "r_tot"],
                         dtype={"drone_id": np.int32, "h1_slots": np.float32,
                                "h2_slots": np.float32, "r_tot": np.float32})
        g = df.groupby("drone_id")
        As.append(g.h1_slots.mean().mean())     # per-UAV time-mean, then mean over u
        Av.append(g.h2_slots.mean().mean())
        rb = g.r_tot.mean()                      # per-UAV time-mean R̄_u
        Rs.append(rb.mean() if args.risk == "meanu" else rb.max())
    return float(np.mean(As)), float(np.mean(Av)), float(np.mean(Rs))

A_s = {p: [] for p in POL}
A_v = {p: [] for p in POL}
R   = {p: [] for p in POL}
for p in POL:
    for C in N_VALUES:
        a, v, r = cell(C, p)
        A_s[p].append(a); A_v[p].append(v); R[p].append(r)

if args.risk == "sys":
    # panel (c) from the master summary: mean system risk on the steady window
    summ = pd.read_csv(os.path.join(DATA_DIR, "sweep_summary_all.csv"))
    summ = summ[(summ.scenario == SCEN) & (summ.K == K)]
    for p in POL:
        d = summ[summ.policy == p].set_index("capacity")["mean_r_sys"]
        R[p] = [float(d.get(C, np.nan)) for C in N_VALUES]

plt.rcParams.update({"font.family": "serif",
                     "font.serif": ["Times New Roman", "Nimbus Roman", "DejaVu Serif"],
                     "mathtext.fontset": "stix", "font.size": 9, "pdf.fonttype": 42})
fig, (axa, axb, axc) = plt.subplots(3, 1, figsize=(4.2, 7.2), sharex=True,
                                    constrained_layout=True)

for p in POL:
    axa.plot(N_VALUES, A_s[p], color=COLORS[p], marker=MARKERS[p], ms=6, lw=2.0)
    axb.plot(N_VALUES, A_v[p], color=COLORS[p], marker=MARKERS[p], ms=6, lw=2.0)
    axc.plot(N_VALUES, R[p],   color=COLORS[p], marker=MARKERS[p], ms=6, lw=2.0)

# ---- panel (a): mean C2 AoI, log y ----
axa.set_yscale("log")
axa.set_ylabel(r"Mean C2 AoI $h_{u,s}$ (slots)")
axa.text(0.03, 0.92, "(a)", transform=axa.transAxes, fontweight="bold", va="top")
axa.grid(True, which="both", ls="--", lw=0.5, alpha=0.4)

# ---- panel (b): mean video AoI, log y ----
axb.set_yscale("log")
axb.set_ylabel(r"Mean video AoI $h_{u,v}$ (slots)")
axb.text(0.03, 0.92, "(b)", transform=axb.transAxes, fontweight="bold", va="top")
axb.grid(True, which="both", ls="--", lw=0.5, alpha=0.4)

# ---- panel (c): risk, log y ----
axc.set_yscale("log")
if args.risk == "sys":
    axc.set_ylabel(r"Mean system risk  $\bar R_{sys}$")
elif args.risk == "meanu":
    axc.set_ylabel(r"Fleet-mean risk  $\mathrm{mean}_u\, \bar R_u$")
    axc.axhline(1.0, ls=(0, (4, 3)), lw=1.0, color="0.35")
else:
    axc.set_ylabel(r"$\max_u \bar R_u$")
    axc.axhline(1.0, ls=(0, (4, 3)), lw=1.0, color="0.35")
    axc.text(0.985, 1.0, "design envelope", transform=axc.get_yaxis_transform(),
             ha="right", va="bottom", fontsize=8, color="0.35")
axc.text(0.03, 0.92, "(c)", transform=axc.transAxes, fontweight="bold", va="top")
axc.grid(True, which="both", ls="--", lw=0.5, alpha=0.4)

# ---- shared x: log, ticks exactly at N_VALUES ----
axc.set_xscale("log")
axc.set_xlim(N_VALUES[0] * 0.9, N_VALUES[-1] * 1.08)
axc.set_xticks(N_VALUES)
axc.xaxis.set_major_formatter(mticker.FixedFormatter([str(n) for n in N_VALUES]))
axc.xaxis.set_minor_formatter(mticker.NullFormatter())
axc.tick_params(axis="x", which="minor", length=0)
axc.set_xlabel(r"Simultaneous UAVs $N$")

# ---- one legend for the whole figure, above panel (a) ----
handles = [Line2D([0], [0], color=COLORS[p], marker=MARKERS[p], lw=2,
                  label=POL_LABEL.get(p, p)) for p in POL]
axa.legend(handles=handles, ncol=min(len(POL), 4), loc="lower center",
           bbox_to_anchor=(0.5, 1.0), columnspacing=1.2, handlelength=2.2,
           borderaxespad=0.2, frameon=False)

OUT_DIR = os.path.join(DATA_DIR, "plots")
os.makedirs(OUT_DIR, exist_ok=True)
base = os.path.join(OUT_DIR, "fig_impossibility_K1_aoi_split"
                    + {"sys": "_rsys", "meanu": "_meanu", "maxu": ""}[args.risk])
fig.savefig(base + ".pdf", bbox_inches="tight")
fig.savefig(base + ".png", dpi=200, bbox_inches="tight")
print("saved", base + ".pdf")
