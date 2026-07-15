#!/usr/bin/env python3
"""fig_routeB_drone_profile -- risk + C2 AoI + video AoI along Route B for a
single UAV (steady/N=C regime), overlaying the 3 policies.
UAV 97 (RouteBhalf, drone_id 78..115), K=1, C=60. The drone id, entry/exit and
ground hotspot are identical across policies (seed=55, static routing); only the
scheduling-dependent AoIs and risk differ.
Panels: (a) composite risk R_u, (b) C2 AoI h_{u,s}, (c) video AoI h_{u,v}.
x = distance travelled along Route B (start y=9750 -> goal y=14000, len~4264 m).

Provenance: csv_export_RC7/K1/C60/4r_b7/<policy>_*_slot_series.csv (+drone_summary).
Output: csv_export_RC7/plots/fig_routeB_drone_profile.{pdf,png}
"""
import os, glob
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

import sys
DATA, SCEN = "csv_export_RC7", "4r_b7"
K   = int(sys.argv[1]) if len(sys.argv) > 1 else 1
C   = int(sys.argv[2]) if len(sys.argv) > 2 else 60
UID = int(sys.argv[3]) if len(sys.argv) > 3 else 97
ROUTE_LEN = np.hypot(-700 - (-350), 14000 - 9750)   # ~4264 m
POL = ["round-robin-sw", "pf-classic-sw", "mr-mw-ch-sw"]
LABEL  = {"round-robin-sw": "RR", "pf-classic-sw": "PF", "mr-mw-ch-sw": "Risk-Aware"}
COLORS = {"round-robin-sw": "#F9A825", "pf-classic-sw": "#E53935", "mr-mw-ch-sw": "#8E24AA"}

def load(pol):
    base = os.path.join(DATA, f"K{K}", f"C{C:02d}", SCEN)
    ds = pd.read_csv(sorted(glob.glob(os.path.join(base, f"{pol}_*_drone_summary.csv")))[0])
    ss = pd.read_csv(sorted(glob.glob(os.path.join(base, f"{pol}_*_slot_series.csv")))[0],
                     usecols=["slot", "drone_id", "r_map", "r_tot", "h1_slots", "h2_slots"])
    row = ds[ds.drone_id == UID].iloc[0]
    entry, flen = int(row.entry_slot), int(row.exit_slot - row.entry_slot)
    d = ss[ss.drone_id == UID].sort_values("slot")
    d = d.assign(dist=(d.slot - entry) / flen * ROUTE_LEN)
    return d

data = {p: load(p) for p in POL}

plt.rcParams.update({"font.family": "serif", "font.serif": ["Times New Roman", "Nimbus Roman", "DejaVu Serif"],
                     "mathtext.fontset": "stix", "font.size": 9, "pdf.fonttype": 42})
fig, (ax0, ax1, ax2) = plt.subplots(3, 1, figsize=(6.6, 6.6), sharex=True, constrained_layout=True)

for p in POL:
    d = data[p]
    ax0.plot(d.dist, d.r_tot,    color=COLORS[p], lw=1.8)
    ax1.plot(d.dist, d.h1_slots, color=COLORS[p], lw=1.8)
    ax2.plot(d.dist, d.h2_slots, color=COLORS[p], lw=1.8)

# Hotspot ZONE: fixed spatial location from build_scenario_variavel.m manualHotspots
# (8 clusters, E = 11166.7 .. 14000, final third of the route). Projected onto the
# along-route distance (E 9750 -> 14000 on RouteBhalf); policy-independent.
hs0 = (11166.7 - 9750) / (14000 - 9750) * ROUTE_LEN
hs1 = (14000.0 - 9750) / (14000 - 9750) * ROUTE_LEN
for ax in (ax0, ax1, ax2):
    ax.axvspan(hs0, hs1, color="0.80", alpha=0.5, lw=0, zorder=0)
    ax.grid(True, which="both", ls=":", lw=0.4, alpha=0.6)
ax0.text(hs0, ax0.get_ylim()[1], " hotspot zone", color="0.35", fontsize=7, va="top", ha="left")

# Flight phases (altProfile=takeoff_landing; takeoffDist=500 m, cruiseAlt=60 m):
# climb over the first 500 m, cruise, descend over the last 500 m — ramp phases use
# phase-aware risk weights (w_tol_vid=0.80).
TO = 500.0
for ax in (ax0, ax1, ax2):
    for xb in (TO, ROUTE_LEN - TO):
        ax.axvline(xb, color="0.45", lw=0.8, ls="-.", alpha=0.8, zorder=1)
for xc, name in [(TO / 2, "takeoff"), ((TO + ROUTE_LEN - TO) / 2, "cruise"),
                 (ROUTE_LEN - TO / 2, "landing")]:
    ax2.text(xc, ax2.get_ylim()[1] * 0.97, name, ha="center", va="top",
             fontsize=6.5, color="0.4", style="italic")

ax0.set_yscale("log")
ax0.set_ylabel(r"Risk  $R_u$"); ax0.text(0.02, 0.9, "(a)", transform=ax0.transAxes, fontweight="bold", va="top")
ax1.set_ylabel(r"C2 AoI  $h_{u,s}$ (slots)"); ax1.text(0.02, 0.9, "(b)", transform=ax1.transAxes, fontweight="bold", va="top")
ax2.set_ylabel(r"video AoI  $h_{u,v}$ (slots)"); ax2.text(0.02, 0.9, "(c)", transform=ax2.transAxes, fontweight="bold", va="top")
ax2.set_xlabel("Distance along Route B  [m]   (start 9750 m $\\rightarrow$ goal 14000 m)")
ax2.set_xlim(0, 4100)   # crop the very end of the route (shared x)

handles = [Line2D([0], [0], color=COLORS[p], lw=2, label=LABEL[p]) for p in POL]
fig.legend(handles=handles, ncol=3, loc="lower center", bbox_to_anchor=(0.5, -0.03), frameon=False, fontsize=9)
fig.suptitle(f"Route-B UAV #{UID} — risk & AoI profiles, 3 policies (K={K}, C={C}, steady/N=C)",
             fontsize=10, fontweight="bold")

OUT = os.path.join(DATA, "plots")
b = os.path.join(OUT, f"fig_routeB_drone_profile_K{K}_C{C}")
for ext in ("pdf", "png"):
    fig.savefig(f"{b}.{ext}", dpi=150, bbox_inches="tight")
    print("saved", f"{b}.{ext}")
for p in POL:
    d = data[p]
    print(f"  {LABEL[p]:11}: mean R_u={d.r_tot.mean():7.2f}  max h1={d.h1_slots.max():5.0f}  max h2={d.h2_slots.max():6.0f}")
