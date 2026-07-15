#!/usr/bin/env python3
"""fig_routeAB_compare -- side-by-side risk & AoI profiles: Route A (left) vs
Route B (right), for one representative UAV each, 3 policies, steady/N=C regime.
Rows: (top) composite risk R_u, (mid) C2 AoI h_{u,s}, (bottom) video AoI h_{u,v}.
Route A: full corridor, no ground hotspots. Route B (RouteBhalf): hotspot zone
(build_scenario_variavel.m manualHotspots, final third) shaded.
Output: csv_export_RC7/plots/fig_routeAB_compare_K{K}_C{C}.{pdf,png}
"""
import os, glob, sys
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

DATA, SCEN = "csv_export_RC7", "4r_b7"
K = int(sys.argv[1]) if len(sys.argv) > 1 else 16
C = int(sys.argv[2]) if len(sys.argv) > 2 else 80
UID_A = int(sys.argv[3]) if len(sys.argv) > 3 else 20    # Route A block (1..N/4)
UID_B = int(sys.argv[4]) if len(sys.argv) > 4 else 130   # RouteBhalf block (N/2+1..3N/4)

LEN_A = np.hypot(1000 - 1000, 9000 - 1000)               # RouteA:     8000 m, no hotspots
LEN_B = np.hypot(-700 - (-350), 14000 - 9750)            # RouteBhalf: ~4264 m, hotspots at end
HS0_B = (11166.7 - 9750) / (14000 - 9750) * LEN_B        # hotspot zone start on Route B
HS1_B = LEN_B
XCUT_B = 4100

POL = ["round-robin-sw", "pf-classic-sw", "mr-mw-ch-sw"]
LABEL  = {"round-robin-sw": "RR", "pf-classic-sw": "PF", "mr-mw-ch-sw": "Risk-Aware"}
COLORS = {"round-robin-sw": "#F9A825", "pf-classic-sw": "#E53935", "mr-mw-ch-sw": "#8E24AA"}

def load(uid, route_len):
    out = {}
    base = os.path.join(DATA, f"K{K}", f"C{C:02d}", SCEN)
    for pol in POL:
        ds = pd.read_csv(sorted(glob.glob(os.path.join(base, f"{pol}_*_drone_summary.csv")))[0])
        ss = pd.read_csv(sorted(glob.glob(os.path.join(base, f"{pol}_*_slot_series.csv")))[0],
                         usecols=["slot", "drone_id", "r_tot", "h1_slots", "h2_slots"])
        row = ds[ds.drone_id == uid].iloc[0]
        entry, flen = int(row.entry_slot), int(row.exit_slot - row.entry_slot)
        d = ss[ss.drone_id == uid].sort_values("slot")
        out[pol] = d.assign(dist=(d.slot - entry) / flen * route_len)
    return out

A = load(UID_A, LEN_A)
B = load(UID_B, LEN_B)

plt.rcParams.update({"font.family": "serif", "font.serif": ["Times New Roman", "Nimbus Roman", "DejaVu Serif"],
                     "mathtext.fontset": "stix", "font.size": 9, "pdf.fonttype": 42})
fig, axes = plt.subplots(3, 2, figsize=(8.0, 6.6), sharex="col", sharey="row", constrained_layout=True)

for col, (data, title) in enumerate([(A, f"Route A  (UAV {UID_A}, no hotspots)"),
                                      (B, f"Route B  (UAV {UID_B})")]):
    for pol in POL:
        d = data[pol]
        axes[0][col].plot(d.dist, d.r_tot,    color=COLORS[pol], lw=1.4)
        axes[1][col].plot(d.dist, d.h1_slots, color=COLORS[pol], lw=1.4)
        axes[2][col].plot(d.dist, d.h2_slots, color=COLORS[pol], lw=1.4)
    axes[0][col].set_title(title, fontweight="bold")
    for r in range(3):
        axes[r][col].grid(True, which="both", ls=":", lw=0.4, alpha=0.6)
    # flight phases (altProfile=takeoff_landing; takeoffDist=500 m): climb over the
    # first 500 m, cruise, descend over the last 500 m — per this column's route length.
    route_len = LEN_A if col == 0 else LEN_B
    TO = 500.0
    for r in range(3):
        for xb in (TO, route_len - TO):
            axes[r][col].axvline(xb, color="0.45", lw=0.8, ls="-.", alpha=0.8, zorder=1)
    for xc, name in [(TO / 2, "takeoff"), (route_len / 2, "cruise"),
                     (route_len - TO / 2, "landing")]:
        axes[2][col].text(xc, axes[2][col].get_ylim()[1] * 0.97, name, ha="center",
                          va="top", fontsize=6.5, color="0.4", style="italic")
    if col == 1:  # Route B hotspot zone + crop
        for r in range(3):
            axes[r][col].axvspan(HS0_B, HS1_B, color="0.80", alpha=0.5, lw=0, zorder=0)
            axes[r][col].set_xlim(0, XCUT_B)
        axes[0][col].text(HS0_B, axes[0][col].get_ylim()[1], " hotspot zone",
                          color="0.35", fontsize=7, va="top", ha="left")
    axes[2][col].set_xlabel("Distance along route  [m]")

axes[0][0].set_yscale("log")
axes[0][0].set_ylabel(r"Risk  $R_u$")
axes[1][0].set_ylabel(r"C2 AoI  $h_{u,s}$ (slots)")
axes[2][0].set_ylabel(r"video AoI  $h_{u,v}$ (slots)")

handles = [Line2D([0], [0], color=COLORS[p], lw=2, label=LABEL[p]) for p in POL]
fig.legend(handles=handles, ncol=3, loc="lower center", bbox_to_anchor=(0.5, -0.03), frameon=False, fontsize=10)
fig.suptitle(f"Route A vs Route B — risk & AoI, 3 policies (K={K}, C={C}, steady/N=C)",
             fontsize=10, fontweight="bold")

OUT = os.path.join(DATA, "plots")
b = os.path.join(OUT, f"fig_routeAB_compare_K{K}_C{C}")
for ext in ("pdf", "png"):
    fig.savefig(f"{b}.{ext}", dpi=150, bbox_inches="tight")
    print("saved", f"{b}.{ext}")
for tag, data in [("A", A), ("B", B)]:
    for p in POL:
        d = data[p]
        print(f"  Route {tag} {LABEL[p]:11}: meanR={d.r_tot.mean():7.2f} maxh1={d.h1_slots.max():5.0f} maxh2={d.h2_slots.max():6.0f}")
