#!/usr/bin/env python3
"""plot_routeB_profile_compare.py -- risk + C2 AoI + video AoI along Route B for a
single UAV (steady/N=C regime), overlaying an arbitrary policy set (default:
Risk-Aware heuristic vs MW-V8). Same figure as plot_routeB_drone_profile.py but
with --dir/--pol/--uid args.

The drone id, entry/exit and ground hotspot are identical across policies
(seed=55, static routing); only the scheduling-dependent AoIs and risk differ.
Panels: (a) composite risk R_u, (b) C2 AoI h_{u,s}, (c) video AoI h_{u,v}.
x = distance travelled along Route B (start y=9750 -> goal y=14000, len~4264 m).

--uid auto picks the middle UAV of the RouteBhalf block (drone ids N/2+1..3N/4,
identical across policies).

Usage:
  python3 plot_routeB_profile_compare.py --K 16 --C 80 --dir csv_export_RC8v8 \
      --pol mr-mw-ch-sw,mw-sw-v8 [--uid 125]
Output: <dir>/plots/fig_routeB_profile_<tags>_K{K}_C{C}_u{UID}.{pdf,png}
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
ap.add_argument("--K", type=int, default=16)
ap.add_argument("--C", type=int, default=80)
ap.add_argument("--pol", default="mr-mw-ch-sw,mw-sw-v8")
ap.add_argument("--uid", default="auto", help="'auto' = middle of the RouteBhalf block")
args = ap.parse_args()

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(SCRIPT_DIR, args.dir)
SCEN, K, C = args.scenario, args.K, args.C
ROUTE_LEN = np.hypot(-700 - (-350), 14000 - 9750)   # ~4264 m

POL    = [p for p in args.pol.split(",") if p]
LABEL  = {"round-robin-sw": "RR", "pf-classic-sw": "PF", "mr-mw-ch-sw": "Risk-Aware",
          "max-risk-pf-sw": "Risk-PF", "mw-sw-otimo-ch": "MW-opt (sw)",
          "mw-otimo-ch": "MW-opt", "mw-sw-v8": "MW-V8", "mw-sw-v9": "MW-V9", "mw-v9-nsw": "MW-V9-nsw"}
COLORS = {"round-robin-sw": "#F9A825", "pf-classic-sw": "#E53935", "mr-mw-ch-sw": "#8E24AA",
          "max-risk-pf-sw": "#1E88E5", "mw-sw-otimo-ch": "#3949AB",
          "mw-otimo-ch": "#6D4C41", "mw-sw-v8": "#00897B", "mw-sw-v9": "#7CB342", "mw-v9-nsw": "#33691E"}

BASE = os.path.join(DATA, f"K{K}", f"C{C:02d}", SCEN)

# ---- UID: middle of the RouteBhalf block (ids N/2+1..3N/4, same across policies)
_ds0 = pd.read_csv(sorted(glob.glob(os.path.join(BASE, f"{POL[0]}_*_drone_summary.csv")))[-1])
N = len(_ds0)
UID = (N // 2 + 1 + 3 * N // 4) // 2 if args.uid == "auto" else int(args.uid)
print(f"N={N}, RouteBhalf block={N//2+1}..{3*N//4}, UID={UID}")

def load(pol):
    ds = pd.read_csv(sorted(glob.glob(os.path.join(BASE, f"{pol}_*_drone_summary.csv")))[-1])
    ss = pd.read_csv(sorted(glob.glob(os.path.join(BASE, f"{pol}_*_slot_series.csv")))[-1],
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

# Flight phases (altProfile=takeoff_landing; takeoffDist=500 m, cruiseAlt=60 m)
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

handles = [Line2D([0], [0], color=COLORS[p], lw=2, label=LABEL.get(p, p)) for p in POL]
fig.legend(handles=handles, ncol=len(POL), loc="lower center", bbox_to_anchor=(0.5, -0.03), frameon=False, fontsize=9)
fig.suptitle(f"Route-B UAV #{UID} — risk & AoI profiles, {' vs '.join(LABEL.get(p,p) for p in POL)} (K={K}, C={C}, steady/N=C)",
             fontsize=10, fontweight="bold")

OUT = os.path.join(DATA, "plots")
os.makedirs(OUT, exist_ok=True)
tags = "_".join(LABEL.get(p, p).replace(" ", "").replace("(", "").replace(")", "") for p in POL)
b = os.path.join(OUT, f"fig_routeB_profile_{tags}_K{K}_C{C}_u{UID}")
for ext in ("pdf", "png"):
    fig.savefig(f"{b}.{ext}", dpi=150, bbox_inches="tight")
    print("saved", f"{b}.{ext}")
for p in POL:
    d = data[p]
    print(f"  {LABEL.get(p,p):11}: mean R_u={d.r_tot.mean():7.2f}  max h1={d.h1_slots.max():5.0f}  max h2={d.h2_slots.max():6.0f}")
