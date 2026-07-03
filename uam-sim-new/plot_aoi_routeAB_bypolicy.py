#!/usr/bin/env python3
"""fig_aoi_routeAB_bypolicy -- per-UAV AoI on Route A vs Route B, BY FLIGHT PHASE,
per policy. Grid: rows = metric (C2 AoI h_{u,s}; video AoI h_{u,v}), cols = policy.
Each panel: x = flight phase (takeoff / cruise / landing), grouped bars Route A vs B.
Phases from altProfile=takeoff_landing (takeoffDist=500 m): the first/last 500 m of
the along-route distance are ramps, the middle is cruise. Per-UAV mean within each
phase first, then averaged over the route's UAVs.
Output: csv_export_RC7/plots/fig_aoi_routeAB_bypolicy_K{K}_C{C}.{pdf,png}
"""
import os, glob, sys
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SCEN = "4r_b7"
K = int(sys.argv[1]) if len(sys.argv) > 1 else 16
C = int(sys.argv[2]) if len(sys.argv) > 2 else 80
DATA = sys.argv[3] if len(sys.argv) > 3 else "csv_export_RC7"
POL = ["round-robin-sw", "pf-classic-sw", "mr-mw-ch-sw", "max-risk-pf-sw"]
LABEL = {"round-robin-sw": "RR", "pf-classic-sw": "PF", "mr-mw-ch-sw": "Risk-Aware",
         "max-risk-pf-sw": "Risk-PF"}
LEN_A, LEN_B, TO = 8000.0, np.hypot(-700 - (-350), 14000 - 9750), 500.0
PHASES = ["takeoff", "cruise", "landing"]
COL_A, COL_B = "#4C9F70", "#B4436C"

def phase_of(dist, L):
    return np.where(dist < TO, "takeoff", np.where(dist > L - TO, "landing", "cruise"))

def per_phase(pol, block, L):
    """Mean per-UAV AoI in each phase, averaged over the block's UAVs. Returns dict."""
    base = os.path.join(DATA, f"K{K}", f"C{C:02d}", SCEN)
    ds = pd.read_csv(sorted(glob.glob(os.path.join(base, f"{pol}_*_drone_summary.csv")))[0])
    ss = pd.read_csv(sorted(glob.glob(os.path.join(base, f"{pol}_*_slot_series.csv")))[0],
                     usecols=["slot", "drone_id", "h1_slots", "h2_slots", "r_tot"])
    ids = ds[(ds.drone_id >= block[0]) & (ds.drone_id <= block[1])]
    ent = dict(zip(ids.drone_id, ids.entry_slot)); fl = dict(zip(ids.drone_id, ids.exit_slot - ids.entry_slot))
    d = ss[ss.drone_id.isin(set(ids.drone_id))].copy()
    d["dist"] = (d.slot - d.drone_id.map(ent)) / d.drone_id.map(fl) * L
    d["phase"] = phase_of(d.dist.values, L)
    cols = ["h1_slots", "h2_slots", "r_tot"]
    perU = d.groupby(["drone_id", "phase"])[cols].mean()                       # per-UAV first
    m = perU.groupby("phase").mean()                                           # then over UAVs
    return {ph: tuple(m.loc[ph, c] if ph in m.index else np.nan for c in cols) for ph in PHASES}

# blocks: Route A = 1..N/4 ; Route B = N/2+1..3N/4  (N from any drone_summary)
_ds = pd.read_csv(sorted(glob.glob(os.path.join(DATA, f"K{K}", f"C{C:02d}", SCEN,
                                                "round-robin-sw_*_drone_summary.csv")))[0])
N = len(_ds); BLK_A = (1, N // 4); BLK_B = (N // 2 + 1, 3 * N // 4)

DATAph = {p: {"A": per_phase(p, BLK_A, LEN_A), "B": per_phase(p, BLK_B, LEN_B)} for p in POL}

plt.rcParams.update({"font.family": "serif", "font.serif": ["Times New Roman", "Nimbus Roman", "DejaVu Serif"],
                     "mathtext.fontset": "stix", "font.size": 9, "pdf.fonttype": 42})
from matplotlib.patches import Patch
PH_COL = {"takeoff": "#6BAED6", "cruise": "#74C476", "landing": "#FD8D3C"}  # color = phase
OFF = [-0.33, -0.21, -0.09, 0.09, 0.21, 0.33]                              # A(3) | B(3)
BW = 0.11

fig, (ax0, ax1, ax2) = plt.subplots(3, 1, figsize=(8.0, 8.2), constrained_layout=True)
xp = np.arange(len(POL))

for ax, mkey, ylab, logy in [(ax0, 2, r"Risk  $R_u$", True),
                             (ax1, 0, r"C2 AoI $h_{u,s}$ (slots)", False),
                             (ax2, 1, r"video AoI $h_{u,v}$ (slots)", False)]:
    for pi, p in enumerate(POL):
        vals = ([DATAph[p]["A"][ph][mkey] for ph in PHASES] +
                [DATAph[p]["B"][ph][mkey] for ph in PHASES])
        for k, v in enumerate(vals):
            ph = PHASES[k % 3]; route_hatch = "" if k < 3 else "////"
            ax.bar(xp[pi] + OFF[k], v, BW, color=PH_COL[ph], hatch=route_hatch,
                   edgecolor="0.25", lw=0.4)
            fmt = f"{v:.2g}" if logy else f"{v:.0f}"
            ax.text(xp[pi] + OFF[k], v, fmt, ha="center", va="bottom", fontsize=5.0, rotation=90)
    for pv in xp[:-1]:
        ax.axvline(pv + 0.5, color="0.85", lw=0.6)
    ax.set_xticks(xp); ax.set_xticklabels([LABEL[p] for p in POL], fontweight="bold")
    ax.set_ylabel(ylab)
    if logy:
        ax.set_yscale("log")
    ax.margins(y=0.18)
    ax.grid(True, axis="y", ls=":", lw=0.4, alpha=0.6)

leg1 = [Patch(facecolor=PH_COL[ph], edgecolor="0.25", label=ph) for ph in PHASES]
leg2 = [Patch(facecolor="0.9", edgecolor="0.25", label="Route A"),
        Patch(facecolor="0.9", edgecolor="0.25", hatch="////", label="Route B")]
ax0.legend(handles=leg1 + leg2, ncol=5, loc="upper left", fontsize=7.5, frameon=False)
fig.suptitle(f"Per-UAV risk & AoI by flight phase — Route A vs B, per policy (K={K}, C={C}, steady/N=C)",
             fontsize=10, fontweight="bold")

OUT = os.path.join(DATA, "plots")
b = os.path.join(OUT, f"fig_aoi_routeAB_bypolicy_K{K}_C{C}")
for ext in ("pdf", "png"):
    fig.savefig(f"{b}.{ext}", dpi=150, bbox_inches="tight")
    print("saved", f"{b}.{ext}")
for p in POL:
    for ph in PHASES:
        a, bb = DATAph[p]["A"][ph], DATAph[p]["B"][ph]
        print(f"  {LABEL[p]:11} {ph:8}: C2 A={a[0]:6.1f} B={bb[0]:6.1f} | vid A={a[1]:7.1f} B={bb[1]:7.1f}")
