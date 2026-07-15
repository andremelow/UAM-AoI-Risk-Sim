#!/usr/bin/env python3
"""plot_fig_corridorB_critical1_3panel.py -- the single most critical UAV of
each policy, 3-panel layout matching fig_corridorB_risk_profile:
  (a) R_u(x)  (b) C2 AoI h_{u,s}(x)  (c) video AoI h_{u,v}(x)
Each curve = per-50m-bin mean along that UAV's own flight; UAV chosen as the
corridor-B drone with the highest flight-average risk R_bar_u (target metric).

Usage:
  python3 plot_fig_corridorB_critical1_3panel.py --dir csv_export_RC14 --K 16 --C 80
Output: <dir>/plots/fig_corridorB_critical_uav1_K{K}_C{C}.{pdf,png}
"""
import os, glob, argparse
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

ap = argparse.ArgumentParser()
ap.add_argument("--dir", default="csv_export_RC14")
ap.add_argument("--scenario", default="4r_b7")
ap.add_argument("--K", type=int, default=16)
ap.add_argument("--C", type=int, default=80)
ap.add_argument("--bin", type=float, default=50.0)
ap.add_argument("--blen", type=float, default=None)
ap.add_argument("--no-shade", action="store_true")
ap.add_argument("--hs-zone", default="0:2843")
ap.add_argument("--start-label", default="Iguatemi Mall")
ap.add_argument("--uid", type=int, default=None,
                help="fix the drone id (same flight under every policy) instead of "
                     "each policy's own worst")
ap.add_argument("--stat", choices=["mean", "p99"], default="mean",
                help="panel (a) statistic within each bin of the UAV's own samples")
ap.add_argument("--rlim", default=None,
                help="y-limits for panel (a) as 'lo:hi', e.g. 0.1:100 (clips the terminal plunge)")
ap.add_argument("--pol", default="round-robin-sw,pf-classic-sw,mw-sw-v8,mw-sw-v9")
args = ap.parse_args()

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.join(SCRIPT_DIR, args.dir, f"K{args.K}", f"C{args.C:02d}", args.scenario)
ROUTE_LEN = args.blen if args.blen else float(np.hypot(-700 - (-350), 14000 - 9750))
HS_LO, HS_END = (float(v) for v in args.hs_zone.split(":"))

POL    = [p for p in args.pol.split(",") if p]
LABEL  = {"round-robin-sw": "RR", "pf-classic-sw": "PF", "mw-sw-v8": "MW-V8", "mw-sw-v9": "MW-V9"}
COLORS = {"round-robin-sw": "#F9A825", "pf-classic-sw": "#E53935",
          "mw-sw-v8": "#00897B", "mw-sw-v9": "#7CB342"}

edges = np.arange(0.0, ROUTE_LEN + args.bin, args.bin)
centers = 0.5 * (edges[:-1] + edges[1:])

plt.rcParams.update({"font.family": "serif",
                     "font.serif": ["Times New Roman", "Nimbus Roman", "DejaVu Serif"],
                     "mathtext.fontset": "stix", "font.size": 9, "pdf.fonttype": 42})
fig, (ax0, ax1, ax2) = plt.subplots(3, 1, figsize=(6.4, 6.6), sharex=True,
                                    constrained_layout=True)
handles = []
for pol in POL:
    ds = pd.read_csv(sorted(glob.glob(os.path.join(BASE, f"{pol}_*_drone_summary.csv")))[-1])
    ds["dur"] = ds.exit_slot - ds.entry_slot
    b = ds[ds.dur < 0.7 * ds.dur.max()].sort_values("drone_id")
    ent = b.entry_slot.values
    resets = np.flatnonzero(np.diff(ent) < 0)
    cut = (resets[0] + 1) if len(resets) else len(b) // 2
    ids_to = set(b.drone_id.values[:cut])

    if args.uid is not None:
        worst = b[b.drone_id == args.uid].iloc[0]
    else:
        worst = b.sort_values("mean_r_tot", ascending=False).iloc[0]
    uid, rbar = int(worst.drone_id), worst.mean_r_tot
    ss = pd.read_csv(sorted(glob.glob(os.path.join(BASE, f"{pol}_*_slot_series.csv")))[-1],
                     usecols=["slot", "drone_id", "r_tot", "h1_slots", "h2_slots"])
    d = ss[ss.drone_id == uid].sort_values("slot")
    trav = (d.slot - worst.entry_slot) / worst.dur * ROUTE_LEN
    dist = (ROUTE_LEN - trav) if uid in ids_to else trav
    g = d.assign(dist=dist).groupby(pd.cut(dist, edges, labels=False), observed=True)

    c = COLORS[pol]
    r_stat = g.r_tot.mean() if args.stat == "mean" else g.r_tot.quantile(0.99)
    ax0.plot(centers, r_stat.reindex(range(len(centers))),           color=c, lw=1.8)
    ax1.plot(centers, g.h1_slots.mean().reindex(range(len(centers))), color=c, lw=1.8)
    ax2.plot(centers, g.h2_slots.mean().reindex(range(len(centers))), color=c, lw=1.8)
    arrow = "→venue" if uid in ids_to else "venue→"
    handles.append(Line2D([0], [0], color=c, lw=2,
                          label=rf"{LABEL[pol]}: #{uid} {arrow}, $\bar R_u$={rbar:.2f}"))

ylab0 = r"$R_u(x)$" if args.stat == "mean" else r"$p_{99}[R_u(t)]$ per bin"
for ax, tag, ylab in ((ax0, "(a)", ylab0),
                      (ax1, "(b)", r"C2 AoI $h_{u,s}(x)$ (slots)"),
                      (ax2, "(c)", r"video AoI $h_{u,v}(x)$ (slots)")):
    ax.set_yscale("log")
    if not args.no_shade:
        ax.axvspan(HS_LO, HS_END, color="0.80", alpha=0.5, lw=0, zorder=0)
    for xb in (500.0, ROUTE_LEN - 500.0):
        ax.axvline(xb, color="0.45", lw=0.8, ls="-.", alpha=0.8)
    ax.grid(True, which="both", ls=":", lw=0.4, alpha=0.6)
    ax.set_ylabel(ylab)
    ax.text(0.02, 0.93, tag, transform=ax.transAxes, fontweight="bold", va="top")
if args.rlim:
    lo, hi = (float(v) for v in args.rlim.split(":"))
    ax0.set_ylim(lo, hi)
ax0.axhline(1.0, ls=(0, (4, 3)), lw=1.0, color="0.35")
ax0.text(0.99, 1.0, "TLS ($R_u = 1$)", transform=ax0.get_yaxis_transform(),
         ha="right", va="bottom", fontsize=7, color="0.35")

ax2.set_xlim(0, ROUTE_LEN)
ax2.set_xlabel("Position along corridor B  [m]")
for x, name, ha in ((0, "Event venue", "left"), (ROUTE_LEN, args.start_label, "right")):
    ax2.annotate(name, xy=(x, 0), xycoords=("data", "axes fraction"),
                 xytext=(0, -28), textcoords="offset points",
                 ha=ha, va="top", fontsize=8, style="italic", color="0.25")
fig.legend(handles=handles, ncol=2, loc="lower center", bbox_to_anchor=(0.5, -0.06),
           frameon=False, fontsize=8)
title = (f"UAV #{args.uid} under each policy — K={args.K}, N={args.C}"
         if args.uid is not None else
         f"Most critical UAV of each policy — K={args.K}, N={args.C}")
fig.suptitle(title, fontsize=10, fontweight="bold")

OUT = os.path.join(SCRIPT_DIR, args.dir, "plots")
os.makedirs(OUT, exist_ok=True)
suff = ("_p99" if args.stat == "p99" else "") + ("_noshade" if args.no_shade else "")
uidtag = f"_u{args.uid}" if args.uid is not None else ""
base = os.path.join(OUT, f"fig_corridorB_critical_uav1_K{args.K}_C{args.C}{uidtag}{suff}")
for ext in ("pdf", "png"):
    fig.savefig(f"{base}.{ext}", dpi=200, bbox_inches="tight")
    print("saved", f"{base}.{ext}")
