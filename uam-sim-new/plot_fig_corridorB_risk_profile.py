#!/usr/bin/env python3
"""plot_fig_corridorB_risk_profile.py -- deep-dive: spatial risk profile along
corridor B (Event venue <-> Iguatemi Mall), one (K, C) cell.

For every corridor-B UAV (both directions, ids identified by flight length),
each steady-window slot sample R_u(t) is mapped to its position along the
corridor (constant speed, entry->exit) and pooled into distance bins:
  R_bar_u(x) = per-UAV MEAN risk within the bin (same semantics as the TLS
  cell metric p99[R_bar_u], spatially resolved — not instantaneous samples):
  panel (a): fleet mean of R_bar_u(x)   panel (b): fleet p99 of R_bar_u(x)
x = distance from the Event venue end (hotspot side) toward Iguatemi Mall.

Steady window follows util/collect_results.m steady_window(): contiguous span
with >= C UAVs airborne (warm-up ramp and drain tail dropped).

Usage:
  python3 plot_fig_corridorB_risk_profile.py --K 16 --C 80 --dir csv_export_RC12 \
      --pol round-robin-sw,pf-classic-sw,mw-sw-v8,mw-sw-v9 [--routes both]
Output: <dir>/plots/fig_corridorB_risk_profile_K{K}_C{C}.{pdf,png}
"""
import os, glob, argparse
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

ap = argparse.ArgumentParser()
ap.add_argument("--dir", default="csv_export_RC12")
ap.add_argument("--scenario", default="4r_b7")
ap.add_argument("--K", type=int, default=16)
ap.add_argument("--C", type=int, default=80)
ap.add_argument("--q", type=float, default=0.99)
ap.add_argument("--bin", type=float, default=50.0, help="distance bin width [m]")
ap.add_argument("--routes", choices=["both", "to-event", "from-event"], default="both",
                help="which corridor-B direction(s) to pool (both = bidirectional traffic)")
ap.add_argument("--hs-zone", default="0:2843",
                help="hotspot-zone extent 'lo:hi' [m from Event venue] for the shaded band "
                     "(RC12 grid: 0:2843; RC13 shortened route: 300:2661)")
ap.add_argument("--no-shade", action="store_true",
                help="omit the shaded hotspot band (suffix _noshade)")
ap.add_argument("--start-label", default="Iguatemi Mall",
                help="label of the corridor-B start endpoint (RC13 moved route: 'Vertiport')")
ap.add_argument("--blen", type=float, default=None,
                help="corridor-B length [m]; default = RC12 geometry (4264.4). "
                     "RC13 shortened route: 3261.0")
ap.add_argument("--pol", default="round-robin-sw,pf-classic-sw,mw-sw-v8,mw-sw-v9")
args = ap.parse_args()

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(SCRIPT_DIR, args.dir)
SCEN, K, C = args.scenario, args.K, args.C
BASE = os.path.join(DATA, f"K{K}", f"C{C:02d}", SCEN)

# Corridor B geometry (experiment 4r_b7: rBhalf start [-350,9750] -> goal [-700,14000];
# rBR is the exact reverse). Goal end (E=14000) carries the 24 Gaussian hotspots
# = Event venue; start end (E=9750) = Iguatemi Mall.
ROUTE_LEN = args.blen if args.blen else float(np.hypot(-700 - (-350), 14000 - 9750))
HS_LO, HS_END = (float(v) for v in args.hs_zone.split(":"))   # shaded hotspot band

POL    = [p for p in args.pol.split(",") if p]
LABEL  = {"round-robin-sw": "RR", "pf-classic-sw": "PF", "mr-mw-ch-sw": "Risk-Aware",
          "max-risk-pf-sw": "Risk-PF", "mw-sw-otimo-ch": "MW-opt (sw)",
          "mw-otimo-ch": "MW-opt", "mw-sw-v8": "MW-V8", "mw-sw-v9": "MW-V9", "mw-v9-nsw": "MW-V9-nsw"}
COLORS = {"round-robin-sw": "#F9A825", "pf-classic-sw": "#E53935", "mr-mw-ch-sw": "#8E24AA",
          "max-risk-pf-sw": "#1E88E5", "mw-sw-otimo-ch": "#3949AB",
          "mw-otimo-ch": "#6D4C41", "mw-sw-v8": "#00897B", "mw-sw-v9": "#7CB342", "mw-v9-nsw": "#33691E"}

def steady_window(ds):
    """[t0, t1] = contiguous span with >= C UAVs airborne (collect_results.m)."""
    t_max = int(ds.exit_slot.max()) + 1
    delta = np.zeros(t_max + 1)
    np.add.at(delta, ds.entry_slot.astype(int).values, 1)
    np.add.at(delta, ds.exit_slot.astype(int).values, -1)
    airborne = np.cumsum(delta)[:-1]
    full = np.flatnonzero(airborne >= C)
    if len(full) == 0:
        raise SystemExit(f"steady window empty: max airborne {airborne.max():.0f} < C={C}")
    return int(full[0]), int(full[-1])

def load(pol):
    ds = pd.read_csv(sorted(glob.glob(os.path.join(BASE, f"{pol}_*_drone_summary.csv")))[-1])
    t0, t1 = steady_window(ds)

    # corridor-B UAVs: flight length ~ROUTE_LEN (route A is 8000 m); ids are
    # contiguous per route block, first half = to-event (rBhalf), second = rBR
    ds["dur"] = ds.exit_slot - ds.entry_slot
    b = ds[ds.dur < 0.7 * ds.dur.max()].sort_values("drone_id")
    # block boundary between rBhalf and rBR: within a route block entry_slot is
    # increasing (regular arrival spacing); it resets at the next block. Robust
    # to unequal block sizes (e.g. C=5 -> 1+2 per B direction).
    ent = b.entry_slot.values
    resets = np.flatnonzero(np.diff(ent) < 0)
    cut = (resets[0] + 1) if len(resets) else len(b) // 2
    ids_to   = b.drone_id.values[:cut]               # Iguatemi -> Event venue
    ids_from = b.drone_id.values[cut:]               # Event venue -> Iguatemi
    keep = {"to-event": set(ids_to), "from-event": set(ids_from),
            "both": set(ids_to) | set(ids_from)}[args.routes]

    ss = pd.read_csv(sorted(glob.glob(os.path.join(BASE, f"{pol}_*_slot_series.csv")))[-1],
                     usecols=["slot", "drone_id", "r_tot", "h1_slots", "h2_slots"])
    ss = ss[ss.drone_id.isin(keep) & ss.slot.between(t0, t1)]
    ss = ss.merge(ds[["drone_id", "entry_slot", "dur"]], on="drone_id")

    trav = (ss.slot - ss.entry_slot) / ss.dur * ROUTE_LEN     # distance travelled
    to_event = ss.drone_id.isin(set(ids_to))
    ss["dist"] = np.where(to_event, ROUTE_LEN - trav, trav)   # distance from Event venue
    print(f"  {LABEL.get(pol,pol):7}: steady [{t0}, {t1}], {ss.drone_id.nunique()} UAVs, "
          f"{len(ss)} samples")
    return ss

edges = np.arange(0.0, ROUTE_LEN + args.bin, args.bin)
centers = 0.5 * (edges[:-1] + edges[1:])

prof = {}
for p in POL:
    d = load(p)
    d = d.assign(aoi=(d.h1_slots + d.h2_slots) / 2.0,
                 bin=pd.cut(d.dist, edges, labels=False))
    # per-UAV mean within each bin, THEN fleet statistics — same semantics as
    # the TLS cell metric p99[R_bar_u] (percentile over UAVs of per-UAV mean
    # risk), spatially resolved; NOT a percentile over instantaneous samples.
    gu = d.groupby(["bin", "drone_id"], observed=True)[["r_tot", "aoi"]].mean()
    gb = gu.groupby(level=0)
    prof[p] = (gb.r_tot.mean().reindex(range(len(centers))),
               gb.r_tot.quantile(args.q).reindex(range(len(centers))),
               gb.aoi.mean().reindex(range(len(centers))))

plt.rcParams.update({"font.family": "serif",
                     "font.serif": ["Times New Roman", "Nimbus Roman", "DejaVu Serif"],
                     "mathtext.fontset": "stix", "font.size": 9, "pdf.fonttype": 42})
fig, (ax0, ax1, ax2) = plt.subplots(3, 1, figsize=(6.4, 6.6), sharex=True,
                                    constrained_layout=True)

for p in POL:
    m, q, a = prof[p]
    ax0.plot(centers, m, color=COLORS[p], lw=1.8)
    ax1.plot(centers, q, color=COLORS[p], lw=1.8)
    ax2.plot(centers, a, color=COLORS[p], lw=1.8)

for ax, tag, ylab in ((ax0, "(a)", r"fleet-mean $\bar R_u(x)$"),
                      (ax1, "(b)", rf"$p_{{{int(round(100*args.q))}}}\,[\bar R_u(x)]$ across UAVs"),
                      (ax2, "(c)", r"mean total AoI $\frac{1}{2}(h_{u,s}+h_{u,v})$ (slots)")):
    ax.set_yscale("log")
    if not args.no_shade:
        ax.axvspan(HS_LO, HS_END, color="0.80", alpha=0.5, lw=0, zorder=0)
    ax.grid(True, which="both", ls=":", lw=0.4, alpha=0.6)
    ax.set_ylabel(ylab)
    ax.text(0.02, 0.93, tag, transform=ax.transAxes, fontweight="bold", va="top")
for ax in (ax0, ax1):
    ax.axhline(1.0, ls=(0, (4, 3)), lw=1.0, color="0.35")
if not args.no_shade:
    ax0.text(HS_END, ax0.get_ylim()[1], "hotspot zone ", color="0.35", fontsize=7,
             va="top", ha="right")

# Flight phases (altProfile=takeoff_landing, takeoffDist=500 m). With both
# directions pooled the first/last 500 m mix landing (one direction) with
# takeoff (the other); single-direction plots get the proper phase names.
TO = 500.0
phase_ev, phase_ig = {"both":       ("takeoff /\nlanding", "takeoff /\nlanding"),
                      "to-event":   ("landing",            "takeoff"),
                      "from-event": ("takeoff",            "landing")}[args.routes]
for ax in (ax0, ax1, ax2):
    for xb in (TO, ROUTE_LEN - TO):
        ax.axvline(xb, color="0.45", lw=0.8, ls="-.", alpha=0.8, zorder=1)
for xc, name in [(TO / 2, phase_ev), (ROUTE_LEN / 2, "cruise"),
                 (ROUTE_LEN - TO / 2, phase_ig)]:
    ax2.text(xc, 0.03, name, transform=ax2.get_xaxis_transform(), ha="center",
             va="bottom", fontsize=6.5, color="0.4", style="italic")

# flight-direction arrow (single-direction plots): to-event flies right->left
# on this axis (Iguatemi at x=L toward the venue at x=0), from-event the reverse
if args.routes != "both":
    x0, x1 = (0.60, 0.40) if args.routes == "to-event" else (0.40, 0.60)
    ax0.annotate("", xy=(x1, 0.90), xytext=(x0, 0.90), xycoords="axes fraction",
                 textcoords="axes fraction",
                 arrowprops=dict(arrowstyle="-|>", color="0.30", lw=1.4))
    ax0.text(0.50, 0.925, "flight direction", transform=ax0.transAxes,
             ha="center", va="bottom", fontsize=7, color="0.30", style="italic")
ax0.text(0.99, 1.0, "TLS ($R_u = 1$)", transform=ax0.get_yaxis_transform(),
         ha="right", va="bottom", fontsize=7, color="0.35")

ax2.set_xlim(0, ROUTE_LEN)
ax2.set_xlabel("Position along corridor B  [m]")
for x, name, ha in ((0, "Event venue", "left"), (ROUTE_LEN, args.start_label, "right")):
    ax2.annotate(name, xy=(x, 0), xycoords=("data", "axes fraction"),
                 xytext=(0, -30), textcoords="offset points",
                 ha=ha, va="top", fontsize=8, style="italic", color="0.25")

handles = [Line2D([0], [0], color=COLORS[p], lw=2, label=LABEL.get(p, p)) for p in POL]
fig.legend(handles=handles, ncol=len(POL), loc="lower center",
           bbox_to_anchor=(0.5, -0.10), frameon=False, fontsize=9)

OUT = os.path.join(DATA, "plots")
os.makedirs(OUT, exist_ok=True)
suff = {"both": "", "to-event": "_toevent", "from-event": "_fromevent"}[args.routes]
if args.no_shade:
    suff += "_noshade"
b = os.path.join(OUT, f"fig_corridorB_risk_profile_K{K}_C{C}{suff}")
for ext in ("pdf", "png"):
    fig.savefig(f"{b}.{ext}", dpi=200, bbox_inches="tight")
    print("saved", f"{b}.{ext}")

print(f"\ncorridor-wide pooled stats ({args.routes}):")
for p in POL:
    m, q, a = prof[p]
    print(f"  {LABEL.get(p,p):7}: mean(profile)={np.nanmean(m):8.3f}  "
          f"peak mean={np.nanmax(m):8.3f}  peak p{int(100*args.q)}={np.nanmax(q):8.3f}  "
          f"mean AoI={np.nanmean(a):7.1f}")
