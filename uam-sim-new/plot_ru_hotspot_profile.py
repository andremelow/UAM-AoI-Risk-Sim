#!/usr/bin/env python3
"""plot_ru_hotspot_profile.py -- uncertainty-disk radius r_u over the hotspot.

Where does each policy let the C2 uncertainty disk grow? Splits corridor-B
flight samples into crowd zone (dist <= hs_end) vs off-crowd and reports:
  - pooled r_u stats per zone (median / p99 / max, % slots blind r_u > W_c/2)
  - per-UAV max r_u over the crowd (median | p99 across UAVs -- no pooling)
Figure: r_u(s) profile along corridor B (bin median + p99), crowd zone
shaded, W_c/2 containment line.

Usage: python3 plot_ru_hotspot_profile.py [--dir csv_export_RC14 --K 8 --C 60]
Output: <dir>/plots/fig_ru_hotspot_K{K}_C{C}.{pdf,png} + printed table.
"""
import os, glob, argparse
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ap = argparse.ArgumentParser()
ap.add_argument("--dir", default="csv_export_RC14")
ap.add_argument("--scenario", default="4r_b7")
ap.add_argument("--K", type=int, default=8)
ap.add_argument("--C", type=int, default=60)
ap.add_argument("--pol", default="round-robin-sw,pf-classic-sw,mw-sw-v8,mw-sw-v9")
ap.add_argument("--q", type=float, default=0.99)
ap.add_argument("--bin", type=float, default=50.0)
ap.add_argument("--r-min", type=float, default=10.0)
ap.add_argument("--update-rate", type=float, default=7.0)
ap.add_argument("--wc", type=float, default=100.0)
ap.add_argument("--hs-end", type=float, default=2843.0)
args = ap.parse_args()

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROUTE_LEN = float(np.hypot(-700 - (-350), 14000 - 9750))
LABEL = {"round-robin-sw": "RR", "pf-classic-sw": "PF",
         "mw-sw-v8": "MW-V8", "mw-sw-v9": "MW-V9"}
COLORS = {"round-robin-sw": "#F9A825", "pf-classic-sw": "#E53935",
          "mw-sw-v8": "#00897B", "mw-sw-v9": "#7CB342"}
POL = [p for p in args.pol.split(",") if p]

def load(pol):
    base = os.path.join(SCRIPT_DIR, args.dir, f"K{args.K}", f"C{args.C:02d}",
                        args.scenario)
    ds = pd.read_csv(sorted(glob.glob(os.path.join(base, f"{pol}_*_drone_summary.csv")))[-1])
    ds["dur"] = ds.exit_slot - ds.entry_slot
    b = ds[ds.dur < 0.7 * ds.dur.max()].sort_values("drone_id")
    ent = b.entry_slot.values
    resets = np.flatnonzero(np.diff(ent) < 0)
    cut = (resets[0] + 1) if len(resets) else len(b) // 2
    ids_to = set(b.drone_id.values[:cut])

    ss = pd.read_csv(sorted(glob.glob(os.path.join(base, f"{pol}_*_slot_series.csv")))[-1],
                     usecols=["slot", "drone_id", "h1_slots"])
    ss = ss[ss.drone_id.isin(set(b.drone_id))]
    ss = ss.merge(ds[["drone_id", "entry_slot", "dur"]], on="drone_id")
    v = float(np.median(ROUTE_LEN / (b.dur.values / args.update_rate)))
    ss["r_u"] = args.r_min + (v / args.update_rate) * ss.h1_slots.values
    trav = (ss.slot - ss.entry_slot) / ss.dur * ROUTE_LEN
    ss["dist"] = np.where(ss.drone_id.isin(ids_to), ROUTE_LEN - trav, trav)
    return ss

QP = int(100 * args.q)
rows, prof = [], {}
for p in POL:
    ss = load(p)
    crowd = ss.dist <= args.hs_end
    for zone, m in (("multidão", crowd), ("fora", ~crowd)):
        r = ss.r_u[m]
        rows.append(dict(pol=LABEL[p], zone=zone, med=r.median(),
                         pq=r.quantile(args.q), mx=r.max(),
                         blind=100.0 * (r > args.wc / 2).mean()))
    # per-UAV max over crowd (no pooling across UAVs)
    gu = ss[crowd].groupby("drone_id").r_u.max()
    rows[-2]["uav_med"] = gu.median()
    rows[-2]["uav_pq"] = gu.quantile(args.q)
    edges = np.arange(0.0, ROUTE_LEN + args.bin, args.bin)
    g = ss.groupby(pd.cut(ss.dist, edges, labels=False), observed=True).r_u
    prof[p] = (edges[:-1] + args.bin / 2, g.median(), g.quantile(args.q))
    print(f"done {LABEL[p]}")

print(f"\nr_u sobre hotspot vs fora -- K={args.K}, N={args.C}, "
      f"W_c/2 = {args.wc/2:.0f} m, multidão = dist <= {args.hs_end:.0f} m\n")
print(f"| política | zona | r_u med [m] | r_u p{QP} [m] | r_u máx [m] | "
      f"% slots cegos | máx/UAV med | máx/UAV p{QP} |")
print("|" + "---|" * 8)
for r in rows:
    extra = (f" {r['uav_med']:.0f} | {r['uav_pq']:.0f} |"
             if "uav_med" in r else " — | — |")
    print(f"| {r['pol']} | {r['zone']} | {r['med']:.0f} | {r['pq']:.0f} | "
          f"{r['mx']:.0f} | {r['blind']:.1f} |" + extra)

plt.rcParams.update({"font.family": "serif", "font.size": 9,
                     "mathtext.fontset": "stix", "pdf.fonttype": 42})
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(6.4, 5.2), sharex=True,
                               constrained_layout=True)
for ax, idx, ttl in ((ax1, 1, "mediana por bin"),
                     (ax2, 2, f"p{QP} por bin")):
    for p in POL:
        x, med, pq = prof[p]
        ax.plot(x, med if idx == 1 else pq,
                color=COLORS[p], lw=1.5, label=LABEL[p])
    ax.axvspan(0, args.hs_end, color="0.85", zorder=0)
    ax.axhline(args.wc / 2, ls="--", lw=1.0, color="k")
    ax.set_yscale("log")
    ax.set_ylabel(f"$r_u$ [m] ({ttl})")
    ax.grid(True, which="both", ls=":", lw=0.35, alpha=0.5)
ax1.text(args.hs_end / 2, ax1.get_ylim()[1], "zona da multidão",
         ha="center", va="top", fontsize=8, color="0.35")
ax2.text(ROUTE_LEN, args.wc / 2, " $W_c/2$", va="bottom", ha="right", fontsize=8)
ax2.set_xlabel("distância ao início do corredor B [m]")
ax1.legend(ncol=4, fontsize=8, frameon=False, loc="upper right")
fig.suptitle(f"$r_u$ ao longo do corredor B -- K={args.K}, N={args.C}",
             fontsize=11, fontweight="bold")

out = os.path.join(SCRIPT_DIR, args.dir, "plots",
                   f"fig_ru_hotspot_K{args.K}_C{args.C:02d}")
for ext in ("pdf", "png"):
    fig.savefig(f"{out}.{ext}", dpi=200, bbox_inches="tight")
    print("saved", f"{out}.{ext}")
