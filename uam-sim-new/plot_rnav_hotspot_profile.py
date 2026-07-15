#!/usr/bin/env python3
"""plot_rnav_hotspot_profile.py -- R_Nav (disk overlap) over the hotspot.

Analog of plot_ru_hotspot_profile.py for the navigation component: where
along corridor B do uncertainty disks actually overlap, per policy?
r_unc is mostly exactly 0, so the profile splits into occurrence and
magnitude:
  top    : share of slots with any overlap (r_unc > 0) per 50 m bin [%]
  bottom : bin mean of r_unc (log scale, display floor annotated)
Table: crowd zone vs off-crowd vs route A -- % slots with overlap,
mean / p99 / max of r_unc (normalized; design envelope = 1).

Usage: python3 plot_rnav_hotspot_profile.py [--dir csv_export_RC14 --K 8 --C 60]
Output: <dir>/plots/fig_rnav_hotspot_K{K}_C{C}.{pdf,png} + printed table.
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
ap.add_argument("--update-rate", type=float, default=7.0)
ap.add_argument("--hs-end", type=float, default=2843.0)
ap.add_argument("--floor", type=float, default=1e-6,
                help="display floor for the log magnitude panel")
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
                     usecols=["slot", "drone_id", "r_unc"])
    ss = ss.merge(ds[["drone_id", "entry_slot", "dur"]], on="drone_id")
    ss["is_corr"] = ss.drone_id.isin(set(b.drone_id))
    trav = (ss.slot - ss.entry_slot) / ss.dur * ROUTE_LEN
    ss["dist"] = np.where(ss.drone_id.isin(ids_to), ROUTE_LEN - trav, trav)
    return ss

QP = int(100 * args.q)
prof = {}
print(f"\nR_nav (r_unc, normalizado; envelope = 1) -- K={args.K}, N={args.C}, "
      f"multidão = dist <= {args.hs_end:.0f} m no corredor B\n")
print(f"| política | zona | % slots c/ overlap | média | p{QP} | máx |")
print("|" + "---|" * 6)
for p in POL:
    ss = load(p)
    zones = (("multidão", ss.is_corr & (ss.dist <= args.hs_end)),
             ("fora (corr. B)", ss.is_corr & (ss.dist > args.hs_end)),
             ("rota A", ~ss.is_corr))
    for zone, m in zones:
        r = ss.r_unc[m]
        print(f"| {LABEL[p]} | {zone} | {100.0 * (r > 0).mean():.2f} | "
              f"{r.mean():.2e} | {r.quantile(args.q):.2e} | {r.max():.2e} |")
    sb = ss[ss.is_corr]
    edges = np.arange(0.0, ROUTE_LEN + args.bin, args.bin)
    g = sb.groupby(pd.cut(sb.dist, edges, labels=False), observed=True).r_unc
    prof[p] = (edges[:-1] + args.bin / 2,
               100.0 * g.apply(lambda x: (x > 0).mean()), g.mean())

plt.rcParams.update({"font.family": "serif", "font.size": 9,
                     "mathtext.fontset": "stix", "pdf.fonttype": 42})
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(6.4, 5.2), sharex=True,
                               constrained_layout=True)
for p in POL:
    x, occ, mag = prof[p]
    ax1.plot(x, occ, color=COLORS[p], lw=1.5, label=LABEL[p])
    ax2.plot(x, np.maximum(mag, args.floor), color=COLORS[p], lw=1.5)
for ax in (ax1, ax2):
    ax.axvspan(0, args.hs_end, color="0.85", zorder=0)
    ax.grid(True, which="both", ls=":", lw=0.35, alpha=0.5)
ax1.set_ylabel("slots com overlap [%]")
ax1.text(args.hs_end / 2, ax1.get_ylim()[1], "zona da multidão",
         ha="center", va="top", fontsize=8, color="0.35")
ax1.legend(ncol=4, fontsize=8, frameon=False, loc="upper left")
ax2.set_yscale("log")
ax2.set_ylabel(r"média por bin de $\tilde R_{\mathrm{Nav}}$")
ax2.axhline(args.floor, ls=":", lw=0.8, color="0.5")
ax2.text(0, args.floor, f" piso de exibição {args.floor:g} (=0)",
         fontsize=7, color="0.5", va="bottom")
ax2.set_xlabel("distância ao início do corredor B [m]")
fig.suptitle(rf"$\tilde R_{{\mathrm{{Nav}}}}$ ao longo do corredor B -- "
             f"K={args.K}, N={args.C}", fontsize=11, fontweight="bold")

out = os.path.join(SCRIPT_DIR, args.dir, "plots",
                   f"fig_rnav_hotspot_K{args.K}_C{args.C:02d}")
for ext in ("pdf", "png"):
    fig.savefig(f"{out}.{ext}", dpi=200, bbox_inches="tight")
    print("saved", f"{out}.{ext}")
