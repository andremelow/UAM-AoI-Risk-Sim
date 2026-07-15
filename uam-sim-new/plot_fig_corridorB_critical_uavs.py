#!/usr/bin/env python3
"""plot_fig_corridorB_critical_uavs.py -- per-policy top-N most critical UAVs.

One panel per policy; each shows the along-corridor risk trace (per-50m-bin
mean of R_u(t)) of the N corridor-B UAVs with the highest flight-average risk
R_bar_u (the TLS target metric). Solid = flies toward the venue (lands there),
dashed = away. Legend gives drone id, direction and flight R_bar_u.

Usage:
  python3 plot_fig_corridorB_critical_uavs.py --dir csv_export_RC14 --K 16 --C 80
Output: <dir>/plots/fig_corridorB_critical_uavs_K{K}_C{C}.{pdf,png}
"""
import os, glob, argparse
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ap = argparse.ArgumentParser()
ap.add_argument("--dir", default="csv_export_RC14")
ap.add_argument("--scenario", default="4r_b7")
ap.add_argument("--K", type=int, default=16)
ap.add_argument("--C", type=int, default=80)
ap.add_argument("--n", type=int, default=3, help="UAVs per policy")
ap.add_argument("--bin", type=float, default=50.0)
ap.add_argument("--blen", type=float, default=None)
ap.add_argument("--hs-zone", default="0:2843")
ap.add_argument("--no-shade", action="store_true")
ap.add_argument("--start-label", default="Iguatemi Mall")
ap.add_argument("--pol", default="round-robin-sw,pf-classic-sw,mw-sw-v8,mw-sw-v9")
args = ap.parse_args()

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.join(SCRIPT_DIR, args.dir, f"K{args.K}", f"C{args.C:02d}", args.scenario)
ROUTE_LEN = args.blen if args.blen else float(np.hypot(-700 - (-350), 14000 - 9750))
HS_LO, HS_END = (float(v) for v in args.hs_zone.split(":"))

POL   = [p for p in args.pol.split(",") if p]
LABEL = {"round-robin-sw": "RR", "pf-classic-sw": "PF", "mw-sw-v8": "MW-V8", "mw-sw-v9": "MW-V9"}
TITLE_COL = {"round-robin-sw": "#F9A825", "pf-classic-sw": "#E53935",
             "mw-sw-v8": "#00897B", "mw-sw-v9": "#7CB342"}
SHADES = ["#B71C1C", "#1E88E5", "#6D4C41"]           # per-UAV colors within a panel

edges = np.arange(0.0, ROUTE_LEN + args.bin, args.bin)
centers = 0.5 * (edges[:-1] + edges[1:])

plt.rcParams.update({"font.family": "serif",
                     "font.serif": ["Times New Roman", "Nimbus Roman", "DejaVu Serif"],
                     "mathtext.fontset": "stix", "font.size": 9, "pdf.fonttype": 42})
fig, axes = plt.subplots(2, 2, figsize=(7.2, 5.4), sharex=True, sharey=True,
                         constrained_layout=True)

for ax, pol in zip(axes.flat, POL):
    ds = pd.read_csv(sorted(glob.glob(os.path.join(BASE, f"{pol}_*_drone_summary.csv")))[-1])
    ds["dur"] = ds.exit_slot - ds.entry_slot
    b = ds[ds.dur < 0.7 * ds.dur.max()].sort_values("drone_id")
    ent = b.entry_slot.values
    resets = np.flatnonzero(np.diff(ent) < 0)
    cut = (resets[0] + 1) if len(resets) else len(b) // 2
    ids_to = set(b.drone_id.values[:cut])            # flies toward the venue

    top = b.sort_values("mean_r_tot", ascending=False).head(args.n)
    ss = pd.read_csv(sorted(glob.glob(os.path.join(BASE, f"{pol}_*_slot_series.csv")))[-1],
                     usecols=["slot", "drone_id", "r_tot"])
    ss = ss[ss.drone_id.isin(set(top.drone_id))].merge(
            ds[["drone_id", "entry_slot", "dur"]], on="drone_id")
    trav = (ss.slot - ss.entry_slot) / ss.dur * ROUTE_LEN
    ss["dist"] = np.where(ss.drone_id.isin(ids_to), ROUTE_LEN - trav, trav)

    for (uid, row), col in zip(top.set_index("drone_id").iterrows(), SHADES):
        d = ss[ss.drone_id == uid]
        prof = d.groupby(pd.cut(d.dist, edges, labels=False), observed=True) \
                .r_tot.mean().reindex(range(len(centers)))
        to_ev = uid in ids_to
        ax.plot(centers, prof, color=col, lw=1.5, ls="-" if to_ev else (0, (4, 2)),
                label=f"#{uid} {'→venue' if to_ev else 'venue→'}  "
                      rf"$\bar R_u$={row.mean_r_tot:.2f}")

    ax.set_yscale("log")
    if not args.no_shade:
        ax.axvspan(HS_LO, HS_END, color="0.80", alpha=0.5, lw=0, zorder=0)
    ax.axhline(1.0, ls=(0, (4, 3)), lw=1.0, color="0.35")
    for xb in (500.0, ROUTE_LEN - 500.0):
        ax.axvline(xb, color="0.45", lw=0.7, ls="-.", alpha=0.7)
    ax.grid(True, which="both", ls=":", lw=0.4, alpha=0.5)
    ax.set_title(LABEL.get(pol, pol), fontsize=9.5, color=TITLE_COL.get(pol, "k"),
                 fontweight="bold", loc="left", pad=3)
    ax.legend(loc="lower center", fontsize=6.3, frameon=False, handlelength=1.7,
              labelspacing=0.25, borderaxespad=0.15)

for ax in axes[1]:
    ax.set_xlim(0, ROUTE_LEN)
    ax.set_xlabel("Position along corridor B  [m]")
for ax in axes[:, 0]:
    ax.set_ylabel(r"$R_u(x)$ (per-UAV bin mean)")
for x, name, ha in ((0, "Event venue", "left"), (ROUTE_LEN, args.start_label, "right")):
    axes[1, 0].annotate(name if ha == "left" else "", xy=(x, 0),
                        xycoords=("data", "axes fraction"), xytext=(0, -28),
                        textcoords="offset points", ha=ha, va="top", fontsize=8,
                        style="italic", color="0.25")
    axes[1, 1].annotate(name if ha == "right" else "", xy=(x, 0),
                        xycoords=("data", "axes fraction"), xytext=(0, -28),
                        textcoords="offset points", ha=ha, va="top", fontsize=8,
                        style="italic", color="0.25")
fig.suptitle(f"Top-{args.n} most critical UAVs per policy — K={args.K}, N={args.C}",
             fontsize=10, fontweight="bold")

OUT = os.path.join(SCRIPT_DIR, args.dir, "plots")
os.makedirs(OUT, exist_ok=True)
base = os.path.join(OUT, f"fig_corridorB_critical_uavs_K{args.K}_C{args.C}_top{args.n}")
for ext in ("pdf", "png"):
    fig.savefig(f"{base}.{ext}", dpi=200, bbox_inches="tight")
    print("saved", f"{base}.{ext}")
