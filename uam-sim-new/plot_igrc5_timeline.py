#!/usr/bin/env python3
"""plot_igrc5_timeline.py -- drones in the iGRC-5 state over time, per policy.

Three stacked time series (lines = policies):
  (a) # drones in the iGRC-5 state (corridor B over the crowd, dist <= hs_end)
      -- kinematics are scheduler-independent, so all policies coincide:
      this is the scenario exposure line.
  (b) of those, # C2-blind (r_u > W_c/2  <=>  h1 > h_crit)
  (c) of those, # with stale video (h2 > tau_vid seconds)
Counts are rolling-mean smoothed for readability (window in seconds).

Usage: python3 plot_igrc5_timeline.py [--dir csv_export_RC14 --K 8 --C 60]
Output: <dir>/plots/fig_igrc5_timeline_K{K}_C{C}.{pdf,png}
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
ap.add_argument("--update-rate", type=float, default=7.0)
ap.add_argument("--hs-end", type=float, default=2843.0)
ap.add_argument("--wc", type=float, default=100.0)
ap.add_argument("--r-min", type=float, default=10.0)
ap.add_argument("--tau-vid", type=float, default=15.0, help="video staleness [s]")
ap.add_argument("--smooth", type=float, default=60.0, help="rolling window [s]")
args = ap.parse_args()

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROUTE_LEN = float(np.hypot(-700 - (-350), 14000 - 9750))
LABEL = {"round-robin-sw": "RR", "pf-classic-sw": "PF",
         "mw-sw-v8": "MW-V8", "mw-sw-v9": "MW-V9"}
COLORS = {"round-robin-sw": "#F9A825", "pf-classic-sw": "#E53935",
          "mw-sw-v8": "#00897B", "mw-sw-v9": "#7CB342"}
POL = [p for p in args.pol.split(",") if p]
UR = args.update_rate

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
                     usecols=["slot", "drone_id", "h1_slots", "h2_slots"])
    ss = ss[ss.drone_id.isin(set(b.drone_id))]
    ss = ss.merge(ds[["drone_id", "entry_slot", "dur"]], on="drone_id")
    v = float(np.median(ROUTE_LEN / (b.dur.values / UR)))
    trav = (ss.slot - ss.entry_slot) / ss.dur * ROUTE_LEN
    ss["dist"] = np.where(ss.drone_id.isin(ids_to), ROUTE_LEN - trav, trav)
    ss = ss[ss.dist <= args.hs_end]                    # iGRC-5 state only
    h_crit = (args.wc / 2.0 - args.r_min) / (v / UR)
    g = ss.groupby("slot")
    t = g.size()
    return pd.DataFrame({
        "n": t,
        "blind": g.h1_slots.apply(lambda s: int((s > h_crit).sum())),
        "stale": g.h2_slots.apply(lambda s: int((s > args.tau_vid * UR).sum())),
    }).reindex(range(int(ss.slot.min()), int(ss.slot.max()) + 1), fill_value=0)

plt.rcParams.update({"font.family": "serif", "font.size": 9,
                     "mathtext.fontset": "stix", "pdf.fonttype": 42})
fig, axes = plt.subplots(3, 1, figsize=(6.8, 6.4), sharex=True,
                         constrained_layout=True)
w = max(1, int(args.smooth * UR))
for p in POL:
    df = load(p)
    tmin = df.index.values / UR / 60.0
    for ax, col in zip(axes, ("n", "blind", "stale")):
        y = df[col].rolling(w, center=True, min_periods=1).mean()
        ax.plot(tmin, y, color=COLORS[p], lw=1.3, label=LABEL[p])
    print(f"done {LABEL[p]}: media iGRC5 = {df.n.mean():.1f} drones | "
          f"cegos = {df.blind.mean():.2f} | video>15s = {df.stale.mean():.2f}")

axes[0].set_ylabel("drones em iGRC 5")
axes[0].set_title("exposição (igual por construção entre políticas)",
                  fontsize=8.5, color="0.35")
axes[1].set_ylabel(f"dos quais C2-cegos\n($r_u > W_c/2$)")
axes[2].set_ylabel(f"dos quais vídeo > {args.tau_vid:.0f} s")
axes[2].set_xlabel("tempo [min]")
for ax in axes:
    ax.grid(True, ls=":", lw=0.35, alpha=0.5)
axes[0].legend(ncol=4, fontsize=8.5, frameon=False, loc="lower center")
fig.suptitle(f"Drones no estado iGRC 5 ao longo do tempo -- K={args.K}, "
             f"N={args.C} (média móvel {args.smooth:.0f} s)",
             fontsize=11, fontweight="bold")

out = os.path.join(SCRIPT_DIR, args.dir, "plots",
                   f"fig_igrc5_timeline_K{args.K}_C{args.C:02d}")
for ext in ("pdf", "png"):
    fig.savefig(f"{out}.{ext}", dpi=200, bbox_inches="tight")
    print("saved", f"{out}.{ext}")
