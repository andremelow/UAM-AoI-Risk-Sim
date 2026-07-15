#!/usr/bin/env python3
"""table_qos_igrc5.py -- link QoS conditioned on the iGRC-5 state.

The iGRC-5 state = corridor-B slots over the crowd plateau (dist <= hs_end);
everything else (corridor-B tails + route A) is iGRC < 5. Per policy and
state, per-UAV QoS over the state's slots, aggregated across UAVs as
median | p99 (never pooled):
  - grant share [% slots]: C2 (source_sched==1), video (==2)
  - C2 age h1 [s]   : per-UAV mean -> med | p99 across UAVs
  - video age h2 [s]: per-UAV mean -> med | p99 across UAVs
  - SNR [dB]        : per-UAV mean -> fleet median (physical sanity column)

Usage: python3 table_qos_igrc5.py [--dir csv_export_RC14 --K 8 --C 60]
Output: markdown on stdout + <dir>/plots/qos_igrc5_K{K}_C{C}.csv
"""
import os, glob, argparse
import numpy as np, pandas as pd

ap = argparse.ArgumentParser()
ap.add_argument("--dir", default="csv_export_RC14")
ap.add_argument("--scenario", default="4r_b7")
ap.add_argument("--K", type=int, default=8)
ap.add_argument("--C", type=int, default=60)
ap.add_argument("--pol", default="round-robin-sw,pf-classic-sw,mw-sw-v8,mw-sw-v9")
ap.add_argument("--q", type=float, default=0.99)
ap.add_argument("--update-rate", type=float, default=7.0)
ap.add_argument("--hs-end", type=float, default=2843.0)
args = ap.parse_args()

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROUTE_LEN = float(np.hypot(-700 - (-350), 14000 - 9750))
LABEL = {"round-robin-sw": "RR", "pf-classic-sw": "PF",
         "mw-sw-v8": "MW-V8", "mw-sw-v9": "MW-V9"}

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
                     usecols=["slot", "drone_id", "h1_slots", "h2_slots",
                              "snr_db", "source_sched"])
    ss = ss.merge(ds[["drone_id", "entry_slot", "dur"]], on="drone_id")
    ss["is_corr"] = ss.drone_id.isin(set(b.drone_id))
    trav = (ss.slot - ss.entry_slot) / ss.dur * ROUTE_LEN
    ss["dist"] = np.where(ss.drone_id.isin(ids_to), ROUTE_LEN - trav, trav)
    ss["igrc5"] = ss.is_corr & (ss.dist <= args.hs_end)
    return ss

QP = int(100 * args.q)
UR = args.update_rate
rows = []
for pol in args.pol.split(","):
    ss = load(pol)
    for zone, m in (("iGRC 5", ss.igrc5), ("iGRC < 5", ~ss.igrc5)):
        g = ss[m].groupby("drone_id")
        gr_c2 = g.source_sched.apply(lambda s: 100.0 * (s == 1).mean())
        gr_vd = g.source_sched.apply(lambda s: 100.0 * (s == 2).mean())
        h1 = g.h1_slots.mean() / UR
        h2 = g.h2_slots.mean() / UR
        snr = g.snr_db.mean()
        rows.append(dict(
            policy=LABEL.get(pol, pol), zone=zone, n_uav=g.ngroups,
            grant_c2_med=gr_c2.median(), grant_vid_med=gr_vd.median(),
            h1_med=h1.median(), h1_pq=h1.quantile(args.q),
            h2_med=h2.median(), h2_pq=h2.quantile(args.q),
            snr_med=snr.median()))
    print(f"done {LABEL.get(pol, pol)}")

print(f"\nQoS condicionado ao estado iGRC 5 (corredor B, dist <= "
      f"{args.hs_end:.0f} m) -- K={args.K}, N={args.C}; "
      f"idades: média por UAV, med | p{QP} entre UAVs\n")
print(f"| política | estado | UAVs | grant C2 [%] | grant víd [%] | "
      f"idade C2 [s] | idade víd [s] | SNR [dB] |")
print("|" + "---|" * 8)
for r in rows:
    print(f"| {r['policy']} | {r['zone']} | {r['n_uav']} | "
          f"{r['grant_c2_med']:.1f} | {r['grant_vid_med']:.1f} | "
          f"{r['h1_med']:.2f} / {r['h1_pq']:.2f} | "
          f"{r['h2_med']:.1f} / {r['h2_pq']:.1f} | {r['snr_med']:.1f} |")

out = os.path.join(SCRIPT_DIR, args.dir, "plots",
                   f"qos_igrc5_K{args.K}_C{args.C:02d}.csv")
os.makedirs(os.path.dirname(out), exist_ok=True)
pd.DataFrame(rows).to_csv(out, index=False)
print("\nsaved", out)
