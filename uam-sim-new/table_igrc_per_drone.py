#!/usr/bin/env python3
"""table_igrc_per_drone.py -- SORA Annex F Eq. 5 iGRC, per drone.

Two variants per drone:
  iGRC_nom : Eq. 5 on the max population density actually OVERFLOWN by the
             drone (nominal track). Route property -- policy-independent.
             D_pop recovered from exports: rho_tilde = r_map*(r*/r_u)^2
             (clipped at the plateau 1.0), D_pop = rho_op * rho_tilde.
  iGRC_eff : containment-aware variant -- max density within REACH of the
             uncertainty disk (corridor segment [dist-r_u, dist+r_u]).
             This is where the scheduler enters: stale C2 -> large r_u ->
             the disk can reach the crowd even when the track is off it.
             Corridor-B drones only; others fall back to nominal (flagged).

iGRC(D) = max(1, ceil(7 + log10(D[ppl/m^2] * A_C[m^2]) - 0.5))   (Eq. 5)
A_C Case 3 (w < 1 m): A_C = 2 r_D d_glide + pi r_D^2, r_D = 0.3 + w/2.

Usage: python3 table_igrc_per_drone.py [--dir csv_export_RC14 --K 8 --C 60]
Output: markdown tables on stdout + <dir>/plots/igrc_per_drone_K{K}_C{C}.csv
"""
import os, glob, argparse
import numpy as np, pandas as pd

ap = argparse.ArgumentParser()
ap.add_argument("--dir", default="csv_export_RC14")
ap.add_argument("--scenario", default="4r_b7")
ap.add_argument("--K", type=int, default=8)
ap.add_argument("--C", type=int, default=60)
ap.add_argument("--pol", default="round-robin-sw,pf-classic-sw,mw-sw-v8,mw-sw-v9")
ap.add_argument("--bin", type=float, default=50.0)
ap.add_argument("--w-ua", type=float, default=0.5)
ap.add_argument("--rho-op", type=float, default=5000.0, help="peak [inhab/km^2]")
ap.add_argument("--r-min", type=float, default=10.0)
ap.add_argument("--c2-hz", type=float, default=2.0)
ap.add_argument("--update-rate", type=float, default=7.0)
ap.add_argument("--h1-fresh", type=float, default=14.0,
                help="max h1 [slots] for samples used in the density profile")
args = ap.parse_args()

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROUTE_LEN = float(np.hypot(-700 - (-350), 14000 - 9750))
RHO_PEAK = args.rho_op * 1e-6                      # [ppl/m^2]
H_PERSON, R_PERSON, THETA = 1.8, 0.3, np.deg2rad(35.0)
r_D = R_PERSON + args.w_ua / 2.0
A_C = 2.0 * r_D * (H_PERSON / np.tan(THETA)) + np.pi * r_D**2
LABEL = {"round-robin-sw": "RR", "pf-classic-sw": "PF",
         "mw-sw-v8": "MW-V8", "mw-sw-v9": "MW-V9"}

def igrc(D):
    """Annex F Eq. 5; D in ppl/m^2."""
    E = max(D * A_C, 1e-12)
    return int(max(1, np.ceil(7.0 + np.log10(E) - 0.5)))

def analyze(pol):
    base = os.path.join(SCRIPT_DIR, args.dir, f"K{args.K}", f"C{args.C:02d}",
                        args.scenario)
    ds = pd.read_csv(sorted(glob.glob(os.path.join(base, f"{pol}_*_drone_summary.csv")))[-1])
    ds["dur"] = ds.exit_slot - ds.entry_slot
    corr = ds[ds.dur < 0.7 * ds.dur.max()].sort_values("drone_id")
    ent = corr.entry_slot.values
    resets = np.flatnonzero(np.diff(ent) < 0)
    cut = (resets[0] + 1) if len(resets) else len(corr) // 2
    ids_to = set(corr.drone_id.values[:cut])
    corr_ids = set(corr.drone_id.values)

    ss = pd.read_csv(sorted(glob.glob(os.path.join(base, f"{pol}_*_slot_series.csv")))[-1],
                     usecols=["slot", "drone_id", "h1_slots", "r_map"])
    ss = ss.merge(ds[["drone_id", "entry_slot", "dur"]], on="drone_id")

    v = float(np.median(ROUTE_LEN / (corr.dur.values / args.update_rate)))
    r_star = args.r_min + v / args.c2_hz
    ss["r_u"] = args.r_min + (v / args.update_rate) * ss.h1_slots.values
    ss["rho"] = np.minimum(ss.r_map.values * (r_star / ss.r_u.values) ** 2, 1.0)

    ss["is_corr"] = ss.drone_id.isin(corr_ids)
    trav = (ss.slot - ss.entry_slot) / ss.dur * ROUTE_LEN
    ss["dist"] = np.where(ss.drone_id.isin(ids_to), ROUTE_LEN - trav, trav)

    # normalized density profile along corridor B from fresh-C2 samples only
    # (small r_u -> de-normalization accurate, clipping rare)
    edges = np.arange(0.0, ROUTE_LEN + args.bin, args.bin)
    fresh = ss[ss.is_corr & (ss.h1_slots <= args.h1_fresh)]
    prof = fresh.groupby(pd.cut(fresh.dist, edges, labels=False),
                         observed=True).rho.median()
    prof = prof.reindex(range(len(edges) - 1)).interpolate(
        limit_direction="both").values

    rows = []
    for did, g in ss.groupby("drone_id"):
        rho_nom = float(g.rho.max())
        is_corr = bool(g.is_corr.iloc[0])
        if is_corr:
            # reach-aware: max profile density in [dist - r_u, dist + r_u]
            b = np.clip((g.dist.values / args.bin).astype(int), 0, len(prof) - 1)
            w = np.ceil(g.r_u.values / args.bin).astype(int)
            rho_reach = rho_nom
            for wu in np.unique(w):
                m = w == wu
                lo = np.clip(b[m] - wu, 0, len(prof) - 1)
                hi = np.clip(b[m] + wu, 0, len(prof) - 1)
                # window max via loop over the few unique widths
                pm = np.array([np.nanmax(prof[l:h + 1]) for l, h in zip(lo, hi)])
                rho_reach = max(rho_reach, float(np.nanmax(pm)))
        else:
            rho_reach = rho_nom                      # no dist mapping (route A)
        rows.append(dict(policy=LABEL.get(pol, pol), drone_id=did,
                         route="B" if is_corr else "A",
                         rho_nom=rho_nom, rho_reach=rho_reach,
                         r_u_max=float(g.r_u.max()),
                         igrc_nom=igrc(RHO_PEAK * rho_nom),
                         igrc_eff=igrc(RHO_PEAK * rho_reach)))
    return pd.DataFrame(rows)

all_rows = []
for pol in args.pol.split(","):
    try:
        df = analyze(pol)
        all_rows.append(df)
        print(f"done {LABEL.get(pol, pol)}")
    except (IndexError, FileNotFoundError) as e:
        print(f"skip {pol}: {type(e).__name__}: {e}")
alld = pd.concat(all_rows, ignore_index=True)

print(f"\nA_C = {A_C:.2f} m^2 | rho_peak = {args.rho_op:.0f}/km^2 | "
      f"K={args.K}, N={args.C} | iGRC = ceil(7 + log10(Dpop*A_C) - 0.5)\n")

def dist_str(s):
    """compact iGRC distribution, e.g. '5 (x92)' or '3 (x4), 5 (x88)'"""
    return ", ".join(f"{v} (x{n})" for v, n in s.value_counts().sort_index().items())

hdr = ("| política | rota | drones | iGRC nom | iGRC eff | eff>nom "
       "| r_u máx [m] | r_u máx A [m] | r_u máx B [m] |")
print(hdr)
print("|" + "---|" * 9)
for lab, gp in alld.groupby("policy", sort=False):
    ru_all = gp.r_u_max.max()
    ru = {rt: g.r_u_max.max() for rt, g in gp.groupby("route")}
    for rt, g in gp.groupby("route"):
        up = int((g.igrc_eff > g.igrc_nom).sum())
        print(f"| {lab} | {rt} | {len(g)} | {dist_str(g.igrc_nom)} | "
              f"{dist_str(g.igrc_eff)} | {up} | {ru_all:.0f} | "
              f"{ru.get('A', float('nan')):.0f} | {ru.get('B', float('nan')):.0f} |")

out = os.path.join(SCRIPT_DIR, args.dir, "plots",
                   f"igrc_per_drone_K{args.K}_C{args.C:02d}.csv")
os.makedirs(os.path.dirname(out), exist_ok=True)
alld.to_csv(out, index=False)
print("\nscenario iGRC (max over drones, nominal) =", int(alld.igrc_nom.max()))
print("saved", out)
