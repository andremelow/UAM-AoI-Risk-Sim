#!/usr/bin/env python3
"""table_sora_sail_by_policy.py -- SORA (Annex F v2.5) SAIL table per policy.

Row logic:
  (i)  intrinsic requirement -- Eq. (3) worst overflown segment (Sec. 3.3.2):
       E_C = Dpop*A_C -> max allowable lambda_GI = TLOS/E_C -> min SAIL.
       Scenario property: identical for every policy (scheduler cannot change
       Dpop or A_C).
  (ii) whether the POLICY sustains that SAIL claim, via the two channels the
       scheduler does control:
        - containment: rate/FH of C2-blind events (r_u > W_c/2) starting over
          the crowd zone (p_q across UAVs). Budget left for blind-induced loss
          of control: lambda_GI^max - 10^-SAILmin -> max tolerable
          P(LoC | blind event) = budget / rate.
        - DAA/video availability over the crowd: share of crowd-zone slots
          with r_vid == 0 (video within coherence window), per-UAV, worst
          (min) across UAVs.

Output: markdown table on stdout + CSV in <dir>/plots/sora_sail_by_policy.csv
Usage: python3 table_sora_sail_by_policy.py [--K 8 --C 60]
"""
import os, glob, argparse
import numpy as np, pandas as pd

ap = argparse.ArgumentParser()
ap.add_argument("--dir", default="csv_export_RC14")
ap.add_argument("--zdir", default="csv_export_RC14Z9", help="dir for mw-z-v9")
ap.add_argument("--scenario", default="4r_b7")
ap.add_argument("--K", type=int, default=8)
ap.add_argument("--C", type=int, default=60)
ap.add_argument("--q", type=float, default=0.99)
ap.add_argument("--bin", type=float, default=50.0)
ap.add_argument("--sail-margin", type=float, default=1.0,
                help="fraction of the lambda_GI budget granted to the platform "
                     "at the min SAIL (rest is scheduler headroom)")
ap.add_argument("--w-ua", type=float, default=0.5)
ap.add_argument("--rho-op", type=float, default=5000.0)
ap.add_argument("--r-min", type=float, default=10.0)
ap.add_argument("--c2-hz", type=float, default=2.0)
ap.add_argument("--update-rate", type=float, default=7.0)
ap.add_argument("--wc", type=float, default=100.0)
ap.add_argument("--hs-end", type=float, default=2843.0)
args = ap.parse_args()

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROUTE_LEN = float(np.hypot(-700 - (-350), 14000 - 9750))
TLOS = 1e-6
RHO_OP = args.rho_op * 1e-6
H_PERSON, R_PERSON, THETA = 1.8, 0.3, np.deg2rad(35.0)
r_D = R_PERSON + args.w_ua / 2.0
A_C = 2.0 * r_D * (H_PERSON / np.tan(THETA)) + np.pi * r_D**2
ROMAN = {1: "I", 2: "II", 3: "III", 4: "IV", 5: "V", 6: "VI"}

POLS = [("round-robin-sw", "RR",      args.dir),
        ("pf-classic-sw",  "PF",      args.dir),
        ("mw-sw-v8",       "MW-V8",   args.dir),
        ("mw-sw-v9",       "MW-V9",   args.dir)]

def steady_window(ds):
    t_max = int(ds.exit_slot.max()) + 1
    delta = np.zeros(t_max + 1)
    np.add.at(delta, ds.entry_slot.astype(int).values, 1)
    np.add.at(delta, ds.exit_slot.astype(int).values, -1)
    airborne = np.cumsum(delta)[:-1]
    # RC14V3 (A-heavy route_frac) never reaches C simultaneous UAVs because of
    # per-route floor rounding -- fall back to the max-occupancy plateau.
    thr = min(args.C, airborne.max())
    full = np.flatnonzero(airborne >= thr)
    return int(full[0]), int(full[-1])

def analyze(pol, base_dir):
    base = os.path.join(SCRIPT_DIR, base_dir, f"K{args.K}", f"C{args.C:02d}",
                        args.scenario)
    ds = pd.read_csv(sorted(glob.glob(os.path.join(base, f"{pol}_*_drone_summary.csv")))[-1])
    t0, t1 = steady_window(ds)
    ds["dur"] = ds.exit_slot - ds.entry_slot
    b = ds[ds.dur < 0.7 * ds.dur.max()].sort_values("drone_id")
    ent = b.entry_slot.values
    resets = np.flatnonzero(np.diff(ent) < 0)
    cut = (resets[0] + 1) if len(resets) else len(b) // 2
    ids_to = set(b.drone_id.values[:cut])
    keep = set(b.drone_id.values)

    ss = pd.read_csv(sorted(glob.glob(os.path.join(base, f"{pol}_*_slot_series.csv")))[-1],
                     usecols=["slot", "drone_id", "h1_slots", "h2_slots", "r_map"])
    ss = ss[ss.drone_id.isin(keep) & ss.slot.between(t0, t1)]
    ss = ss.merge(ds[["drone_id", "entry_slot", "dur"]], on="drone_id")

    v = float(np.median(ROUTE_LEN / (b.dur.values / args.update_rate)))
    r_star = args.r_min + v / args.c2_hz
    r_u = args.r_min + (v / args.update_rate) * ss.h1_slots.values
    rho_tilde = np.minimum(ss.r_map.values * (r_star / r_u) ** 2, 1.0)
    ss["EC"] = RHO_OP * rho_tilde * A_C
    trav = (ss.slot - ss.entry_slot) / ss.dur * ROUTE_LEN
    ss["dist"] = np.where(ss.drone_id.isin(ids_to), ROUTE_LEN - trav, trav)

    # (i) intrinsic: worst-bin p_q of per-UAV bin-mean E_C
    edges = np.arange(0.0, ROUTE_LEN + args.bin, args.bin)
    ss["bin"] = pd.cut(ss.dist, edges, labels=False)
    gu = ss.groupby(["bin", "drone_id"], observed=True).EC.mean()
    ec_max = float(gu.groupby(level=0).quantile(args.q).max())
    lam_max = TLOS / ec_max
    sail_min = int(min(6, max(1, np.ceil(-np.log10(lam_max)))))

    # (ii-a) containment-blind events over the crowd
    h_crit = (args.wc / 2.0 - args.r_min) / (v / args.update_rate)
    rates_crowd, avail = [], []
    for _, g in ss.sort_values("slot").groupby("drone_id"):
        above = (g.h1_slots.values > h_crit).astype(int)
        starts = np.flatnonzero(np.diff(above) == 1) + 1
        if above[0]:
            starts = np.r_[0, starts]
        hours = (g.dur.iloc[0] / args.update_rate) / 3600.0
        n_crowd = int(np.sum(g.dist.values[starts] <= args.hs_end)) if len(starts) else 0
        rates_crowd.append(n_crowd / hours)
        # (ii-b) DAA/video staleness over the crowd: mean video age [s]
        gc = g[g.dist <= args.hs_end]
        if len(gc):
            avail.append(float(gc.h2_slots.mean() / args.update_rate))
    rate_q = float(np.quantile(rates_crowd, args.q))
    vid_age_q = float(np.quantile(avail, args.q))
    vid_age_med = float(np.median(avail))

    # scheduler headroom at the min SAIL
    budget = lam_max - args.sail_margin * 10.0 ** (-sail_min)
    p_loc_max = np.inf if rate_q <= 0 else max(budget, 0.0) / rate_q

    if rate_q <= 0:
        verdict = f"SAIL {ROMAN[sail_min]} sustentável"
    elif p_loc_max >= 1e-6:
        verdict = f"SAIL {ROMAN[sail_min]} condicional"
    else:
        verdict = "não sustenta (≈ Certified)"
    return dict(policy=pol, EC=ec_max, lam_max=lam_max, sail_min=sail_min,
                blind_crowd_pq=rate_q, p_loc_max=p_loc_max,
                vid_age_med=vid_age_med, vid_age_pq=vid_age_q,
                verdict=verdict)

rows = []
for pol, lab, d in POLS:
    try:
        r = analyze(pol, d)
        r["label"] = lab
        rows.append(r)
        print(f"done {lab}")
    except (IndexError, FileNotFoundError) as e:
        print(f"skip {lab}: {type(e).__name__}: {e} ({d})")

QP = int(round(100 * args.q))
print(f"\nA_C = {A_C:.2f} m^2 | TLOS = 1e-6 fat/FH | crowd zone <= {args.hs_end:.0f} m "
      f"| K={args.K}, N={args.C}\n")
hdr = (f"| política | E_C máx [ppl] | λ_GI máx [/FH] | SAIL mín | "
       f"cegos/FH multidão (p{QP}) | P(LoC|cego) máx | idade vídeo multidão med/p{QP} [s] | veredito |")
print(hdr)
print("|" + "---|" * 8)
for r in rows:
    ploc = "—" if np.isinf(r["p_loc_max"]) else f"{r['p_loc_max']:.1e}"
    print(f"| {r['label']} | {r['EC']:.3f} | {r['lam_max']:.2e} | "
          f"{ROMAN[r['sail_min']]} | {r['blind_crowd_pq']:.1f} | {ploc} | "
          f"{r['vid_age_med']:.1f} / {r['vid_age_pq']:.1f} | {r['verdict']} |")

out = os.path.join(SCRIPT_DIR, args.dir, "plots",
                   f"sora_sail_by_policy_K{args.K}_C{args.C}.csv")
pd.DataFrame(rows)[["label", "EC", "lam_max", "sail_min", "blind_crowd_pq",
                    "p_loc_max", "vid_age_med", "vid_age_pq", "verdict"]]\
  .to_csv(out, index=False)
print("\nsaved", out)
