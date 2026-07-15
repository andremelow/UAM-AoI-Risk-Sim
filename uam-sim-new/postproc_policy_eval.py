#!/usr/bin/env python3
"""postproc_policy_eval.py -- flow-level SLA evaluation per policy, no R.

Standalone post-processing (does not touch the simulator). For each
(policy, K, N, seed, UAV, flow) counts VIOLATION EVENTS -- maximal contiguous
runs where age_flow > tau_flow (one run = one contingency activation) -- and
converts to events per UAV flight hour.

Flow -> trace mapping (this simulator exports two age traces):
  c2        : h1_slots  vs tau_c2    (containment-critical C2 age)
  telemetry : h1_slots  vs tau_track (tracking-grade position age; same trace
                                      as c2, tighter threshold)
  video     : h2_slots  vs tau_daa   (DAA video-loop age; also reported as
                                      time-fraction unavailability)

Statistics: rates are computed PER SEED; within a seed the fleet statistic is
the WORST CASE across UAVs (max_u), never a fleet mean. Seeds are then
averaged with a 95% CI (t-based for n>=2 seeds; for n=1 the CI is the exact
Garwood/Poisson interval on the worst UAV's event count). Zero events in a
seed => upper bound lambda <= 3 / T_FH(total seed flight hours), rule of
three, flagged as a bound.

Verdict per (policy, K, N): PASS iff
  lambda_c2   <= alpha * lambda_max          (containment budget)
  lambda_track<= lambda_max_track            (telemetry-loss budget)
  u_video     <= u_max_video                 (video-loop unavailability)
No weighted sums anywhere. Summary number per policy = fraction of the
(K, N) grid where it passes.

Config: DEFAULTS below, overridable via --config file.json. Defaults are
declared placeholders, not regulatory values.

Usage:
  python3 postproc_policy_eval.py --dir csv_export_RC14 [--config cfg.json]
Outputs (<dir>/plots/policy_eval/):
  pass_fail_table.csv, rates_per_seed.csv, fig_lambda_c2_vs_N.{pdf,png}
"""
import os, glob, json, argparse
import numpy as np, pandas as pd
from scipy.stats import chi2, t as tdist
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

DEFAULTS = dict(
    tau_c2_s=4.0,          # containment: r_u > W_c/2  <=>  h1 > ~4 s (geometry)
    tau_track_s=2.0,       # tracking-grade telemetry age
    tau_daa_s=15.0,        # video coherence for the DAA loop
    lambda_max=2.0,        # contingency budget [events/FH] before alpha
    alpha=0.1,             # fraction of the budget granted to the C2 flow
    lambda_max_track=10.0, # telemetry-loss events/FH
    u_max_video=0.10,      # max video-loop unavailability (time fraction)
    slot_s=1.0 / 7.0,      # slot duration [s]
    ci_level=0.95,
)

ap = argparse.ArgumentParser()
ap.add_argument("--dir", default="csv_export_RC14")
ap.add_argument("--scenario", default="4r_b7")
ap.add_argument("--config", default=None, help="JSON overriding DEFAULTS")
ap.add_argument("--pol", default="round-robin-sw,pf-classic-sw,mw-sw-v8,mw-sw-v9")
ap.add_argument("--K-list", default="", help="restrict grid, e.g. 8,16")
ap.add_argument("--fig-K", type=int, default=8, help="K shown in the figure")
args = ap.parse_args()

CFG = dict(DEFAULTS)
if args.config:
    with open(args.config) as f:
        CFG.update(json.load(f))

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(SCRIPT_DIR, args.dir)
POL = [p for p in args.pol.split(",") if p]
LABEL = {"round-robin-sw": "RR", "pf-classic-sw": "PF",
         "mw-sw-v8": "MW-V8", "mw-sw-v9": "MW-V9"}
# grayscale-safe, IEEE single column
STYLE = {"round-robin-sw": ("0.00", "-", "o"), "pf-classic-sw": ("0.35", "--", "s"),
         "mw-sw-v8": ("0.55", ":", "x"), "mw-sw-v9": ("0.00", "-.", "^")}
FLOWS = [("c2", "h1_slots", CFG["tau_c2_s"]),
         ("telemetry", "h1_slots", CFG["tau_track_s"]),
         ("video", "h2_slots", CFG["tau_daa_s"])]

def grid_cells():
    ks = ([int(k) for k in args.K_list.split(",") if k] if args.K_list else
          sorted(int(d[1:]) for d in os.listdir(DATA) if d.startswith("K")))
    for K in ks:
        for cd in sorted(os.listdir(os.path.join(DATA, f"K{K}"))):
            if cd.startswith("C"):
                yield K, int(cd[1:])

def count_events(above):
    """number of maximal contiguous True-runs"""
    return int(above[0]) + int(np.sum(np.diff(above.astype(np.int8)) == 1))

def eval_seed(ss, dur_h):
    """per-UAV event rates for one seed replicate -> seed-level worst cases"""
    out = {}
    per_uav = {f: [] for f, _, _ in FLOWS}
    u_vid = []
    for did, g in ss.groupby("drone_id", sort=False):
        T = dur_h[did]
        for f, col, tau in FLOWS:
            above = g[col].values * CFG["slot_s"] > tau
            per_uav[f].append((count_events(above), T))
        u_vid.append(np.mean(g["h2_slots"].values * CFG["slot_s"] > CFG["tau_daa_s"]))
    T_total = sum(T for _, T in per_uav["c2"])
    for f, _, _ in FLOWS:
        ks_ = np.array([k for k, _ in per_uav[f]], float)
        Ts = np.array([T for _, T in per_uav[f]], float)
        rates = ks_ / Ts
        i = int(np.argmax(rates))
        if ks_.sum() == 0:                       # rule of three on total exposure
            out[f] = dict(lam=3.0 / T_total, bound=True, k=0, T=T_total)
        else:
            out[f] = dict(lam=float(rates[i]), bound=False,
                          k=int(ks_[i]), T=float(Ts[i]))
    out["u_video"] = float(np.max(u_vid))
    return out

def garwood(k, T, level):
    """exact Poisson CI for a count k over exposure T"""
    a = 1.0 - level
    lo = 0.0 if k == 0 else chi2.ppf(a / 2, 2 * k) / (2 * T)
    hi = chi2.ppf(1 - a / 2, 2 * k + 2) / (2 * T)
    return lo, hi

rows, per_seed_rows = [], []
for K, C in grid_cells():
    base = os.path.join(DATA, f"K{K}", f"C{C:02d}", args.scenario)
    for pol in POL:
        runs = sorted(glob.glob(os.path.join(base, f"{pol}_*_slot_series.csv")))
        if not runs:
            continue
        seeds = []
        for run in runs:                          # each run file = one seed
            ds = pd.read_csv(run.replace("_slot_series", "_drone_summary"),
                             usecols=["drone_id", "entry_slot", "exit_slot"])
            dur_h = dict(zip(ds.drone_id,
                             (ds.exit_slot - ds.entry_slot) * CFG["slot_s"] / 3600.0))
            ss = pd.read_csv(run, usecols=["drone_id", "h1_slots", "h2_slots"])
            s = eval_seed(ss, dur_h)
            s.update(policy=pol, K=K, N=C, seed=os.path.basename(run))
            seeds.append(s)
            per_seed_rows.append({**{f"lam_{f}": s[f]["lam"] for f, _, _ in FLOWS},
                                  **{f"bound_{f}": s[f]["bound"] for f, _, _ in FLOWS},
                                  "u_video": s["u_video"], "policy": pol,
                                  "K": K, "N": C, "seed": s["seed"]})
        n = len(seeds)
        r = dict(policy=pol, K=K, N=C, n_seeds=n)
        for f, _, _ in FLOWS:
            lams = np.array([s[f]["lam"] for s in seeds])
            r[f"lam_{f}"] = float(lams.mean())
            r[f"bound_{f}"] = all(s[f]["bound"] for s in seeds)
            if n >= 2:
                half = tdist.ppf(0.5 + CFG["ci_level"] / 2, n - 1) * lams.std(ddof=1) / np.sqrt(n)
                r[f"lo_{f}"], r[f"hi_{f}"] = max(0.0, r[f"lam_{f}"] - half), r[f"lam_{f}"] + half
            else:                                 # n=1: exact Poisson on the count
                r[f"lo_{f}"], r[f"hi_{f}"] = garwood(seeds[0][f]["k"],
                                                     seeds[0][f]["T"], CFG["ci_level"])
        r["u_video"] = float(np.mean([s["u_video"] for s in seeds]))
        r["pass_c2"] = r["lam_c2"] <= CFG["alpha"] * CFG["lambda_max"]
        r["pass_track"] = r["lam_telemetry"] <= CFG["lambda_max_track"]
        r["pass_video"] = r["u_video"] <= CFG["u_max_video"]
        # a criterion violated only by a zero-event rule-of-three bound is not
        # an observed failure -- the exposure is just too small to demonstrate
        # the budget: INCONCL, not FAIL
        obs_fail = ((not r["pass_c2"] and not r["bound_c2"]) or
                    (not r["pass_track"] and not r["bound_telemetry"]) or
                    not r["pass_video"])
        allpass = r["pass_c2"] and r["pass_track"] and r["pass_video"]
        r["PASS"] = allpass
        r["verdict"] = "PASS" if allpass else ("FAIL" if obs_fail else "INCONCL")
        rows.append(r)
    print(f"cell K{K}/N{C} done")

res = pd.DataFrame(rows)
OUT = os.path.join(DATA, "plots", "policy_eval")
os.makedirs(OUT, exist_ok=True)
res.to_csv(os.path.join(OUT, "pass_fail_table.csv"), index=False)
pd.DataFrame(per_seed_rows).to_csv(os.path.join(OUT, "rates_per_seed.csv"), index=False)

print("\nconfig:", json.dumps(CFG))
print(f"\nbudget C2 = alpha*lambda_max = {CFG['alpha'] * CFG['lambda_max']:.3g}/FH | "
      f"telemetry <= {CFG['lambda_max_track']:.3g}/FH | "
      f"u_video <= {CFG['u_max_video']:.0%}\n")
print("| política | K | N | λ_c2 [/FH] | λ_telem [/FH] | u_vídeo | veredito |")
print("|" + "---|" * 7)
for _, r in res.iterrows():
    bc = "≤" if r.bound_c2 else ""
    bt = "≤" if r.bound_telemetry else ""
    print(f"| {LABEL.get(r.policy, r.policy)} | {r.K} | {r.N} | "
          f"{bc}{r.lam_c2:.3g} | {bt}{r.lam_telemetry:.3g} | "
          f"{r.u_video:.1%} | {r.verdict} |")

print("\nfração da grade (K,N) por política:")
for pol in POL:
    sub = res[res.policy == pol]
    ninc = int((sub.verdict == "INCONCL").sum())
    print(f"  {LABEL.get(pol, pol):7} PASS {sub.PASS.mean():6.1%} "
          f"({int(sub.PASS.sum())}/{len(sub)})  INCONCL {ninc}  "
          f"FAIL {int((sub.verdict == 'FAIL').sum())}")

# figure: lambda_c2 vs N at fig-K, IEEE single column
sub = res[res.K == args.fig_K]
if len(sub):
    plt.rcParams.update({"font.family": "serif", "font.size": 8,
                         "mathtext.fontset": "stix", "pdf.fonttype": 42})
    fig, ax = plt.subplots(figsize=(3.5, 2.6), constrained_layout=True)
    for pol in POL:
        g = sub[sub.policy == pol].sort_values("N")
        if not len(g):
            continue
        col, ls, mk = STYLE[pol]
        ax.plot(g.N, g.lam_c2, color=col, ls=ls, lw=1.1, label=LABEL[pol])
        obs, bnd = g[~g.bound_c2], g[g.bound_c2]
        ax.plot(obs.N, obs.lam_c2, ls="none", marker=mk, ms=3.5, color=col)
        ax.plot(bnd.N, bnd.lam_c2, ls="none", marker=mk, ms=3.5, color=col,
                mfc="white")
        ax.vlines(g.N, g.lo_c2, g.hi_c2, color=col, lw=0.6, alpha=0.7)
    bud = CFG["alpha"] * CFG["lambda_max"]
    ax.axhline(bud, color="k", lw=0.8)
    ax.text(sub.N.max(), bud, rf" $\alpha\,\lambda_{{max}}$",
            fontsize=7, va="bottom", ha="right")
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xticks(sorted(sub.N.unique()))
    ax.set_xticklabels([str(n) for n in sorted(sub.N.unique())], fontsize=6)
    ax.tick_params(axis="x", which="minor", bottom=False)
    ax.set_xlabel("Simultaneous UAVs $N$")
    ax.set_ylabel(r"$\lambda_{c2}$ [events/FH], worst UAV")
    ax.grid(True, which="both", ls=":", lw=0.3, alpha=0.5)
    ax.legend(fontsize=6.5, frameon=False, ncol=2)
    ax.set_title(f"K = {args.fig_K} (open markers: rule-of-three bound)",
                 fontsize=7)
    for ext in ("pdf", "png"):
        fig.savefig(os.path.join(OUT, f"fig_lambda_c2_vs_N.{ext}"),
                    dpi=200, bbox_inches="tight")
    print("\nsaved figure", os.path.join(OUT, "fig_lambda_c2_vs_N.{pdf,png}"))
