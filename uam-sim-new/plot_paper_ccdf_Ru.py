#!/usr/bin/env python3
"""FIG A -- fig_ccdf_Ru_N{N_TESE}_K{K_TESE}.pdf  (IEEE single column, vector PDF)

Empirical CCDF of the instantaneous composite risk R_{u*}(t) of the WORST UAV
u* at the thesis cell (K = K_TESE, N = N_TESE), one curve per policy:
  u*  = argmax_u of the temporal mean of R_u(t) (t > BURNIN), averaged across
        the SEEDS seeds (per-seed temporal mean -> seed mean -> argmax);
  CCDF pooled over the SEEDS seeds' post-burn-in samples of R_{u*}(t);
  each curve truncated where fewer than MIN_EXC exceedances support it.

Vertical dashed line at x = 1: "design envelope (R_u > 1, TLOS-calibrated)"
(valid ONLY under the r_tau renormalisation r* = r_min + v_max*tau_max --
attested by the dataset validation, which otherwise aborts).
Horizontal dotted line at 1e-2 (the p99 level). NO lower-bound (L_B) content.

Sidecar: fig_ccdf_Ru_N{N}_K{K}.json (headline numbers) + _caption.txt.

Usage:  python3 plot_paper_ccdf_Ru.py [--dir csv_export_RC12]
"""
import os, json, argparse
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import fig_style
import paper_data_checks as pdc

# ── Parameters ────────────────────────────────────────────────────────
K_TESE  = 16
N_TESE  = 100
BURNIN  = int(1e4)     # slots discarded before any estimate
SEEDS   = 10
T       = int(1e6)     # required horizon (slots) per run
MIN_EXC = 100          # truncate CCDF below 100 exceedances (~1e-5 with 1e7 samples)

POL   = pdc.REQUIRED_POLICIES
LABEL = {"round-robin-sw": "RR", "pf-classic-sw": "PF",
         "mw-sw-v9": "Risk-Aware", "mw-v9-nsw": "Risk-Aware (non-preempt.)"}
SPLIT_COLORS = {"round-robin-sw": "#F9A825", "pf-classic-sw": "#E53935",
                "mw-sw-v9": "#7CB342", "mw-v9-nsw": "#33691E"}

ap = argparse.ArgumentParser()
ap.add_argument("--dir", default="csv_export_RC12")
ap.add_argument("--scenario", default="4r_b7")
ap.add_argument("--K", type=int, default=K_TESE)
ap.add_argument("--N", type=int, default=N_TESE)
ap.add_argument("--seeds", type=int, default=SEEDS)
ap.add_argument("--T", type=float, default=T)
ap.add_argument("--burnin", type=float, default=BURNIN)
ap.add_argument("--min-exc", type=int, default=MIN_EXC)
ap.add_argument("--min-n", type=int, default=100,
                help="u* candidates need at least this many post-burn-in samples")
ap.add_argument("--outbound", action="store_true",
                help="restrict u* selection to OUTBOUND drones (RouteA/RouteBhalf)")
ap.add_argument("--exclude-ramps", action="store_true",
                help="mask both 500 m ramps (climb-out + landing outage); suffix _noramps")
ap.add_argument("--cdf", action="store_true",
                help="plot the empirical CDF P(R_{u*}(t) <= x) instead of the CCDF; "
                     "annotates P(R_{u*} < 1) per policy")
ap.add_argument("--color", action="store_true",
                help="use the split-figure color palette instead of grayscale")
ap.add_argument("--invert", action="store_true",
                help="swap axes: probability on x, risk on y")
ap.add_argument("--allow-unattested-renorm", action="store_true",
                help="downgrade validation 3 to a warning; envelope label drops "
                     "the TLOS-calibrated claim")
args = ap.parse_args()
SEEDS, T, BURNIN, MIN_EXC = args.seeds, int(args.T), int(args.burnin), args.min_exc
K_TESE, N_TESE = args.K, args.N

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(SCRIPT_DIR, args.dir)
OUT  = os.path.join(DATA, "plots")
CACHE = os.path.join(OUT, "paper_cache")
os.makedirs(OUT, exist_ok=True)

# ── Validations 1-3 (abort inside on failure) ─────────────────────────
runs_by = pdc.validate(DATA, args.scenario, [(K_TESE, N_TESE)], SEEDS, T,
                       strict_renorm=not args.allow_unattested_renorm)

# ── u* and pooled CCDF per policy ─────────────────────────────────────
results = {}
for pol in POL:
    runs = runs_by[(K_TESE, N_TESE, pol)][:SEEDS]
    stats = [pdc.uav_stats(r, BURNIN, CACHE, exclude_ramps=args.exclude_ramps) for r in runs]
    stats = [st[st.n >= args.min_n] for st in stats]
    if args.outbound:
        meta = pd.read_csv(runs[0]["ss_path"].replace("_slot_series.csv", "_drone_summary.csv"),
                           usecols=["drone_id", "entry_slot", "exit_slot"])
        meta["dur"] = meta.exit_slot - meta.entry_slot
        li = sorted(meta[meta.dur > meta.dur.min() * 1.5].drone_id)
        si = sorted(meta[~(meta.dur > meta.dur.min() * 1.5)].drone_id)
        ob = set(li[:len(li)//2]) | set(si[:len(si)//2])
        stats = [st[st.drone_id.isin(ob)] for st in stats]
    # per-seed temporal mean -> mean across seeds per UAV (inner join on ids)
    mm = stats[0][["drone_id", "mean_r"]].rename(columns={"mean_r": "m0"})
    for i, st in enumerate(stats[1:], 1):
        mm = mm.merge(st[["drone_id", "mean_r"]].rename(columns={"mean_r": f"m{i}"}),
                      on="drone_id", how="inner")
    seed_mean = mm.drop(columns="drone_id").mean(axis=1)
    u_star = int(mm.drone_id.iloc[int(seed_mean.values.argmax())])
    pooled = np.concatenate([pdc.uav_series(r, u_star, BURNIN,
                                            exclude_ramps=args.exclude_ramps) for r in runs])
    results[pol] = dict(u_star=u_star, pooled=pooled)
    print(f"{LABEL[pol]:28} u*={u_star}  samples={pooled.size}")

# ── Figure ────────────────────────────────────────────────────────────
fig_style.apply()
fig, ax = plt.subplots(figsize=(3.5, 2.6), constrained_layout=True)

for pol in POL:
    r = results[pol]["pooled"]
    xs = np.sort(r[r > 0]); n = xs.size
    if args.cdf:
        p = (np.arange(n) + 1) / n               # P(R <= x)
        keep = np.ones(n, bool)
    else:
        p = (n - np.arange(n)) / n               # P(R > x)
        keep = n * p >= MIN_EXC                  # truncate the unsupported tail
    idx = np.unique(np.linspace(0, n - 1, 2000).astype(int))
    idx = idx[keep[idx]]
    if args.color:
        kw = dict(color=SPLIT_COLORS[pol], lw=1.8)
    else:
        kw = fig_style.line_kwargs(pol)
        kw.pop("marker", None); kw.pop("markerfacecolor", None); kw.pop("markeredgecolor", None)
    kw["label"] = LABEL[pol]
    if args.invert:
        ax.plot(p[idx], xs[idx], **kw)
    else:
        ax.plot(xs[idx], p[idx], **kw)

env_lbl = (" design envelope\n ($R_u > 1$, TLOS-calibrated)"
           if not args.allow_unattested_renorm else " design envelope\n ($R_u > 1$)")
if args.invert:
    ax.set_yscale("log")                        # risk on y
    if not args.cdf:
        ax.set_xscale("log")
    ax.axhline(1.0, ls=(0, (4, 3)), lw=0.9, color="0.35")
    ax.text(0.02, 1.0, env_lbl.replace("\n", ""), transform=ax.get_yaxis_transform(),
            ha="left", va="bottom", fontsize=6.5, color="0.35")
else:
    if not args.cdf:
        ax.set_yscale("log")
    ax.set_xscale("log")
    ax.axvline(1.0, ls=(0, (4, 3)), lw=0.9, color="0.35")
    ax.text(1.0, 0.985, env_lbl,
            transform=ax.get_xaxis_transform(), ha="left", va="top",
            fontsize=6.5, color="0.35")
prob_lbl = (r"$P(R_{u^*}(t) \leq x)$" if args.cdf else r"$P(R_{u^*}(t) > x)$")
if args.cdf:
    notes = [f"$P(R_{{u^*}}\!<\!1)$ = " +
             f"{float((results[pol]['pooled'] < 1.0).mean()):.3f}  {LABEL[pol]}"
             for pol in POL]
    ax.text(0.98, 0.03, "\n".join(notes), transform=ax.transAxes, fontsize=6.2,
            va="bottom", ha="right", color="0.15")
if args.invert:
    ax.set_xlim(0, 1.02) if args.cdf else None
    ax.set_xlabel(prob_lbl)
    ax.set_ylabel("Composite risk, $x$")
    ax.legend(loc="upper left", fontsize=7)
else:
    if args.cdf:
        ax.set_ylim(0, 1.02)
    else:
        ax.axhline(1e-2, ls=":", lw=0.9, color="0.35")
    ax.set_ylabel(prob_lbl)
    ax.set_xlabel("Composite risk, $x$")
    ax.legend(loc="upper left" if args.cdf else "lower left", fontsize=7)
ax.grid(True, which="both", ls=":", lw=0.4, alpha=0.6)

base = os.path.join(OUT, f"fig_{'cdf' if args.cdf else 'ccdf'}_Ru_N{N_TESE}_K{K_TESE}"
                    + ("_inv" if args.invert else "") + ("_color" if args.color else "")
                    + ("_noramps" if args.exclude_ramps else "")
                    + ("_outbound" if args.outbound else ""))
fig.savefig(base + ".pdf", bbox_inches="tight")
fig.savefig(base + ".png", dpi=200, bbox_inches="tight")
print("saved", base + ".pdf")

# ── Sidecar: headline JSON + provisional caption ──────────────────────
def p_exc(pol):        # P(R_{u*} > 1), pooled
    r = results[pol]["pooled"]
    return float((r > 1.0).mean())

def p99(pol):          # pooled p99 at the thesis cell
    r = results[pol]["pooled"]
    return float(np.quantile(r, 0.99))

head = {"cell": {"K": K_TESE, "N": N_TESE}, "burnin_slots": BURNIN,
        "seeds": SEEDS, "horizon_slots": T, "policies": {}}
ref_exc, ref_p99 = p_exc("mw-sw-v9"), p99("mw-sw-v9")
for pol in POL:
    head["policies"][LABEL[pol]] = {
        "u_star": results[pol]["u_star"],
        "P_exceed_envelope": p_exc(pol),
        "p99_Ru_ustar": p99(pol),
        "P_exceed_ratio_vs_RiskAware": p_exc(pol) / max(ref_exc, 1e-300),
        "p99_ratio_vs_RiskAware": p99(pol) / max(ref_p99, 1e-300),
    }
with open(base + ".json", "w") as f:
    json.dump(head, f, indent=2)

caption = (
    f"Fig. A. Empirical CCDF of the instantaneous composite risk $R_{{u^*}}(t)$ of the "
    f"worst UAV at $N={N_TESE}$ simultaneous UAVs and $K={K_TESE}$ channels, one curve per "
    f"scheduling policy. For each policy, $u^*$ maximises the post-burn-in temporal mean of "
    f"$R_u$ averaged over {SEEDS} independent seeds; the CCDF pools the {SEEDS} seeds' "
    f"post-burn-in samples (burn-in {BURNIN:,} slots, horizon $10^6$ slots per seed) and is "
    f"truncated where fewer than {MIN_EXC} exceedances remain. The dashed vertical line marks "
    f"the TLOS-calibrated design envelope $R_u=1$ under the saturated-radius renormalisation "
    f"$r^*=r_{{\\min}}+v_{{\\max}}\\tau_{{\\max}}$; the dotted horizontal line marks the "
    f"$10^{{-2}}$ (p99) level."
)
with open(base + "_caption.txt", "w") as f:
    f.write(caption + "\n")
print("sidecar:", base + ".json", "+ _caption.txt")
