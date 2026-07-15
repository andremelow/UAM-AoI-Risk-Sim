#!/usr/bin/env python3
"""FIG B -- fig_p99_envelope_K{K_ENV}.pdf  (IEEE single column, vector PDF)

Worst-UAV p99 of the composite risk vs the number of simultaneous UAVs N, at
K = K_ENV channels, one curve per policy:
  per UAV per seed: temporal p99 of R_u(t) (t > BURNIN)
  -> mean across the SEEDS seeds per UAV
  -> maximum across UAVs (the plotted value).
Error bars: two-sided 95% t-interval over the SEEDS per-seed p99 values of the
argmax UAV. Horizontal line at R = 1 (TLOS-calibrated envelope, attested by the
dataset validation). N*_p99 per policy = largest N with the value <= 1,
annotated on the figure. NO lower-bound (L_B) content.

Sidecar: fig_p99_envelope_K{K_ENV}.json + _caption.txt.

Usage:  python3 plot_paper_p99_envelope.py [--dir csv_export_RC12]
"""
import os, json, argparse
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
import fig_style
import paper_data_checks as pdc

# ── Parameters ────────────────────────────────────────────────────────
K_ENV  = 1
N_GRID = [5, 10, 20, 40, 60, 80, 100, 120]
BURNIN = int(1e4)
SEEDS  = 10
T      = int(1e6)

# two-sided 95% Student-t critical values, df = 1..29 (no scipy in pipeline)
T975 = {1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571, 6: 2.447, 7: 2.365,
        8: 2.306, 9: 2.262, 10: 2.228, 11: 2.201, 12: 2.179, 14: 2.145,
        19: 2.093, 24: 2.064, 29: 2.045}

POL   = pdc.REQUIRED_POLICIES
LABEL = {"round-robin-sw": "RR", "pf-classic-sw": "PF",
         "mw-sw-v9": "Risk-Aware", "mw-v9-nsw": "Risk-Aware (non-preempt.)"}

ap = argparse.ArgumentParser()
ap.add_argument("--dir", default="csv_export_RC12")
ap.add_argument("--scenario", default="4r_b7")
ap.add_argument("--seeds", type=int, default=SEEDS)
ap.add_argument("--T", type=float, default=T)
ap.add_argument("--burnin", type=float, default=BURNIN)
ap.add_argument("--min-n", type=int, default=100,
                help="drop UAVs with fewer post-burn-in samples")
ap.add_argument("--allow-unattested-renorm", action="store_true")
args = ap.parse_args()
SEEDS, T, BURNIN = args.seeds, int(args.T), int(args.burnin)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(SCRIPT_DIR, args.dir)
OUT  = os.path.join(DATA, "plots")
CACHE = os.path.join(OUT, "paper_cache")
os.makedirs(OUT, exist_ok=True)

# ── Validations 1-3 (abort inside on failure) ─────────────────────────
cells = [(K_ENV, N) for N in N_GRID]
runs_by = pdc.validate(DATA, args.scenario, cells, SEEDS, T,
                       strict_renorm=not args.allow_unattested_renorm)

# ── Metric: per UAV p99 per seed -> seed mean -> max over UAVs ────────
curve = {p: {"y": [], "lo": [], "hi": []} for p in POL}
for pol in POL:
    for N in N_GRID:
        runs = runs_by[(K_ENV, N, pol)][:SEEDS]
        stats = [pdc.uav_stats(r, BURNIN, CACHE) for r in runs]
        stats = [st[st.n >= args.min_n] for st in stats]
        pp = stats[0][["drone_id", "p99_r"]].rename(columns={"p99_r": "s0"})
        for i, st in enumerate(stats[1:], 1):
            pp = pp.merge(st[["drone_id", "p99_r"]].rename(columns={"p99_r": f"s{i}"}),
                          on="drone_id", how="inner")
        vals = pp.drop(columns="drone_id").values          # UAV x seed
        seed_mean = vals.mean(axis=1)
        i_star = int(seed_mean.argmax())
        y = float(seed_mean[i_star])
        s = vals[i_star]                                   # the argmax UAV's seeds
        if len(s) > 1:
            tcrit = T975.get(len(s) - 1, 2.045)
            half = tcrit * s.std(ddof=1) / np.sqrt(len(s))
        else:
            half = 0.0                      # single seed: no CI
        curve[pol]["y"].append(y)
        curve[pol]["lo"].append(max(y - half, 1e-12))
        curve[pol]["hi"].append(y + half)

def n_star(pol):
    ok = [N for N, y in zip(N_GRID, curve[pol]["y"]) if y <= 1.0]
    return max(ok) if ok else None

# ── Figure ────────────────────────────────────────────────────────────
fig_style.apply()
fig, ax = plt.subplots(figsize=(3.5, 2.6), constrained_layout=True)

for pol in POL:
    y = np.array(curve[pol]["y"])
    lo = np.array(curve[pol]["lo"]); hi = np.array(curve[pol]["hi"])
    kw = fig_style.line_kwargs(pol)
    kw["label"] = LABEL[pol]
    ax.errorbar(N_GRID, y, yerr=[y - lo, hi - y], capsize=1.8, elinewidth=0.7,
                **kw)

ax.axhline(1.0, ls=(0, (4, 3)), lw=0.9, color="0.35")
ax.text(0.985, 1.0, "$R=1$", transform=ax.get_yaxis_transform(),
        ha="right", va="bottom", fontsize=7, color="0.35")

# N*_p99 annotations (stacked bottom-left, one line per policy)
notes = []
for pol in POL:
    ns = n_star(pol)
    notes.append(f"$N^*_{{p99}}$({LABEL[pol]}) = {ns if ns is not None else '--'}")
ax.text(0.02, 0.03, "\n".join(notes), transform=ax.transAxes, fontsize=6.2,
        va="bottom", ha="left", color="0.15")

ax.set_yscale("log")
ax.set_xscale("log")
ax.set_xticks(N_GRID)
ax.xaxis.set_major_formatter(mticker.FixedFormatter([str(n) for n in N_GRID]))
ax.xaxis.set_minor_formatter(mticker.NullFormatter())
ax.tick_params(axis="x", which="minor", length=0)
ax.set_xlabel("Number of UAVs, $N$")
ax.set_ylabel(r"$\max_u\ \mathrm{p99}$ of $R_u$")
ax.grid(True, which="both", ls=":", lw=0.4, alpha=0.6)
ax.legend(loc="upper left", fontsize=6.5)

base = os.path.join(OUT, f"fig_p99_envelope_K{K_ENV}")
fig.savefig(base + ".pdf", bbox_inches="tight")
fig.savefig(base + ".png", dpi=200, bbox_inches="tight")
print("saved", base + ".pdf")

# ── Sidecar: headline JSON + provisional caption ──────────────────────
head = {"K": K_ENV, "N_grid": N_GRID, "burnin_slots": BURNIN, "seeds": SEEDS,
        "horizon_slots": T, "policies": {}}
ref = {N: y for N, y in zip(N_GRID, curve["mw-sw-v9"]["y"])}
for pol in POL:
    head["policies"][LABEL[pol]] = {
        "N_star_p99": n_star(pol),
        "p99_by_N": {str(N): y for N, y in zip(N_GRID, curve[pol]["y"])},
        "p99_ratio_vs_RiskAware_by_N":
            {str(N): y / max(ref[N], 1e-300)
             for N, y in zip(N_GRID, curve[pol]["y"])},
    }
with open(base + ".json", "w") as f:
    json.dump(head, f, indent=2)

caption = (
    f"Fig. B. Worst-UAV 99th percentile of the composite risk versus the number of "
    f"simultaneous UAVs $N$, at $K={K_ENV}$ channels. For each policy and each $N$: the "
    f"temporal p99 of $R_u(t)$ is computed per UAV and per seed over the post-burn-in "
    f"horizon (burn-in {BURNIN:,} slots, $10^6$ slots per seed), averaged across "
    f"{SEEDS} independent seeds per UAV, and maximised across UAVs. Error bars are "
    f"two-sided 95% Student-$t$ intervals over the {SEEDS} per-seed p99 values of the "
    f"argmax UAV. The dashed line marks the TLOS-calibrated envelope $R=1$ under the "
    f"saturated-radius renormalisation $r^*=r_{{\\min}}+v_{{\\max}}\\tau_{{\\max}}$; "
    f"$N^*_{{p99}}$ is the largest $N$ whose value does not exceed it."
)
with open(base + "_caption.txt", "w") as f:
    f.write(caption + "\n")
print("sidecar:", base + ".json", "+ _caption.txt")
