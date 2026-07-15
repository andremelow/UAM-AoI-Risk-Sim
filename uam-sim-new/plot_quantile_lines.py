#!/usr/bin/env python3
"""plot_quantile_lines.py -- worst-UAV quantile of the composite risk as LINES:
one panel per K, x = N (simultaneous UAVs), one line per policy,
y = max_u P_q(R_u(t)) (per-UAV temporal quantile q, then max over the fleet).

Companion to p99_heatmap in line form; default q = 0.95. TLS line at 1.
Per-(q, mask) stats cache: <dir>/plots/qstats_q{q}[_noland].csv.

Usage:
  python3 plot_quantile_lines.py --dir csv_export_RC11 --q 0.95 \
      --pol round-robin-sw,pf-classic-sw,mw-sw-v9,mw-v9-nsw [--exclude-landing]
Output: <dir>/plots/fig_q{q}_lines[_noland].{pdf,png}
"""
import os, glob, argparse
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

ap = argparse.ArgumentParser()
ap.add_argument("--dir", default="csv_export_RC11")
ap.add_argument("--scenario", default="4r_b7")
ap.add_argument("--q", type=float, default=0.95)
ap.add_argument("--metric", choices=["quantile", "exceed"], default="quantile",
                help="quantile: max_u P_q(R_u); exceed: max_u P_t(R_u > tls) [%%]")
ap.add_argument("--tls", type=float, default=1.0)
ap.add_argument("--pol", default="round-robin-sw,pf-classic-sw,mw-sw-v9,mw-v9-nsw")
ap.add_argument("--min-slots", type=int, default=10)
ap.add_argument("--exclude-landing", action="store_true")
ap.add_argument("--cached", action="store_true")
args = ap.parse_args()

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(SCRIPT_DIR, args.dir)
OUT  = os.path.join(DATA, "plots")
os.makedirs(OUT, exist_ok=True)
QTAG = f"q{int(round(args.q*100))}" if args.metric == "quantile" else "exceed"
LTAG = "_noland" if args.exclude_landing else ""
STATS_CSV = os.path.join(OUT, f"qstats_{QTAG}{LTAG}.csv")

K_VALUES = [1, 2, 4, 8, 16, 32]
C_VALUES = [5, 10, 20, 40, 60, 80, 100, 120]
POL    = [p for p in args.pol.split(",") if p]
LABEL  = {"round-robin-sw": "RR", "pf-classic-sw": "PF", "mr-mw-ch-sw": "Risk-Aware",
          "max-risk-pf-sw": "Risk-PF", "mw-sw-otimo-ch": "MW-opt (sw)",
          "mw-otimo-ch": "MW-opt", "mw-sw-v8": "MW-V8", "mw-sw-v9": "MW-V9",
          "mw-v9-nsw": "MW-V9-nsw"}
COLORS = {"round-robin-sw": "#F9A825", "pf-classic-sw": "#E53935", "mr-mw-ch-sw": "#8E24AA",
          "max-risk-pf-sw": "#1E88E5", "mw-sw-otimo-ch": "#3949AB",
          "mw-otimo-ch": "#6D4C41", "mw-sw-v8": "#00897B", "mw-sw-v9": "#7CB342",
          "mw-v9-nsw": "#33691E"}
MARKERS = {"round-robin-sw": "o", "pf-classic-sw": "s", "mr-mw-ch-sw": "*",
           "max-risk-pf-sw": "D", "mw-sw-otimo-ch": "v", "mw-otimo-ch": "P",
           "mw-sw-v8": "X", "mw-sw-v9": "^", "mw-v9-nsw": "v"}

def cell_q(K, C, pol):
    fs = sorted(glob.glob(os.path.join(DATA, f"K{K}", f"C{C:02d}", args.scenario,
                                        f"{pol}_*_slot_series.csv")))
    if not fs:
        return np.nan
    cols = ["drone_id", "r_tot"] + (["slot"] if args.exclude_landing else [])
    df = pd.read_csv(fs[-1], usecols=cols,
                     dtype={"drone_id": np.int32, "r_tot": np.float32})
    if args.exclude_landing:
        dsf = fs[-1].replace("_slot_series.csv", "_drone_summary.csv")
        meta = pd.read_csv(dsf, usecols=["drone_id", "entry_slot", "exit_slot"])
        meta["dur"] = meta.exit_slot - meta.entry_slot
        fr = np.where(meta.dur > meta.dur.min() * 1.5, 500.0/8000.0, 500.0/4264.4)
        meta["cut"] = meta.entry_slot + ((1.0 - fr) * meta.dur).astype(int)
        df = df.merge(meta[["drone_id", "cut"]], on="drone_id")
        df = df[df.slot < df.cut]
    g = df.groupby("drone_id").r_tot
    if args.metric == "exceed":
        v = g.apply(lambda s: 100.0 * (s > args.tls).mean())
    else:
        v = g.quantile(args.q)
    v = v[g.size() >= args.min_slots]
    return float(v.max()) if len(v) else np.nan

if args.cached and os.path.exists(STATS_CSV):
    stats = pd.read_csv(STATS_CSV)
    print("loaded", STATS_CSV)
else:
    rows = []
    for K in K_VALUES:
        for C in C_VALUES:
            for pol in POL:
                v = cell_q(K, C, pol)
                if np.isnan(v):
                    print(f"  MISSING K{K} C{C} {pol}")
                    continue
                rows.append(dict(K=K, C=C, policy=pol, max_q=v))
        print(f"  done K{K}", flush=True)
    stats = pd.DataFrame(rows)
    stats.to_csv(STATS_CSV, index=False)
    print("saved", STATS_CSV)

plt.rcParams.update({"font.family": "serif",
                     "font.serif": ["Times New Roman", "Nimbus Roman", "DejaVu Serif"],
                     "mathtext.fontset": "stix", "font.size": 8,
                     "xtick.labelsize": 6.5, "ytick.labelsize": 7, "pdf.fonttype": 42})
fig, axes = plt.subplots(1, len(K_VALUES), figsize=(2.0*len(K_VALUES), 2.6),
                         sharey=True, constrained_layout=True, squeeze=False)
axes = axes.flatten()

for ki, K in enumerate(K_VALUES):
    ax = axes[ki]
    for pol in POL:
        d = stats[(stats.K == K) & (stats.policy == pol)].sort_values("C")
        ax.plot(d.C, d.max_q, color=COLORS[pol], marker=MARKERS[pol], ms=4.5, lw=1.6)
    if args.metric == "quantile":
        ax.axhline(1.0, ls=(0, (4, 3)), lw=0.8, color="0.35")
        if ki == len(K_VALUES) - 1:
            ax.text(0.97, 1.0, " TLS", transform=ax.get_yaxis_transform(),
                    ha="left", va="bottom", fontsize=6.5, color="0.35", clip_on=False)
        ax.set_yscale("log")
    else:
        ax.set_ylim(-3, 78)
    ax.set_xscale("log")
    ax.set_xticks(C_VALUES)
    ax.set_xticklabels(C_VALUES, rotation=90)
    ax.minorticks_off()
    ax.set_title(f"K = {K}", fontweight="bold")
    ax.set_xlabel("Simultaneous UAVs")
    if ki == 0:
        if args.metric == "exceed":
            ax.set_ylabel(r"$\max_u\ P_t(R_u > 1)$  [%]")
        else:
            ax.set_ylabel(rf"$\max_u\ P_{{{int(round(args.q*100))}}}$ of $R_u$")
    ax.grid(True, which="both", ls=":", lw=0.4, alpha=0.6)

handles = [Line2D([0], [0], color=COLORS[p], marker=MARKERS[p], lw=1.6, ms=4.5,
                  label=LABEL.get(p, p)) for p in POL]
fig.legend(handles=handles, ncol=len(POL), loc="lower center",
           bbox_to_anchor=(0.5, -0.08), frameon=False, fontsize=8)

base = os.path.join(OUT, f"fig_{QTAG}_lines{LTAG}")
for ext in ("pdf", "png"):
    fig.savefig(f"{base}.{ext}", dpi=170, bbox_inches="tight")
    print("saved", f"{base}.{ext}")
