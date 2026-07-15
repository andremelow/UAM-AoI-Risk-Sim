#!/usr/bin/env python3
"""
INFOCOM figures for the Risk-Aware Max-Weight paper (RC5, route-B, K=1, C=100).

Fig 1  ccdf_system_risk        : empirical CCDF P(R_sys(t) > r), 3 policies, y-log + log-log.
Fig 2  per_uav_profile_{mean,p95}: per-UAV risk sorted by RR exposure, RR vs Risk-Aware.
Fig 3  max_uav_vs_load         : sweep of global max_u R_u(t) vs load C, K=1, per policy.

Design notes (kept in code, not on the figure):
- R_sys(t) is NOT stored by the sim; we reconstruct it as sum over active route-B
  UAVs of the phase-aware per-UAV risk R_u(t) = we_unc*r_unc + we_map*r_map +
  we_vid*r_vid, with ramp weights (0.10,0.10,0.80) on the first/last RAMP=350 slots
  and cruise weights (1/3,1/3,1/3) elsewhere -- matching core/step_risk.m. omega=1.
- Route B = RouteBhalf UAVs (flight duration <= 4000 slots), the hotspot sub-population.
- Grayscale-safe: each policy has a distinct linestyle AND marker (not color alone).
- IEEE single-column width ~3.5 in; style inherited from plot_risk_analysis.py
  (sans-serif, size ~ , spines off, dpi 150), PDF vector output (no PGF in repo).
- Premise note: risk here is CONVEX (R_gnd ~ r_u^2), so redistribution lowers BOTH
  mean and tail (Jensen); the tail (right side of the CCDF) is where it is dramatic.
"""
import os, glob, argparse
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

ap = argparse.ArgumentParser()
ap.add_argument("--dir", default="csv_export_RC5")
ap.add_argument("--K", type=int, default=1)
ap.add_argument("--C", type=int, default=100)
ap.add_argument("--scenario", default="2r_b23")
ap.add_argument("--outdir", default="csv_export_RC5_routeB/plots")
ap.add_argument("--thr", type=float, default=None, help="optional TLS vertical line")
ap.add_argument("--scope", default="routeB", choices=["routeB", "system"],
                help="routeB = only RouteBhalf UAVs; system = all UAVs (both routes)")
args = ap.parse_args()
SYSTEM = (args.scope == "system")
STAG = "" if not SYSTEM else "_system"   # filename suffix for system-scope figures
os.makedirs(args.outdir, exist_ok=True)
RAMP = 350
FLEN_B = 4000   # route-B = flight duration <= 4000 slots

# --- IEEE single-column style (inherit repo conventions) ---
plt.rcParams.update({
    "font.family": "sans-serif", "font.size": 8,
    "axes.titlesize": 8, "axes.labelsize": 8,
    "legend.fontsize": 7, "xtick.labelsize": 7, "ytick.labelsize": 7,
    "figure.dpi": 150, "axes.spines.top": False, "axes.spines.right": False,
    "lines.linewidth": 1.3, "legend.frameon": False,
})
COL_W, COL_H = 3.5, 2.5   # inches, single column

# policy -> (label, color, linestyle, marker)  -- grayscale-safe via ls+marker
POL = {
    "round-robin-sw": ("RR",         "#7a7a7a", "-",  "o"),
    "pf-classic-sw":  ("PF",         "#d62728", "--", "s"),
    "mr-mw-ch-sw":    ("Risk-Aware", "#6a3d9a", ":",  "*"),
}
ORDER = ["round-robin-sw", "pf-classic-sw", "mr-mw-ch-sw"]

def _cell_files(pol, C):
    base = os.path.join(args.dir, f"K{args.K}", f"C{C}", args.scenario)
    ss = glob.glob(os.path.join(base, f"{pol}_*_slot_series.csv"))
    ds = glob.glob(os.path.join(base, f"{pol}_*_drone_summary.csv"))
    return (ss[0], ds[0]) if ss and ds else (None, None)

def _load_phase_aware(pol, C):
    """Return per-(slot,drone) route-B dataframe with phase-aware R_u."""
    ss, ds = _cell_files(pol, C)
    if ss is None:
        return None
    d = pd.read_csv(ds, usecols=["drone_id", "entry_slot", "exit_slot"])
    d["flen"] = d.exit_slot - d.entry_slot
    keep = set(d.drone_id) if SYSTEM else set(d.loc[d.flen <= FLEN_B, "drone_id"])
    ent = dict(zip(d.drone_id, d.entry_slot)); fl = dict(zip(d.drone_id, d.flen))
    df = pd.read_csv(ss, usecols=["slot", "drone_id", "r_unc", "r_map", "r_vid"])
    df = df[df.drone_id.isin(keep)].copy()
    loc = df.slot - df.drone_id.map(ent); flv = df.drone_id.map(fl)
    ramp = (loc < RAMP) | (loc > (flv - RAMP))
    wu = np.where(ramp, 0.10, 1/3); wm = np.where(ramp, 0.10, 1/3); wv = np.where(ramp, 0.80, 1/3)
    df["Ru"] = wu*df.r_unc + wm*df.r_map + wv*df.r_vid
    return df

def _sys_series(df):
    """R_sys^B(t) = sum over active route-B UAVs per slot."""
    return df.groupby("slot").Ru.sum().values

def ccdf(x):
    xs = np.sort(np.asarray(x, float))
    n = xs.size
    # P(X > xs[i]) = (n-1-i)/n ; complementary cumulative, no binning
    p = (n - 1 - np.arange(n)) / n
    return xs, p

# ============================ FIG 1: CCDF ============================
series = {p: _sys_series(_load_phase_aware(p, args.C)) for p in ORDER}

def _plot_ccdf(loglog, fname):
    fig, ax = plt.subplots(figsize=(COL_W, COL_H))
    for p in ORDER:
        lab, c, ls, mk = POL[p]
        xs, pp = ccdf(series[p])
        m = pp > 0
        xs, pp = xs[m], pp[m]
        me = max(1, xs.size // 10)
        ax.plot(xs, pp, color=c, ls=ls, marker=mk, markevery=me, ms=3.5,
                markerfacecolor="none", label=lab)
    ax.set_yscale("log")
    if loglog:
        ax.set_xscale("log")
    else:
        ax.ticklabel_format(axis="x", style="sci", scilimits=(0, 0))
    if args.thr is not None:
        ax.axvline(args.thr, color="k", lw=0.9, ls="-.")
        ax.text(args.thr, 0.5, " TLS", fontsize=6, rotation=90, va="center")
    ax.set_xlabel(r"System risk $R_{\mathrm{sys}}$")
    ax.set_ylabel(r"$P(R_{\mathrm{sys}} > r)$")
    ax.set_title("CCDF of system risk")
    ax.grid(True, which="both", ls="--", alpha=0.35)
    ax.legend(loc="lower left")
    fig.tight_layout(pad=0.4)
    for ext in ("pdf", "png"):
        fig.savefig(os.path.join(args.outdir, f"{fname}.{ext}"), bbox_inches="tight")
    plt.close(fig)
    print("saved", fname)

_plot_ccdf(False, "ccdf_system_risk"+STAG)
_plot_ccdf(True,  "ccdf_system_risk_loglog"+STAG)

# ==================== FIG 2: per-UAV profile ====================
# per-UAV stat over its flight, indexed by drone_id; sort by RR exposure.
def per_uav_stat(pol, stat):
    df = _load_phase_aware(pol, args.C)
    g = df.groupby("drone_id").Ru
    return g.mean() if stat == "mean" else g.quantile(0.95)

def _plot_peruav(stat, fname):
    rr = per_uav_stat("round-robin-sw", stat)
    ra = per_uav_stat("mr-mw-ch-sw", stat)
    common = rr.index.intersection(ra.index)
    rr = rr.loc[common]; ra = ra.loc[common]
    order = rr.sort_values(ascending=False).index      # sort by RR exposure
    rr_s = rr.loc[order].values; ra_s = ra.loc[order].values
    x = np.arange(1, len(order) + 1)
    fig, ax = plt.subplots(figsize=(COL_W, COL_H))
    lab, c, ls, mk = POL["round-robin-sw"]
    ax.plot(x, rr_s, color=c, ls=ls, marker=mk, ms=3, markevery=max(1, len(x)//12),
            markerfacecolor="none", label="RR")
    lab, c, ls, mk = POL["mr-mw-ch-sw"]
    ax.plot(x, ra_s, color=c, ls=ls, marker=mk, ms=4, markevery=max(1, len(x)//12),
            markerfacecolor="none", label="Risk-Aware")
    ax.set_yscale("log")
    ax.set_xlabel("UAV index (sorted by RR exposure)")
    ax.set_ylabel(rf"Per-UAV risk $R_u$  ({'mean' if stat=='mean' else 'P95'})")
    ax.set_title("Per-UAV risk: RR vs Risk-Aware")
    ax.grid(True, which="both", ls="--", alpha=0.35)
    ax.legend(loc="upper right")
    fig.tight_layout(pad=0.4)
    for ext in ("pdf", "png"):
        fig.savefig(os.path.join(args.outdir, f"{fname}.{ext}"), bbox_inches="tight")
    plt.close(fig)
    print("saved", fname)

_plot_peruav("mean", "per_uav_profile_mean"+STAG)
_plot_peruav("p95",  "per_uav_profile_p95"+STAG)

# ==================== FIG 3: max_u R_u(t) vs load ====================
Cs = [20, 40, 60, 80, 100, 120]
maxvals = {p: [] for p in ORDER}
for C in Cs:
    for p in ORDER:
        df = _load_phase_aware(p, C)
        # global worst single-UAV risk over the whole horizon
        maxvals[p].append(np.nan if df is None else df.Ru.max())
fig, ax = plt.subplots(figsize=(COL_W, COL_H))
for p in ORDER:
    lab, c, ls, mk = POL[p]
    ax.plot(Cs, maxvals[p], color=c, ls=ls, marker=mk, ms=4,
            markerfacecolor="none", label=lab)
ax.set_yscale("log")
ax.set_xlabel("Simultaneous UAVs (load)")
ax.set_ylabel(r"$\max_u R_u(t)$")
ax.set_title("Worst-case single-UAV risk vs load")
ax.set_xticks(Cs)
ax.grid(True, which="both", ls="--", alpha=0.35)
ax.legend(loc="upper left")
fig.tight_layout(pad=0.4)
for ext in ("pdf", "png"):
    fig.savefig(os.path.join(args.outdir, f"max_uav_vs_load{STAG}.{ext}"), bbox_inches="tight")
plt.close(fig)
print("saved max_uav_vs_load")

# ---- console readout: P(exceed) at a few quantiles, mean preservation check ----
print("\n=== route-B R_sys summary (K={}, C={}) ===".format(args.K, args.C))
for p in ORDER:
    s = series[p]
    print(f"  {POL[p][0]:11s} mean={s.mean():10.0f}  P95={np.quantile(s,0.95):10.0f}  "
          f"P99={np.quantile(s,0.99):10.0f}  max={s.max():10.0f}")
