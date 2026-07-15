#!/usr/bin/env python3
"""
plot_rc7_tls.py  -- Target-Level-of-Safety (TLS) figures for RC7, computed on the
SAME condition as the risk split: steady/full-load window only (>= C UAVs airborne,
warm-up ramp and drain tail dropped), scenario 4r_b7.

R_sys(t) is reconstructed from the per-(slot,drone) slot_series exactly as
core/step_risk.m: phase-aware per-UAV risk R_u = wu*r_unc + wm*r_map + wv*r_vid
with ramp weights (0.10,0.10,0.80) on the first/last RAMP=350 slots of each UAV's
flight and cruise weights (1/3,1/3,1/3) elsewhere; R_sys(t) = sum over active UAVs.
The steady window matches util/collect_results.m steady_window() (span between the
first and last slot with >= C UAVs airborne).

Figures (over the whole K x C grid):
  tls_ccdf_K{...}      : CCDF P(R_sys>r) per policy, panel per K at --ccdf-C, TLS line
  tls_envelope_heatmap : heatmap P(R_sys > TLS) over K x C, one panel per policy
  tls_exceed_vs_load   : P(R_sys > TLS) vs simultaneous UAVs, panel per K, per policy
  tls_maxRu_vs_load    : worst single-UAV risk max_u R_u vs load, panel per K (companion)

Stats are cached to <outdir>/tls_grid_stats.csv so re-plotting is instant.

Usage:
  python3 plot_rc7_tls.py --dir csv_export_RC7 --tls 500
  python3 plot_rc7_tls.py --dir csv_export_RC7 --tls 1000 --cached
"""
import os, glob, argparse
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.colors import LogNorm

ap = argparse.ArgumentParser()
ap.add_argument("--dir", default="csv_export_RC7")
ap.add_argument("--scenario", default="4r_b7")
ap.add_argument("--tls", type=float, default=500.0, help="Target Level of Safety on R_sys(t)")
ap.add_argument("--ccdf-C", type=int, default=100, help="load C used for the CCDF panels")
ap.add_argument("--outdir", default=None)
ap.add_argument("--cached", action="store_true", help="reuse tls_grid_stats.csv, skip reconstruction")
args = ap.parse_args()

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR   = os.path.join(SCRIPT_DIR, args.dir)
OUT_DIR    = args.outdir or os.path.join(DATA_DIR, "plots")
os.makedirs(OUT_DIR, exist_ok=True)
STATS_CSV  = os.path.join(OUT_DIR, "tls_grid_stats.csv")
RAMP = 350

K_VALUES = [1, 2, 4, 8, 16, 32]
C_VALUES = [5, 10, 20, 40, 60, 80, 100, 120]

POL       = ["round-robin-sw", "pf-classic-sw", "mr-mw-ch-sw", "mw-sw-otimo-ch", "mw-otimo-ch"]
POL_LABEL = {"round-robin-sw": "RR", "pf-classic-sw": "PF", "mr-mw-ch-sw": "heuristica",
             "mw-sw-otimo-ch": "mw(sw)", "mw-otimo-ch": "mw(no-sw)"}
COLORS    = {"round-robin-sw": "#F9A825", "pf-classic-sw": "#E53935", "mr-mw-ch-sw": "#8E24AA",
             "mw-sw-otimo-ch": "#1565C0", "mw-otimo-ch": "#00897B"}
MARKERS   = {"round-robin-sw": "o", "pf-classic-sw": "s", "mr-mw-ch-sw": "*",
             "mw-sw-otimo-ch": "P", "mw-otimo-ch": "X"}


def _latest(pattern):
    fs = sorted(glob.glob(pattern))          # timestamp in filename -> lexical sort = chronological
    return fs[-1] if fs else None


def _cell_files(K, C, pol):
    base = os.path.join(DATA_DIR, f"K{K}", f"C{C:02d}", args.scenario)
    ss = _latest(os.path.join(base, f"{pol}_*_slot_series.csv"))
    ds = _latest(os.path.join(base, f"{pol}_*_drone_summary.csv"))
    return ss, ds


def reconstruct(K, C, pol):
    """Return steady-window R_sys(t) array and worst single-UAV risk, or (None, nan)."""
    ss, ds = _cell_files(K, C, pol)
    if ss is None or ds is None:
        return None, np.nan
    d = pd.read_csv(ds, usecols=["drone_id", "entry_slot", "exit_slot"])
    df = pd.read_csv(ss, usecols=["slot", "drone_id", "r_unc", "r_map", "r_vid"],
                     dtype={"slot": np.int32, "drone_id": np.int32,
                            "r_unc": np.float32, "r_map": np.float32, "r_vid": np.float32})
    ent = dict(zip(d.drone_id, d.entry_slot))
    fl  = dict(zip(d.drone_id, (d.exit_slot - d.entry_slot)))
    loc = df.slot - df.drone_id.map(ent)
    flv = df.drone_id.map(fl)
    ramp = (loc < RAMP) | (loc > (flv - RAMP))
    wu = np.where(ramp, 0.10, 1/3); wm = wu; wv = np.where(ramp, 0.80, 1/3)
    df["Ru"] = wu * df.r_unc + wm * df.r_map + wv * df.r_vid
    g = df.groupby("slot")
    Rsys = g.Ru.sum().values
    nact = g.drone_id.size().values
    maxRu = float(df.Ru.max())
    # steady window: contiguous span with >= C airborne (fallback: >= 0.9*max)
    thr = C if (nact >= C).any() else max(1, int(0.9 * nact.max()))
    hit = np.where(nact >= thr)[0]
    if hit.size:
        Rsys = Rsys[hit[0]:hit[-1] + 1]
    return Rsys, maxRu


# ---------------------------------------------------------------- build/load stats
if args.cached and os.path.exists(STATS_CSV):
    stats = pd.read_csv(STATS_CSV)
    print(f"loaded cached stats: {STATS_CSV}")
else:
    rows = []
    for K in K_VALUES:
        for C in C_VALUES:
            for pol in POL:
                Rsys, maxRu = reconstruct(K, C, pol)
                if Rsys is None or Rsys.size == 0:
                    print(f"  MISSING  K{K} C{C} {pol}")
                    continue
                rows.append(dict(K=K, C=C, policy=pol,
                                 mean=float(Rsys.mean()),
                                 p95=float(np.quantile(Rsys, 0.95)),
                                 p99=float(np.quantile(Rsys, 0.99)),
                                 pmax=float(Rsys.max()),
                                 maxRu=maxRu,
                                 pexc_tls=float((Rsys > args.tls).mean()),
                                 nslots=int(Rsys.size)))
            print(f"  done K{K} C{C}")
    stats = pd.DataFrame(rows)
    stats.to_csv(STATS_CSV, index=False)
    print(f"saved {STATS_CSV}  ({len(stats)} cells)")

# recompute exceedance at the requested TLS (so --cached works with a new --tls only for CCDF;
# for heatmap/exceed we need the raw dist, so if --tls differs from cache, recompute pexc from CCDF cells)
POL = [p for p in POL if p in set(stats.policy.unique())]
TLS = args.tls
TLSTAG = f"_tls{int(TLS)}"

# =============================================================== FIG A: CCDF panels
def ccdf(x):
    xs = np.sort(np.asarray(x, float)); n = xs.size
    return xs, (n - 1 - np.arange(n)) / n

fig, axes = plt.subplots(2, 3, figsize=(12, 7.0), constrained_layout=True, squeeze=False)
axes = axes.flatten()
for ki, K in enumerate(K_VALUES):
    ax = axes[ki]
    for pol in POL:
        Rsys, _ = reconstruct(K, args.ccdf_C, pol)
        if Rsys is None or Rsys.size == 0:
            continue
        xs, pp = ccdf(Rsys); m = pp > 0
        ax.plot(xs[m], pp[m], color=COLORS[pol], lw=1.8, label=POL_LABEL[pol])
    ax.axvline(TLS, color="k", ls="-.", lw=1.2)
    ax.text(TLS, 0.5, " TLS", rotation=90, va="center", fontsize=8)
    ax.set_yscale("log"); ax.set_xscale("log")
    ax.set_title(f"K = {K}", fontweight="bold")
    if ki % 3 == 0:
        ax.set_ylabel(r"$P(R_{sys} > r)$")
    if ki >= 3:
        ax.set_xlabel(r"System risk $R_{sys}(t)$")
    ax.grid(True, which="both", ls="--", alpha=0.35)
handles = [Line2D([0], [0], color=COLORS[p], lw=2, label=POL_LABEL[p]) for p in POL]
handles.append(Line2D([0], [0], color="k", ls="-.", lw=1.2, label=f"TLS = {int(TLS)}"))
fig.legend(handles=handles, ncol=len(handles), loc="lower center",
           bbox_to_anchor=(0.5, -0.03), frameon=False, fontsize=10)
fig.suptitle(f"CCDF of system risk (steady window, C = {args.ccdf_C})", fontweight="bold")
base = os.path.join(OUT_DIR, f"tls_ccdf_C{args.ccdf_C}{TLSTAG}")
for ext in ("pdf", "png"):
    fig.savefig(f"{base}.{ext}", dpi=150, bbox_inches="tight")
print("saved", base)
plt.close(fig)

# =============================================================== FIG B: envelope heatmap
# P(R_sys > TLS) over K x C, one panel per policy.
fig, axes = plt.subplots(1, len(POL), figsize=(3.1*len(POL), 3.4),
                         constrained_layout=True, squeeze=False)
axes = axes.flatten()
for pi, pol in enumerate(POL):
    ax = axes[pi]
    M = np.full((len(K_VALUES), len(C_VALUES)), np.nan)
    for i, K in enumerate(K_VALUES):
        for j, C in enumerate(C_VALUES):
            r = stats[(stats.K == K) & (stats.C == C) & (stats.policy == pol)]
            if len(r):
                M[i, j] = 100.0 * r.pexc_tls.values[0]
    im = ax.imshow(M, aspect="auto", origin="lower", cmap="magma", vmin=0, vmax=100)
    ax.set_xticks(range(len(C_VALUES))); ax.set_xticklabels(C_VALUES, fontsize=7)
    ax.set_yticks(range(len(K_VALUES))); ax.set_yticklabels(K_VALUES, fontsize=8)
    ax.set_title(POL_LABEL[pol], fontweight="bold")
    ax.set_xlabel("Simultaneous UAVs  C")
    if pi == 0:
        ax.set_ylabel("K")
    for i in range(len(K_VALUES)):
        for j in range(len(C_VALUES)):
            if not np.isnan(M[i, j]):
                ax.text(j, i, f"{M[i,j]:.0f}", ha="center", va="center",
                        fontsize=6, color="white" if M[i, j] < 60 else "black")
cbar = fig.colorbar(im, ax=axes.tolist(), shrink=0.85)
cbar.set_label(f"P(R_sys > {int(TLS)})  [%]")
fig.suptitle(f"TLS envelope: exceedance probability over the K x C grid (TLS = {int(TLS)}, steady window)",
             fontweight="bold", fontsize=10)
base = os.path.join(OUT_DIR, f"tls_envelope_heatmap{TLSTAG}")
for ext in ("pdf", "png"):
    fig.savefig(f"{base}.{ext}", dpi=150, bbox_inches="tight")
print("saved", base)
plt.close(fig)

# =============================================================== FIG C: exceed vs load
fig, axes = plt.subplots(2, 3, figsize=(12, 7.0), constrained_layout=True, squeeze=False)
axes = axes.flatten()
for ki, K in enumerate(K_VALUES):
    ax = axes[ki]
    for pol in POL:
        r = stats[(stats.K == K) & (stats.policy == pol)].sort_values("C")
        if len(r):
            ax.plot(r.C, 100.0 * r.pexc_tls, color=COLORS[pol], marker=MARKERS[pol],
                    ms=6, lw=2.0, label=POL_LABEL[pol])
    ax.set_title(f"K = {K}", fontweight="bold")
    ax.set_xticks(C_VALUES); ax.tick_params(axis="x", labelsize=7)
    ax.set_ylim(-3, 103)
    if ki % 3 == 0:
        ax.set_ylabel(f"P(R_sys > {int(TLS)})  [%]")
    if ki >= 3:
        ax.set_xlabel("Simultaneous UAVs  C")
    ax.grid(True, ls="--", alpha=0.4)
handles = [Line2D([0], [0], color=COLORS[p], marker=MARKERS[p], lw=2, label=POL_LABEL[p]) for p in POL]
fig.legend(handles=handles, ncol=len(handles), loc="lower center",
           bbox_to_anchor=(0.5, -0.03), frameon=False, fontsize=10)
fig.suptitle(f"TLS exceedance vs load (TLS = {int(TLS)}, steady window)", fontweight="bold")
base = os.path.join(OUT_DIR, f"tls_exceed_vs_load{TLSTAG}")
for ext in ("pdf", "png"):
    fig.savefig(f"{base}.{ext}", dpi=150, bbox_inches="tight")
print("saved", base)
plt.close(fig)

# =============================================================== FIG D: max_u R_u vs load (companion)
fig, axes = plt.subplots(2, 3, figsize=(12, 7.0), constrained_layout=True, squeeze=False)
axes = axes.flatten()
for ki, K in enumerate(K_VALUES):
    ax = axes[ki]
    for pol in POL:
        r = stats[(stats.K == K) & (stats.policy == pol)].sort_values("C")
        if len(r):
            ax.plot(r.C, r.maxRu, color=COLORS[pol], marker=MARKERS[pol],
                    ms=6, lw=2.0, label=POL_LABEL[pol])
    ax.set_title(f"K = {K}", fontweight="bold"); ax.set_yscale("log")
    ax.set_xticks(C_VALUES); ax.tick_params(axis="x", labelsize=7)
    if ki % 3 == 0:
        ax.set_ylabel(r"$\max_u R_u$")
    if ki >= 3:
        ax.set_xlabel("Simultaneous UAVs  C")
    ax.grid(True, which="both", ls="--", alpha=0.4)
handles = [Line2D([0], [0], color=COLORS[p], marker=MARKERS[p], lw=2, label=POL_LABEL[p]) for p in POL]
fig.legend(handles=handles, ncol=len(handles), loc="lower center",
           bbox_to_anchor=(0.5, -0.03), frameon=False, fontsize=10)
fig.suptitle("Worst-case single-UAV risk vs load (steady window)", fontweight="bold")
base = os.path.join(OUT_DIR, "tls_maxRu_vs_load")
for ext in ("pdf", "png"):
    fig.savefig(f"{base}.{ext}", dpi=150, bbox_inches="tight")
print("saved", base)
plt.close(fig)
