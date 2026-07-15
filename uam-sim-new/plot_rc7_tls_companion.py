#!/usr/bin/env python3
"""
plot_rc7_tls_companion.py -- companion to the TLS envelope for RC7.

CORRECT per-UAV metric (not fleet mean, not pooled across UAVs):
  For each UAV u, R̄_u = time-average of the NORMALIZED phase-aware risk R_u(t)
  over the steady/full-load window (warm-up ramp and drain tail dropped).
  Cell metric = max_u R̄_u.

R_u(t) is read straight from the slot_series column `r_tot`, which the simulator
already writes as the phase-aware normalized per-UAV risk
  R_u = w_eff_unc·(R_nav/R̄_nav) + w_eff_map·(R_gnd/R̄_gnd) + w_eff_vid·R_vid
with altitude-based ramp weights (core/step_risk.m:106). We therefore do NOT
re-derive weights (the previous script's 350-slot ramp guess is avoided).

Steady window matches util/collect_results.m steady_window(): the contiguous span
between the first and last slot with >= C UAVs airborne.

Figures (3 panels: RR, PF, Risk-Aware; axes C x K identical to the RC7 envelope):
  tls_companion_maxRbar : heatmap of max_u R̄_u, cell values, diverging colormap
                          centered at 1 (log color if range > 1 decade), with the
                          black solid acceptance contour max_u R̄_u = 1 labeled "TLS".
  tls_exceed_maxPu      : CORRECTED exceedance heatmap -- worst-UAV time-fraction
                          above the normalized TLS, max_u P_t(R_u(t) > 1). Replaces
                          the old pooled P(R_sys > 500) heatmap (see note in console).

Console: per policy, the acceptance frontier = largest C with max_u R̄_u <= 1, per K.

Usage:  python3 plot_rc7_tls_companion.py --dir csv_export_RC7
"""
import os, glob, argparse
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import TwoSlopeNorm
from matplotlib.lines import Line2D

ap = argparse.ArgumentParser()
ap.add_argument("--dir", default="csv_export_RC7")
ap.add_argument("--scenario", default="4r_b7")
ap.add_argument("--tls", type=float, default=1.0, help="normalized acceptance level on R_u (default 1)")
ap.add_argument("--min-slots", type=int, default=10, help="drop UAVs with fewer steady slots (edge guard)")
ap.add_argument("--run", choices=["first", "latest"], default="latest",
                help="which run to use per cell when multiple timestamps exist "
                     "(first = original/earliest; runs are byte-identical, so this only fixes provenance)")
ap.add_argument("--window", choices=["flight", "steady"], default="flight",
                help="flight = per-UAV mean over the whole flight (drone_summary.mean_r_tot, "
                     "matches the original figure); steady = restrict to the >=C full-load span "
                     "(discards warm-up ramp, tighter frontier)")
ap.add_argument("--pol", default="round-robin-sw,pf-classic-sw,mr-mw-ch-sw,max-risk-pf-sw",
                help="comma-separated policy list (e.g. add mw-sw-v8 after merging RC9)")
ap.add_argument("--exclude-ramps", action="store_true",
                help="mask BOTH ramps (first and final 500 m of each UAV's route: "
                     "climb-out and landing outage spikes); suffix _noramps")
ap.add_argument("--exclude-landing", action="store_true",
                help="mask each UAV's landing ramp (final 500 m of its route, "
                     "outage-driven risk spike) before computing R̄_u and the "
                     "exceedance; output files get the _noland suffix")
ap.add_argument("--outbound", action="store_true",
                help="restrict to OUTBOUND drones (RouteA/RouteBhalf: first half "
                     "of each corridor's contiguous id block); suffix _outbound")
ap.add_argument("--cached", action="store_true")
args = ap.parse_args()

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR   = os.path.join(SCRIPT_DIR, args.dir)
OUT_DIR    = os.path.join(DATA_DIR, "plots")
os.makedirs(OUT_DIR, exist_ok=True)
LTAG       = "_noramps" if args.exclude_ramps else ("_noland" if args.exclude_landing else "")
LTAG      += "_outbound" if args.outbound else ""
STATS_CSV  = os.path.join(OUT_DIR, f"tls_companion_stats{LTAG}.csv")

K_VALUES = [1, 2, 4, 8, 16, 32]
C_VALUES = [5, 10, 20, 40, 60, 80, 100, 120]
POL       = [p for p in args.pol.split(",") if p]
POL_LABEL = {"round-robin-sw": "RR", "pf-classic-sw": "PF", "mr-mw-ch-sw": "Risk-Aware",
             "max-risk-pf-sw": "Risk-PF", "mw-sw-otimo-ch": "MW-opt (sw)",
             "mw-otimo-ch": "MW-opt", "mw-sw-v8": "MW-V8", "mw-sw-v9": "MW-V9", "mw-v9-nsw": "MW-V9-nsw"}

# ---- IEEE single-column style (IEEEtran-compatible, grayscale-safe via cell numbers + contour) ----
plt.rcParams.update({
    "font.family": "serif", "font.serif": ["Times New Roman", "Nimbus Roman", "DejaVu Serif"],
    "mathtext.fontset": "stix",
    "font.size": 8, "axes.titlesize": 9, "axes.labelsize": 8,
    "xtick.labelsize": 7, "ytick.labelsize": 7, "figure.dpi": 150,
})

def _pick(pat):
    fs = sorted(glob.glob(pat))            # timestamp in name -> lexical sort = chronological
    if not fs:
        return None
    return fs[0] if args.run == "first" else fs[-1]

def _cell_files(K, C, pol):
    base = os.path.join(DATA_DIR, f"K{K}", f"C{C:02d}", args.scenario)
    return (_pick(os.path.join(base, f"{pol}_*_slot_series.csv")),
            _pick(os.path.join(base, f"{pol}_*_drone_summary.csv")))

def per_uav_steady(K, C, pol):
    """Return per-UAV DataFrame of (R̄_u, frac_above_TLS), or None.

    window='flight'  -> R̄_u over the whole flight (matches the original figure);
                        R̄_u taken from drone_summary.mean_r_tot when present.
    window='steady'  -> restrict each UAV's samples to the >=C full-load span.
    """
    ss, ds = _cell_files(K, C, pol)
    if ss is None or ds is None:
        return None
    df = pd.read_csv(ss, usecols=["slot", "drone_id", "r_tot"],
                     dtype={"slot": np.int32, "drone_id": np.int32, "r_tot": np.float32})
    if args.exclude_landing or args.exclude_ramps:
        # 500 m ramps masked (low-altitude outage spikes); corridor A (8 km)
        # vs B (4.264 km) classified by flight duration
        meta = pd.read_csv(ds, usecols=["drone_id", "entry_slot", "exit_slot"])
        meta["dur"] = meta.exit_slot - meta.entry_slot
        frac = np.where(meta.dur > meta.dur.min() * 1.5, 500.0/8000.0, 500.0/4264.4)
        meta["hi"] = meta.entry_slot + ((1.0 - frac) * meta.dur).astype(int)
        meta["lo"] = meta.entry_slot + (frac * meta.dur).astype(int)
        df = df.merge(meta[["drone_id", "lo", "hi"]], on="drone_id")
        df = df[df.slot < df.hi]
        if args.exclude_ramps:
            df = df[df.slot > df.lo]
        df = df.drop(columns=["lo", "hi"])
    if args.window == "steady":
        nact = df.groupby("slot").drone_id.size()
        thr = C if (nact >= C).any() else max(1, int(0.9 * nact.max()))
        hit = nact.index[nact >= thr]
        if len(hit) == 0:
            return None
        lo, hi = hit.min(), hit.max()
        df = df[(df.slot >= lo) & (df.slot <= hi)]
    g = df.groupby("drone_id")
    rbar = g.r_tot.mean()                                   # R̄_u : per-UAV time-average
    frac = g.r_tot.apply(lambda s: float((s > args.tls).mean()))  # P_t(R_u>TLS) per UAV
    n    = g.r_tot.size()
    if args.window == "flight" and not (args.exclude_landing or args.exclude_ramps):
        # prefer the simulator's own per-UAV mean over the whole flight, if available
        try:
            ms = pd.read_csv(ds, usecols=["drone_id", "mean_r_tot"]).set_index("drone_id")["mean_r_tot"]
            rbar = ms.reindex(rbar.index).fillna(rbar)
        except Exception:
            pass
    keep = n >= args.min_slots
    out = pd.DataFrame({"rbar": rbar[keep], "frac": frac[keep]})
    if args.outbound:
        meta = pd.read_csv(ds, usecols=["drone_id", "entry_slot", "exit_slot"])
        meta["dur"] = meta.exit_slot - meta.entry_slot
        long_ids  = sorted(meta[meta.dur > meta.dur.min() * 1.5].drone_id)
        short_ids = sorted(meta[~(meta.dur > meta.dur.min() * 1.5)].drone_id)
        ob = set(long_ids[:len(long_ids)//2]) | set(short_ids[:len(short_ids)//2])
        out = out[out.index.isin(ob)]
    return out

# ------------------------------------------------------------------ build/load stats
if args.cached and os.path.exists(STATS_CSV):
    stats = pd.read_csv(STATS_CSV)
    print("loaded", STATS_CSV)
else:
    rows = []
    for K in K_VALUES:
        for C in C_VALUES:
            for pol in POL:
                pu = per_uav_steady(K, C, pol)
                if pu is None or len(pu) == 0:
                    print(f"  MISSING K{K} C{C} {pol}")
                    continue
                rows.append(dict(K=K, C=C, policy=pol,
                                 max_rbar=float(pu.rbar.max()),
                                 argmax_uav=int(pu.rbar.idxmax()),
                                 mean_rbar=float(pu.rbar.mean()),
                                 max_frac=float(pu.frac.max()),   # worst-UAV time-fraction > TLS
                                 n_uav=int(len(pu))))
        print(f"  done K{K}")
    stats = pd.DataFrame(rows)
    stats.to_csv(STATS_CSV, index=False)
    print("saved", STATS_CSV, f"({len(stats)} cells)")

def matrix(col, pol):
    M = np.full((len(K_VALUES), len(C_VALUES)), np.nan)
    for i, K in enumerate(K_VALUES):
        for j, C in enumerate(C_VALUES):
            r = stats[(stats.K == K) & (stats.C == C) & (stats.policy == pol)]
            if len(r):
                M[i, j] = r[col].values[0]
    return M

# ================================================================ FIG 1: max_u R̄_u
allv = stats.max_rbar.values
lo, hi = np.log10(np.nanmin(allv)), np.log10(np.nanmax(allv))
log_scale = (hi - lo) > 1.0
# vmin extended below the data so the darkest data-blue maps to a mid blue
# (lighter blue -> the black cell numbers stand out); center stays at log10(1)=0.
norm = TwoSlopeNorm(vmin=min(2.5 * lo, -0.5), vcenter=0.0, vmax=max(hi, 1e-3))
cmap = plt.get_cmap("RdBu_r")

fig, axes = plt.subplots(1, len(POL), figsize=(7.16, 1.13), constrained_layout=True, squeeze=False)
axes = axes.flatten()
for pi, pol in enumerate(POL):
    ax = axes[pi]
    M = matrix("max_rbar", pol)
    Z = np.log10(M)
    im = ax.imshow(Z, aspect="auto", origin="lower", cmap=cmap, norm=norm)
    ax.set_xticks(range(len(C_VALUES))); ax.set_xticklabels(C_VALUES)
    ax.set_yticks(range(len(K_VALUES))); ax.set_yticklabels(K_VALUES)
    ax.set_title(POL_LABEL[pol], fontweight="bold")
    ax.set_xlabel(r"Simultaneous UAVs")
    if pi == 0:
        ax.set_ylabel(r"$K$")
    for i in range(len(K_VALUES)):
        for j in range(len(C_VALUES)):
            v = M[i, j]
            if np.isfinite(v):
                # 4 sig figs above 1 so the r_vid saturation reads 10.16 (not 10.2) and
                # distinct values don't collapse; keep 2 decimals below 1 for readability.
                txt = f"{v:.2f}" if v < 1 else f"{v:.4g}"
                ax.text(j, i, txt, ha="center", va="center", fontsize=5.5,
                        color="black", fontweight="bold")
# colorbar removed (values are printed in every cell)
# value label on the right, where the colorbar used to be (vertical)
fig.text(1.005, 0.5, r"$\max_u \bar R_u$", rotation=90, va="center", ha="center", fontsize=10)
WTAG = ("" if args.window == "flight" else f"_{args.window}") + LTAG
base = os.path.join(OUT_DIR, f"tls_companion_maxRbar{WTAG}")
for ext in ("pdf", "png"):
    fig.savefig(f"{base}.{ext}", bbox_inches="tight")
print("saved", base)
plt.close(fig)

# ================================================================ FIG 2: corrected exceedance
fig, axes = plt.subplots(1, len(POL), figsize=(7.16, 2.7), constrained_layout=True, squeeze=False)
axes = axes.flatten()
for pi, pol in enumerate(POL):
    ax = axes[pi]
    M = 100.0 * matrix("max_frac", pol)
    im = ax.imshow(M, aspect="auto", origin="lower", cmap="magma", vmin=0, vmax=100)
    ax.set_xticks(range(len(C_VALUES))); ax.set_xticklabels(C_VALUES)
    ax.set_yticks(range(len(K_VALUES))); ax.set_yticklabels(K_VALUES)
    ax.set_title(POL_LABEL[pol], fontweight="bold")
    ax.set_xlabel(r"Simultaneous UAVs $C$")
    if pi == 0:
        ax.set_ylabel(r"$K$")
    for i in range(len(K_VALUES)):
        for j in range(len(C_VALUES)):
            v = M[i, j]
            if np.isfinite(v):
                ax.text(j, i, f"{v:.0f}", ha="center", va="center", fontsize=5.5,
                        color="white" if v < 60 else "black", fontweight="bold")
cbar = fig.colorbar(im, ax=axes.tolist(), shrink=0.9, pad=0.01)
cbar.set_label(r"$\max_u\, P_t(R_u > 1)$  [%]")
base = os.path.join(OUT_DIR, f"tls_exceed_maxPu{WTAG}")
for ext in ("pdf", "png"):
    fig.savefig(f"{base}.{ext}", bbox_inches="tight")
print("saved", base)
plt.close(fig)

# ================================================================ console: acceptance frontier
print("\n=== Acceptance frontier: largest C with max_u R̄_u <= 1  (per K) ===")
hdr = "  K  " + "".join(f"{POL_LABEL[p]:>12s}" for p in POL)
print(hdr)
for K in K_VALUES:
    cells = []
    for pol in POL:
        r = stats[(stats.K == K) & (stats.policy == pol)].sort_values("C")
        acc = r[r.max_rbar <= args.tls]
        cells.append(f"{int(acc.C.max()):>12d}" if len(acc) else f"{'none':>12s}")
    print(f"  {K:<3d}" + "".join(cells))
print("\n(‘none’ = TLS violated at every sampled C ≥ 5)")
