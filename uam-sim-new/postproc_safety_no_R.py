#!/usr/bin/env python3
"""postproc_safety_no_R.py -- pure post-processing safety evaluation that never
touches the composite risk index R. Everything is derived from raw per-slot
ages (h1 = C2 status, h2 = video) in the exported slot_series.

Pipeline
  1. Threshold exceedance per flow f in {c2, video}: contiguous EVENTS with
     age_f > tau_f (tau from config, not hardcoded). Per (policy, K, N, seed,
     UAV): lambda_f [events/FH], event-duration mean and p99 [s].
  2. Annex-F mapping (SORA v2.5, Eq. 3), C2/containment flow only:
       E_fat(s) = lambda_c2(s) * P_cont_fail * A_c * rho_op(s) * F_c
     with conservative P_cont_fail = 1 (UPPER BOUND: every containment-blind
     event treated as an uncontrolled ground impact). Binned by arc-length s.
  3. Statistics per seed first, then averaged across seeds with 95% t-CI.
     No pooling across seeds; across UAVs only order statistics (max / p_q).
     Bins with insufficient exposure are flagged and hollowed in the figure.
  4. Figures (grayscale-safe, IEEE single column):
       figA: lambda_c2 vs N (log-y) + SAIL budgets alpha*1e-5, alpha in cfg
       figB: E_fat(s) vs arc-length + TLOS 1e-6 /FH

Config: DEFAULTS below; override any key with --config file.json and/or the
dedicated CLI flags. tau_c2_slots=None derives the containment threshold from
geometry: (W_c/2 - r_min) / (v / update_rate).

Usage:
  python3 postproc_safety_no_R.py --dir csv_export_RC14 --K 8 --C-focal 60
Outputs in <dir>/plots/postproc_safety/.
"""
import os, glob, json, argparse
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

# ── Config struct (all thresholds/constants live here, not in the code) ──
DEFAULTS = {
    # flow thresholds
    "tau_c2_slots": None,        # None -> derive from geometry (containment)
    "tau_video_s": 15.0,         # operational DAA staleness bound [s]
    # geometry / kinematics (RC12/RC14 corridor-B design values)
    "wc_m": 100.0,               # corridor width W_c
    "r_min_m": 10.0,
    "update_rate_hz": 7.0,       # overridden by manifest when available
    "route_len_m": float(np.hypot(350.0, 4250.0)),
    # Annex-F parameters
    "A_c_m2": None,              # None -> Case 3 (w_ua < 1 m), Annex F 1.8.3
    "w_ua_m": 0.5,
    "F_c": 1.0,                  # exposure factor (iGRC assumption)
    "P_cont_fail": 1.0,          # conservative upper bound, declared
    "rho_peak_inhab_km2": 5000.0,
    "rho_bg_frac": 0.01,         # background density as fraction of peak
    "crowd_zone_m": [0.0, 2843.0],
    "TLOS_per_FH": 1e-6,
    "sail_budget_per_FH": 1e-5,  # lambda_GI at the scenario's min SAIL (V)
    "alphas": [0.05, 0.1, 0.3],  # scheduler share of the SAIL budget
    # statistics
    "q": 0.99,                   # order statistic across UAVs (within seed)
    "ci_level": 0.95,
    "bin_m": 100.0,
    "min_bin_hours": 0.05,       # per-seed exposure floor per bin [FH]
    "min_bin_uavs": 10,
}
POLICIES = ["round-robin-sw", "pf-classic-sw", "mw-sw-v8", "mw-sw-v9"]
LABEL  = {"round-robin-sw": "RR", "pf-classic-sw": "PF",
          "mw-sw-v8": "MW-V8", "mw-sw-v9": "MW-V9"}
# grayscale-safe: distinct line styles + markers carry the encoding
STYLE  = {"round-robin-sw": ("0.00", "-",  "o"),
          "pf-classic-sw":  ("0.35", "--", "s"),
          "mw-sw-v8":       ("0.55", ":",  "x"),
          "mw-sw-v9":       ("0.00", "-.", "^")}

ap = argparse.ArgumentParser()
ap.add_argument("--dir", default="csv_export_RC14")
ap.add_argument("--scenario", default="4r_b7")
ap.add_argument("--K", type=int, default=8)
ap.add_argument("--C-focal", type=int, default=60, help="cell for figB/table")
ap.add_argument("--N-list", default="", help="comma list; default = all found")
ap.add_argument("--config", default="", help="JSON overriding DEFAULTS keys")
args = ap.parse_args()

CFG = dict(DEFAULTS)
if args.config:
    with open(args.config) as f:
        CFG.update(json.load(f))

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(SCRIPT_DIR, args.dir)
OUT = os.path.join(DATA, "plots", "postproc_safety")
os.makedirs(OUT, exist_ok=True)
RL = CFG["route_len_m"]

if CFG["A_c_m2"] is None:
    r_D = 0.3 + CFG["w_ua_m"] / 2.0
    CFG["A_c_m2"] = 2.0 * r_D * (1.8 / np.tan(np.deg2rad(35.0))) + np.pi * r_D**2
RHO_PEAK = CFG["rho_peak_inhab_km2"] * 1e-6          # [people/m^2]

def rho_op(s):
    """Population density at arc-length s [people/m^2] (crowd plateau model:
    RC14's 24-hotspot field saturates at the approved peak across the zone)."""
    lo, hi = CFG["crowd_zone_m"]
    return np.where((s >= lo) & (s <= hi), RHO_PEAK, CFG["rho_bg_frac"] * RHO_PEAK)

def steady_window(ds, C):
    t_max = int(ds.exit_slot.max()) + 1
    delta = np.zeros(t_max + 1)
    np.add.at(delta, ds.entry_slot.astype(int).values, 1)
    np.add.at(delta, ds.exit_slot.astype(int).values, -1)
    airborne = np.cumsum(delta)[:-1]
    full = np.flatnonzero(airborne >= min(C, airborne.max()))
    return int(full[0]), int(full[-1])

def events_from_series(age, tau):
    """Contiguous runs with age > tau -> (start indices, durations in samples)."""
    above = (age > tau).astype(int)
    d = np.diff(above)
    starts = np.flatnonzero(d == 1) + 1
    ends   = np.flatnonzero(d == -1) + 1
    if above[0]:
        starts = np.r_[0, starts]
    if above[-1]:
        ends = np.r_[ends, len(above)]
    return starts, (ends - starts)

def load_cell(pol, K, C):
    """Yield one dict per seed replicate (run file) for cell (pol, K, C)."""
    base = os.path.join(DATA, f"K{K}", f"C{C:02d}", args.scenario)
    runs = sorted(glob.glob(os.path.join(base, f"{pol}_*_slot_series.csv")))
    for run in runs:
        run_id = os.path.basename(run).replace("_slot_series.csv", "")
        dsf = os.path.join(base, f"{run_id}_drone_summary.csv")
        man = os.path.join(base, f"{run_id}_manifest.csv")
        upd = CFG["update_rate_hz"]
        if os.path.exists(man):
            upd = float(pd.read_csv(man).update_rate.iloc[0])
        ds = pd.read_csv(dsf)
        t0, t1 = steady_window(ds, C)
        ds["dur"] = ds.exit_slot - ds.entry_slot
        b = ds[ds.dur < 0.7 * ds.dur.max()].sort_values("drone_id")
        ent = b.entry_slot.values
        resets = np.flatnonzero(np.diff(ent) < 0)
        cut = (resets[0] + 1) if len(resets) else len(b) // 2
        ids_to = set(b.drone_id.values[:cut])
        ss = pd.read_csv(run, usecols=["slot", "drone_id", "h1_slots", "h2_slots"])
        ss = ss[ss.drone_id.isin(set(b.drone_id)) & ss.slot.between(t0, t1)]
        ss = ss.merge(ds[["drone_id", "entry_slot", "dur"]], on="drone_id")
        v = float(np.median(RL / (b.dur.values / upd)))          # [m/s]
        trav = (ss.slot - ss.entry_slot) / ss.dur * RL
        ss["dist"] = np.where(ss.drone_id.isin(ids_to), RL - trav, trav)
        tau_c2 = CFG["tau_c2_slots"]
        if tau_c2 is None:
            tau_c2 = (CFG["wc_m"] / 2.0 - CFG["r_min_m"]) / (v / upd)
        yield dict(seed=run_id, ss=ss, upd=upd, tau_c2=tau_c2,
                   tau_vid=CFG["tau_video_s"] * upd)

def per_uav_records(cell, pol, K, C):
    """Step 1: per-UAV exceedance records + per-bin per-UAV c2 event rates."""
    recs, bins = [], []
    edges = np.arange(0.0, RL + CFG["bin_m"], CFG["bin_m"])
    for _, g in cell["ss"].sort_values("slot").groupby("drone_id"):
        hours = len(g) / cell["upd"] / 3600.0        # exposure inside window
        if hours <= 0:
            continue
        for flow, col, tau in (("c2", "h1_slots", cell["tau_c2"]),
                               ("video", "h2_slots", cell["tau_vid"])):
            starts, durs = events_from_series(g[col].values, tau)
            durs_s = durs / cell["upd"]
            recs.append(dict(policy=pol, K=K, N=C, seed=cell["seed"],
                             uav=int(g.drone_id.iloc[0]), flow=flow,
                             flight_hours=hours, n_events=len(starts),
                             lambda_per_FH=len(starts) / hours,
                             dur_mean_s=float(np.mean(durs_s)) if len(durs) else np.nan,
                             dur_p99_s=float(np.quantile(durs_s, 0.99)) if len(durs) else np.nan))
            if flow == "c2":
                bi = np.digitize(g.dist.values, edges) - 1
                h_bin = np.bincount(bi, minlength=len(edges) - 1) / cell["upd"] / 3600.0
                e_bin = np.bincount(bi[starts], minlength=len(edges) - 1) if len(starts) \
                        else np.zeros(len(edges) - 1)
                rate = np.divide(e_bin, h_bin, out=np.full(len(h_bin), np.nan),
                                 where=h_bin > 0)
                bins.append((rate, h_bin))
    return recs, bins

def tci(vals, level):
    """Mean and half-width of the t confidence interval across seeds."""
    vals = np.asarray(vals, float)
    n = len(vals)
    if n < 2:
        return float(np.mean(vals)), np.nan
    from scipy import stats
    hw = stats.t.ppf(0.5 + level / 2.0, n - 1) * np.std(vals, ddof=1) / np.sqrt(n)
    return float(np.mean(vals)), float(hw)

# ── Sweep the K row ───────────────────────────────────────────────────
if args.N_list:
    N_VALUES = [int(x) for x in args.N_list.split(",")]
else:
    N_VALUES = sorted(int(d[1:]) for d in os.listdir(os.path.join(DATA, f"K{args.K}"))
                      if d.startswith("C"))
edges = np.arange(0.0, RL + CFG["bin_m"], CFG["bin_m"])
centers = 0.5 * (edges[:-1] + edges[1:])

all_recs, figA, figB = [], {}, {}
n_seeds_seen = set()
for pol in POLICIES:
    lamN = {}
    for C in N_VALUES:
        seed_worst = []
        for cell in load_cell(pol, args.K, C):
            n_seeds_seen.add(cell["seed"].split("_")[-2] + "_" + cell["seed"].split("_")[-1])
            recs, bins = per_uav_records(cell, pol, args.K, C)
            all_recs += recs
            lam_c2 = [r["lambda_per_FH"] for r in recs
                      if r["flow"] == "c2" and r["seed"] == cell["seed"]]
            seed_worst.append(max(lam_c2))            # worst over UAVs, per seed
            if C == args.C_focal:
                rates = np.vstack([r for r, _ in bins])
                hrs   = np.vstack([h for _, h in bins])
                lam_bin = np.nanquantile(rates, CFG["q"], axis=0)  # across UAVs
                efat = lam_bin * CFG["P_cont_fail"] * CFG["A_c_m2"] \
                       * rho_op(centers) * CFG["F_c"]
                ok = (hrs.sum(axis=0) >= CFG["min_bin_hours"]) & \
                     ((~np.isnan(rates)).sum(axis=0) >= CFG["min_bin_uavs"])
                figB.setdefault(pol, []).append((efat, ok))
        if seed_worst:
            lamN[C] = tci(seed_worst, CFG["ci_level"])
    figA[pol] = lamN

df = pd.DataFrame(all_recs)
df.to_csv(os.path.join(OUT, "exceedance_per_uav.csv"), index=False)
n_seeds = df.groupby(["policy", "N"]).seed.nunique().max()
print(f"seeds per cell: {n_seeds}" +
      ("  [WARNING: single seed -- 95% CI unavailable, shown as n/a]"
       if n_seeds < 2 else ""))

plt.rcParams.update({"font.family": "serif",
                     "font.serif": ["Times New Roman", "Nimbus Roman", "DejaVu Serif"],
                     "mathtext.fontset": "stix", "font.size": 8, "pdf.fonttype": 42})

# ── Fig A: lambda_c2 vs N ─────────────────────────────────────────────
FLOOR_A = 1e-3      # display floor: zero observed events (declared, not data)
fig, ax = plt.subplots(figsize=(3.5, 2.7), constrained_layout=True)
for pol in POLICIES:
    Ns = sorted(figA[pol])
    mu = np.array([figA[pol][n][0] for n in Ns])
    hw = np.array([figA[pol][n][1] for n in Ns])
    col, ls, mk = STYLE[pol]
    ax.errorbar(Ns, np.maximum(mu, FLOOR_A), yerr=None if np.isnan(hw).all() else hw,
                color=col, ls=ls, marker=mk, ms=3.5, lw=1.1, mfc="white",
                label=LABEL[pol], capsize=2)
ax.annotate("0 events (display floor)", (N_VALUES[0], FLOOR_A), fontsize=6,
            color="0.4", va="bottom", ha="left")
for a in CFG["alphas"]:
    y = a * CFG["sail_budget_per_FH"]
    ax.axhline(y, color="0.6", lw=0.7, ls=(0, (1, 1)))
    ax.annotate(rf"$\alpha$={a}", (N_VALUES[-1], y), fontsize=6, color="0.4",
                va="bottom", ha="right")
ax.set_yscale("log")
ax.set_xlabel("Simultaneous UAVs $N$")
ax.set_ylabel(r"$\lambda_{c2}$ worst UAV [events/FH]")
ax.grid(True, which="both", ls=":", lw=0.3, alpha=0.5)
ax.legend(frameon=False, fontsize=7, ncol=2)
for ext in ("pdf", "png"):
    fig.savefig(os.path.join(OUT, f"figA_lambda_c2_vs_N.{ext}"), dpi=300)
print("saved figA_lambda_c2_vs_N.{pdf,png}")

# ── Fig B: E_fat(s) at the focal cell ─────────────────────────────────
FLOOR_B = 1e-7      # display floor: zero events in bin (declared, not data)
fig, ax = plt.subplots(figsize=(3.5, 2.7), constrained_layout=True)
peak_efat = {}
for pol in POLICIES:
    seed_curves = np.vstack([e for e, _ in figB[pol]])
    ok = np.logical_and.reduce([o for _, o in figB[pol]])
    mu = np.nanmean(seed_curves, axis=0)
    peak_efat[pol] = float(np.nanmax(mu))
    disp = np.where(mu > 0, mu, FLOOR_B)
    col, ls, mk = STYLE[pol]
    ax.plot(centers[ok], disp[ok], color=col, ls=ls, lw=1.1, label=LABEL[pol])
    if (~ok).any():
        ax.plot(centers[~ok], disp[~ok], color=col, ls="none", marker=mk,
                ms=2.5, mfc="white", mew=0.5)
        print(f"  {LABEL[pol]}: {(~ok).sum()} low-sample bins (hollow markers)")
ax.annotate("0 events in bin (display floor)", (RL * 0.02, FLOOR_B), fontsize=6,
            color="0.4", va="bottom", ha="left")
ax.axhline(CFG["TLOS_per_FH"], color="0.1", lw=0.9, ls=(0, (4, 2)))
ax.annotate("TLOS $10^{-6}$", (RL * 0.98, CFG["TLOS_per_FH"]), fontsize=6.5,
            color="0.1", va="bottom", ha="right")
lo, hi = CFG["crowd_zone_m"]
ax.axvspan(lo, hi, color="0.8", alpha=0.35, lw=0)
ax.set_yscale("log")
ax.set_xlabel("Arc-length $s$ from Event venue [m]")
ax.set_ylabel(r"$E_{fat}(s)$ upper bound [fat/FH]")
ax.set_title(rf"$P_{{cont.fail}}=1$ (upper bound), K={args.K}, N={args.C_focal}",
             fontsize=7.5)
ax.grid(True, which="both", ls=":", lw=0.3, alpha=0.5)
ax.legend(frameon=False, fontsize=7, ncol=2)
for ext in ("pdf", "png"):
    fig.savefig(os.path.join(OUT, f"figB_efat_arclength.{ext}"), dpi=300)
print("saved figB_efat_arclength.{pdf,png}")

# ── Summary table (focal cell) ────────────────────────────────────────
foc = df[(df.N == args.C_focal)]
print(f"\nSummary (K={args.K}, N={args.C_focal}; worst over UAVs, per seed then "
      f"averaged; {n_seeds} seed(s)):")
hdr = f"{'policy':8} {'lam_c2':>9} {'lam_vid':>9} {'peakEfat':>10}"
for a in CFG["alphas"]:
    hdr += f" {'a=' + str(a):>8}"
hdr += f" {'TLOS':>6}"
print(hdr)
rows_out = []
for pol in POLICIES:
    sub = foc[foc.policy == pol]
    wl_c2 = sub[sub.flow == "c2"].groupby("seed").lambda_per_FH.max().mean()
    wl_vd = sub[sub.flow == "video"].groupby("seed").lambda_per_FH.max().mean()
    line = f"{LABEL[pol]:8} {wl_c2:9.2f} {wl_vd:9.2f} {peak_efat[pol]:10.2e}"
    flags = {}
    for a in CFG["alphas"]:
        ok = wl_c2 <= a * CFG["sail_budget_per_FH"]
        flags[f"alpha_{a}"] = ok
        line += f" {'PASS' if ok else 'FAIL':>8}"
    tl = peak_efat[pol] <= CFG["TLOS_per_FH"]
    flags["TLOS"] = tl
    line += f" {'PASS' if tl else 'FAIL':>6}"
    print(line)
    rows_out.append(dict(policy=pol, lambda_c2_worst=wl_c2, lambda_video_worst=wl_vd,
                         peak_Efat=peak_efat[pol], **flags))
pd.DataFrame(rows_out).to_csv(os.path.join(OUT, "summary_table.csv"), index=False)
print("\nsaved", os.path.join(OUT, "summary_table.csv"))
print(f"config: tau_video={CFG['tau_video_s']}s, A_c={CFG['A_c_m2']:.2f} m^2, "
      f"F_c={CFG['F_c']}, P_cont_fail={CFG['P_cont_fail']} (upper bound), "
      f"bin={CFG['bin_m']:.0f} m")
