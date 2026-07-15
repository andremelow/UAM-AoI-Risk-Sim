#!/usr/bin/env python3
"""plot_fig_sora_lambda_fatality.py -- SORA Annex F (v2.5) native acceptance
metric along corridor B, built ONLY from the ground-risk component (r_map),
i.e. no weighted-sum scalarization:

  lambda_fatality(x) = lambda_GI * Dpop(x) * F_exp * A_C * P(fat|collision)
                                                        [Annex F, Eq. (3)]
with the iGRC conservative assumptions F_exp = 1, P(fat|collision) = 1
(Annex F, Eq. (4)) and acceptance  lambda_fatality <= TLOS = 1e-6 fat/FH
met for ALL overflown segments (Annex F, Sec. 3.3.2) -- not on flight average.

Where AoI enters: Dpop(x) is the mean physical density inside the navigation
uncertainty disk B_u(r_u), r_u = r_min + (v/updateRate)*h1. Stale C2 inflates
the disk; over a heterogeneous field this drags crowd density into Dpop.

De-normalization chain (RC14 exports use use_normalization=true):
  r_map(exported) = (integral of rho_tilde over B_u) / (pi r*^2),
  rho_tilde in [0,1] with peak == rho_op -> Dpop = rho_op * r_map*(r*/r_u)^2,
  r* = r_min + v/c2_hz  (paper design point).

Two variants plotted (per-UAV bin mean, then p99 across UAVs -- same
aggregation as the TLS target metric):
  (a) SORA-calibrated: A_C from Annex F Case 3 (UA width < 1 m):
      A_C = 2 r_D d_glide + pi r_D^2,  r_D = r_person + w/2,
      d_glide = h_person / tan(theta)             [Annex F, Sec. 1.8.3]
  (b) Conservative containment: whole uncertainty disk treated as critical
      (A_C -> pi r_u^2), i.e. expected people-in-disk. Upper bound.

Also prints, per policy: worst-bin p99 casualty expectation E_C = Dpop*A_C,
the max allowable lambda_GI = TLOS/E_C and the implied minimum SAIL
(Annex F Table 1: SAIL I..VI <-> lambda_GI 1e-1..1e-6 /FH).

Usage:
  python3 plot_fig_sora_lambda_fatality.py --dir csv_export_RC14 --K 8 --C 60
"""
import os, glob, argparse
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

ap = argparse.ArgumentParser()
ap.add_argument("--dir", default="csv_export_RC14")
ap.add_argument("--scenario", default="4r_b7")
ap.add_argument("--K", type=int, default=8)
ap.add_argument("--C", type=int, default=60)
ap.add_argument("--q", type=float, default=0.99)
ap.add_argument("--bin", type=float, default=50.0)
ap.add_argument("--routes", choices=["both", "to-event", "from-event"], default="both")
ap.add_argument("--pol", default="round-robin-sw,pf-classic-sw,mw-sw-v8,mw-sw-v9")
ap.add_argument("--sail", type=int, default=4, choices=[1, 2, 3, 4, 5, 6],
                help="assumed achieved SAIL -> lambda_GI = 10^-sail /FH")
ap.add_argument("--w-ua", type=float, default=0.5, help="UA width [m] (<1 m: Case 3)")
ap.add_argument("--rho-op", type=float, default=5000.0, help="peak density [inhab/km^2]")
ap.add_argument("--r-min", type=float, default=10.0)
ap.add_argument("--c2-hz", type=float, default=2.0)
ap.add_argument("--update-rate", type=float, default=7.0)
ap.add_argument("--hs-zone", default="0:2843")
ap.add_argument("--wc", type=float, default=100.0,
                help="corridor width W_c [m]; containment buffer = W_c/2")
args = ap.parse_args()

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.join(SCRIPT_DIR, args.dir, f"K{args.K}", f"C{args.C:02d}", args.scenario)
ROUTE_LEN = float(np.hypot(-700 - (-350), 14000 - 9750))
HS_LO, HS_END = (float(v) for v in args.hs_zone.split(":"))

POL    = [p for p in args.pol.split(",") if p]
LABEL  = {"round-robin-sw": "RR", "pf-classic-sw": "PF",
          "mw-sw-v8": "MW-V8", "mw-sw-v9": "MW-V9",
          "mw-z": "MW-Z", "mw-z-v9": "MW-Z-V9"}
COLORS = {"round-robin-sw": "#F9A825", "pf-classic-sw": "#E53935",
          "mw-sw-v8": "#00897B", "mw-sw-v9": "#7CB342",
          "mw-z": "#1E88E5", "mw-z-v9": "#6A1B9A"}

# ── SORA constants (Annex F) ──────────────────────────────────────────
TLOS     = 1e-6                            # fatalities per flight hour
LAM_GI   = 10.0 ** (-args.sail)            # loss-of-control rate for the SAIL
RHO_OP   = args.rho_op * 1e-6              # [people/m^2]
H_PERSON, R_PERSON, THETA = 1.8, 0.3, np.deg2rad(35.0)   # Annex F Table 3
d_glide  = H_PERSON / np.tan(THETA)
r_D      = R_PERSON + args.w_ua / 2.0
A_C      = 2.0 * r_D * d_glide + np.pi * r_D**2          # Case 3 (w < 1 m)
print(f"A_C = {A_C:.2f} m^2 (Case 3, w={args.w_ua} m) | lambda_GI = {LAM_GI:.0e}/FH "
      f"(SAIL {args.sail}) | TLOS = {TLOS:.0e} fat/FH | rho_op = {RHO_OP:.1e} ppl/m^2")

def steady_window(ds):
    t_max = int(ds.exit_slot.max()) + 1
    delta = np.zeros(t_max + 1)
    np.add.at(delta, ds.entry_slot.astype(int).values, 1)
    np.add.at(delta, ds.exit_slot.astype(int).values, -1)
    airborne = np.cumsum(delta)[:-1]
    full = np.flatnonzero(airborne >= args.C)
    if len(full) == 0:
        raise SystemExit(f"steady window empty (max airborne {airborne.max():.0f})")
    return int(full[0]), int(full[-1])

def load(pol):
    ds = pd.read_csv(sorted(glob.glob(os.path.join(BASE, f"{pol}_*_drone_summary.csv")))[-1])
    t0, t1 = steady_window(ds)
    ds["dur"] = ds.exit_slot - ds.entry_slot
    b = ds[ds.dur < 0.7 * ds.dur.max()].sort_values("drone_id")
    ent = b.entry_slot.values
    resets = np.flatnonzero(np.diff(ent) < 0)
    cut = (resets[0] + 1) if len(resets) else len(b) // 2
    ids_to, ids_from = b.drone_id.values[:cut], b.drone_id.values[cut:]
    keep = {"to-event": set(ids_to), "from-event": set(ids_from),
            "both": set(ids_to) | set(ids_from)}[args.routes]

    ss = pd.read_csv(sorted(glob.glob(os.path.join(BASE, f"{pol}_*_slot_series.csv")))[-1],
                     usecols=["slot", "drone_id", "h1_slots", "r_map"])
    ss = ss[ss.drone_id.isin(keep) & ss.slot.between(t0, t1)]
    ss = ss.merge(ds[["drone_id", "entry_slot", "dur"]], on="drone_id")

    # empirical cruise speed [m/s] from paired flight durations (seed-fixed)
    v = float(np.median(ROUTE_LEN / (b.dur.values / args.update_rate)))
    r_star = args.r_min + v / args.c2_hz
    r_u    = args.r_min + (v / args.update_rate) * ss.h1_slots.values

    # mean rho_tilde inside the disk: must land in [~rho_0, 1] -- sanity check.
    # Values slightly above 1 come from the export's slot interpolation of
    # r_map vs the stepwise h1; clip at the physical ceiling (field peak = 1).
    rho_raw   = ss.r_map.values * (r_star / r_u) ** 2
    rho_tilde = np.minimum(rho_raw, 1.0)
    print(f"  {LABEL.get(pol, pol):7}: v = {v:.2f} m/s, r* = {r_star:.1f} m, "
          f"rho_tilde p50/p99/max = {np.percentile(rho_raw, 50):.3f}/"
          f"{np.percentile(rho_raw, 99):.3f}/{rho_raw.max():.3f}"
          f"{'  (clipped to 1)' if rho_raw.max() > 1 else ''}")

    ss = ss.assign(
        EC_sora=RHO_OP * rho_tilde * A_C,                    # Dpop * A_C  [people]
        EC_cont=RHO_OP * ss.r_map.values * np.pi * r_star**2)  # people in disk
    trav = (ss.slot - ss.entry_slot) / ss.dur * ROUTE_LEN
    to_event = ss.drone_id.isin(set(ids_to))
    ss["dist"] = np.where(to_event, ROUTE_LEN - trav, trav)

    # containment-blind events: upcrossings of r_u above the corridor buffer
    # W_c/2 (uncertainty no longer contained in the corridor). Rate per FH,
    # per UAV -- the channel through which the SCHEDULER feeds lambda_GI.
    h_crit = (args.wc / 2.0 - args.r_min) / (v / args.update_rate)   # [slots]
    rates, rates_crowd = [], []
    for _, g in ss.sort_values("slot").groupby("drone_id"):
        above  = (g.h1_slots.values > h_crit).astype(int)
        starts = np.flatnonzero(np.diff(above) == 1) + 1
        if above[0]:
            starts = np.r_[0, starts]
        hours = (g.dur.iloc[0] / args.update_rate) / 3600.0
        rates.append(len(starts) / hours)
        # events that begin while over the hotspot (crowd) zone
        n_crowd = int(np.sum(g.dist.values[starts] <= HS_END)) if len(starts) else 0
        rates_crowd.append(n_crowd / hours)
    rates, rates_crowd = np.asarray(rates), np.asarray(rates_crowd)
    blind[pol] = (float(np.median(rates)), float(np.quantile(rates, args.q)),
                  float((rates > 0).mean()), h_crit,
                  float(np.median(rates_crowd)),
                  float(np.quantile(rates_crowd, args.q)),
                  float((rates_crowd > 0).mean()))
    return ss

edges = np.arange(0.0, ROUTE_LEN + args.bin, args.bin)
centers = 0.5 * (edges[:-1] + edges[1:])

prof, summary, blind = {}, [], {}
for p in POL:
    d = load(p)
    d["bin"] = pd.cut(d.dist, edges, labels=False)
    gu = d.groupby(["bin", "drone_id"], observed=True)[["EC_sora", "EC_cont"]].mean()
    gb = gu.groupby(level=0)
    q_sora = gb.EC_sora.quantile(args.q).reindex(range(len(centers)))
    q_cont = gb.EC_cont.quantile(args.q).reindex(range(len(centers)))
    prof[p] = (q_sora, q_cont)
    ec_max = float(q_sora.max())
    lam_max = TLOS / ec_max
    sail_req = int(min(6, max(1, np.ceil(-np.log10(lam_max)))))
    summary.append((LABEL.get(p, p), ec_max, lam_max, sail_req))

plt.rcParams.update({"font.family": "serif",
                     "font.serif": ["Times New Roman", "Nimbus Roman", "DejaVu Serif"],
                     "mathtext.fontset": "stix", "font.size": 9, "pdf.fonttype": 42})
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(8.4, 3.4), sharex=True,
                               constrained_layout=True)
QPCT = int(round(100 * args.q))

for ax, col, title in ((ax1, 0, rf"(a) SORA-calibrated ($A_C$ = {A_C:.1f} m$^2$, "
                                 "Annex F Case 3)"),
                       (ax2, 1, r"(b) conservative: containment disk as $A_C$")):
    for p in POL:
        ax.plot(centers, LAM_GI * prof[p][col].values, color=COLORS[p], lw=1.5,
                label=LABEL.get(p, p))
    ax.axhline(TLOS, ls=(0, (4, 3)), lw=1.0, color="0.2")
    ax.axvspan(HS_LO, HS_END, color="0.55", alpha=0.14, lw=0)
    ax.set_yscale("log")
    ax.set_xlim(0, ROUTE_LEN)
    ax.set_xlabel("Distance from Event venue [m]", fontsize=9)
    ax.set_title(title, fontsize=9)
    ax.grid(True, which="both", ls=":", lw=0.35, alpha=0.55)
ax1.text(ROUTE_LEN * 0.99, TLOS, " TLOS $10^{-6}$", fontsize=7, color="0.2",
         va="bottom", ha="right")
ax1.set_ylabel(rf"$p_{{{QPCT}}}$ over UAVs of "
               rf"$\lambda_{{fatality}}(x)$  [fat/FH]", fontsize=9)
fig.suptitle(rf"Annex F Eq. (3) along corridor B -- K = {args.K}, N = {args.C}, "
             rf"$\lambda_{{GI}} = 10^{{-{args.sail}}}$/FH (SAIL {args.sail})",
             fontsize=10, fontweight="bold")
handles = [Line2D([0], [0], color=COLORS[p], lw=1.8, label=LABEL.get(p, p)) for p in POL]
handles.append(Line2D([0], [0], ls=(0, (4, 3)), color="0.2", lw=1.1,
                      label="TLOS = $10^{-6}$ fat/FH"))
fig.legend(handles=handles, ncol=len(handles), loc="lower center",
           bbox_to_anchor=(0.5, -0.09), frameon=False, fontsize=8.5)

OUT = os.path.join(SCRIPT_DIR, args.dir, "plots")
os.makedirs(OUT, exist_ok=True)
sfx = "" if args.routes == "both" else "_" + args.routes.replace("-", "")
base = os.path.join(OUT, f"fig_sora_lambda_fatality_K{args.K}_C{args.C}{sfx}")
for ext in ("pdf", "png"):
    fig.savefig(f"{base}.{ext}", dpi=200, bbox_inches="tight")
    print("saved", f"{base}.{ext}")

print(f"\nWorst-bin p{QPCT} casualty expectation E_C = Dpop*A_C (SORA-calibrated):")
print(f"{'policy':9} {'E_C [ppl]':>11} {'max lam_GI':>12} {'min SAIL':>9}")
for lab, ec, lam, sail in summary:
    print(f"{lab:9} {ec:11.3e} {lam:12.3e} {'SAIL ' + str(sail):>9}")

h_crit = blind[POL[0]][3]
print(f"\nContainment-blind events (r_u > W_c/2 = {args.wc/2:.0f} m, i.e. "
      f"h1 > {h_crit:.0f} slots), rate per flight hour across corridor-B UAVs:")
print(f"{'policy':9} {'median':>10} {'p' + str(QPCT):>10} {'% UAVs':>8} "
      f"| over crowd: {'median':>8} {'p' + str(QPCT):>10} {'% UAVs':>8}")
for p in POL:
    med, q, frac, _, medc, qc, fracc = blind[p]
    print(f"{LABEL.get(p, p):9} {med:10.2f} {q:10.2f} {100 * frac:7.1f}% "
          f"| {'':11} {medc:8.2f} {qc:10.2f} {100 * fracc:7.1f}%")
