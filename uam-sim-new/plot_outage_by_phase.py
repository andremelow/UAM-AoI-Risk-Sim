#!/usr/bin/env python3
"""plot_outage_by_phase.py -- outage analysis per policy per flight phase.

Phases per UAV: takeoff = first 500 m, landing = final 500 m, cruise = middle
(corridor A 8 km vs B 4.264 km classified by flight duration). Outage =
SNR < thresholdSNR (5 dB), which is trajectory-driven and hence identical
across policies (same seed); what differs per policy is the RESPONSE:

  (a) served share of slots per phase;
  (b) served share DURING OUTAGE slots (wasted grants: the SNR gate of the
      -ch policies makes this ~0; blind policies burn channel there);
  (c) mean C2 AoI h1 per phase (who lets C2 age in the ramps);
  (d) mean composite risk R_u per phase (what actually matters).

Rows of panels split the fleet by corridor: Route A (8 km) vs Route B
(4.264 km, hotspot zone at the end), classified by flight duration.

Usage:
  python3 plot_outage_by_phase.py --dir csv_export_RC11 --K 16 --C 80
Output: <dir>/plots/fig_outage_by_phase_K{K}_C{C}.{pdf,png} + console table
"""
import os, glob, argparse
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

ap = argparse.ArgumentParser()
ap.add_argument("--dir", default="csv_export_RC11")
ap.add_argument("--scenario", default="4r_b7")
ap.add_argument("--K", type=int, default=16)
ap.add_argument("--C", type=int, default=80)
ap.add_argument("--thr", type=float, default=5.0, help="outage SNR threshold [dB]")
ap.add_argument("--pol", default="round-robin-sw,pf-classic-sw,mw-sw-v9,mw-v9-nsw")
ap.add_argument("--outbound", action="store_true",
                help="restrict to the OUTBOUND routes only (RouteA and RouteBhalf; "
                     "ids are assigned sequentially per route: first half of each "
                     "corridor block is the outbound leg)")
args = ap.parse_args()

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(SCRIPT_DIR, args.dir)
K, C = args.K, args.C

POL    = [p for p in args.pol.split(",") if p]
LABEL  = {"round-robin-sw": "RR", "pf-classic-sw": "PF", "mr-mw-ch-sw": "Risk-Aware",
          "mw-sw-v9": "MW-V9", "mw-v9-nsw": "MW-V9-nsw", "mw-sw-v8": "MW-V8"}
COLORS = {"round-robin-sw": "#F9A825", "pf-classic-sw": "#E53935", "mr-mw-ch-sw": "#8E24AA",
          "mw-sw-v9": "#7CB342", "mw-v9-nsw": "#33691E", "mw-sw-v8": "#00897B"}
PHASES = ["takeoff", "cruise", "landing"]

rows = []
for pol in POL:
    fs = sorted(glob.glob(os.path.join(DATA, f"K{K}", f"C{C:02d}", args.scenario,
                                        f"{pol}_*_slot_series.csv")))[-1]
    ds = pd.read_csv(fs.replace("_slot_series", "_drone_summary"),
                     usecols=["drone_id", "entry_slot", "exit_slot"])
    ds["dur"] = ds.exit_slot - ds.entry_slot
    ds["fr"] = np.where(ds.dur > ds.dur.min() * 1.5, 500.0/8000.0, 500.0/4264.4)
    ss = pd.read_csv(fs, usecols=["slot", "drone_id", "snr_db", "h1_slots",
                                  "r_tot", "source_sched"],
                     dtype={"slot": np.int32, "drone_id": np.int32}).merge(ds, on="drone_id")
    ss["prog"] = (ss.slot - ss.entry_slot) / ss.dur
    ss["phase"] = np.select([ss.prog < ss.fr, ss.prog > 1 - ss.fr],
                            ["takeoff", "landing"], "cruise")
    ss["route"] = np.where(ss.dur > ss.dur.min() * 1.5, "A", "B")
    if args.outbound:
        # outbound = first half of each corridor's contiguous id block
        long_ids  = sorted(ds[ds.dur > ds.dur.min() * 1.5].drone_id)
        short_ids = sorted(ds[~(ds.dur > ds.dur.min() * 1.5)].drone_id)
        keep = set(long_ids[:len(long_ids)//2]) | set(short_ids[:len(short_ids)//2])
        ss = ss[ss.drone_id.isin(keep)]
    ss["out"] = ss.snr_db < args.thr
    ss["srv"] = ss.source_sched > 0
    for (rt, ph), d in ss.groupby(["route", "phase"]):
        out = d[d.out]
        rows.append(dict(policy=pol, route=rt, phase=ph,
                         outage=100 * d.out.mean(),
                         served=100 * d.srv.mean(),
                         served_in_outage=100 * out.srv.mean() if len(out) else np.nan,
                         h1=d.h1_slots.mean(),
                         r_tot=d.r_tot.mean()))
    print(f"done {pol}", flush=True)

T = pd.DataFrame(rows)
print("\n=== outage & resposta por fase x rota (K=%d, C=%d, thr=%.0f dB) ===" % (K, C, args.thr))
print(T.assign(policy=T.policy.map(LABEL)).set_index(["route", "policy", "phase"])
       .sort_index().round(2))

plt.rcParams.update({"font.family": "serif",
                     "font.serif": ["Times New Roman", "Nimbus Roman", "DejaVu Serif"],
                     "mathtext.fontset": "stix", "font.size": 8,
                     "xtick.labelsize": 7.5, "ytick.labelsize": 7, "pdf.fonttype": 42})
ROUTES = ["A", "B"]
fig, axes2 = plt.subplots(2, 4, figsize=(9.6, 4.9), constrained_layout=True)
METRICS = [("served", "Served slots  [%]", False),
           ("served_in_outage", "Served DURING outage  [%]\n(wasted grants)", False),
           ("h1", "Mean C2 AoI $h_{u,s}$ (slots)", False),
           ("r_tot", "Mean risk $R_u$", True)]

x = np.arange(len(PHASES))
W = 0.8 / len(POL)
for ri, rt in enumerate(ROUTES):
    Tr = T[T.route == rt]
    for ax, (col, lab, logy) in zip(axes2[ri], METRICS):
        for pi, pol in enumerate(POL):
            v = [Tr[(Tr.policy == pol) & (Tr.phase == ph)][col].iloc[0] for ph in PHASES]
            ax.bar(x + (pi - (len(POL)-1)/2) * W, v, W, color=COLORS[pol],
                   edgecolor="0.25", lw=0.4)
        ax.set_xticks(x); ax.set_xticklabels(PHASES)
        if ri == 0:
            ax.set_xticklabels([])
        ax.set_ylabel((f"Route {rt}\n" if ax is axes2[ri][0] else "") + lab, fontsize=7.5)
        if logy:
            ax.set_yscale("log")
        ax.grid(True, axis="y", which="both", ls=":", lw=0.4, alpha=0.6)
    o = [Tr[(Tr.policy == POL[0]) & (Tr.phase == ph)].outage.iloc[0] for ph in PHASES]
    axes2[ri][0].set_title(f"Route {rt} — outage: {o[0]:.1f}% / {o[1]:.1f}% / {o[2]:.1f}%",
                           fontsize=7.5, color="0.35")

handles = [Patch(facecolor=COLORS[p], edgecolor="0.25", label=LABEL.get(p, p)) for p in POL]
fig.legend(handles=handles, ncol=len(POL), loc="lower center",
           bbox_to_anchor=(0.5, -0.09), frameon=False, fontsize=8)
OTAG = " — OUTBOUND legs only" if args.outbound else ""
fig.suptitle(f"Outage response by flight phase{OTAG} — K={K}, C={C} (outage = SNR < {args.thr:.0f} dB)",
             fontsize=9, fontweight="bold")

OUT = os.path.join(DATA, "plots")
os.makedirs(OUT, exist_ok=True)
base = os.path.join(OUT, f"fig_outage_by_phase_routes_K{K}_C{C}"
                    + ("_outbound" if args.outbound else ""))
for ext in ("pdf", "png"):
    fig.savefig(f"{base}.{ext}", dpi=170, bbox_inches="tight")
    print("saved", f"{base}.{ext}")
