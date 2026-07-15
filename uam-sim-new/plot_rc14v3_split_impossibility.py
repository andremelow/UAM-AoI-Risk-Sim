#!/usr/bin/env python3
"""plot_rc14v3_split_impossibility.py -- single-cell (K8, N60) versions of the
split and impossibility figures for RC14V3 (A-heavy fleet, route_frac
[3/8,3/8,1/8,1/8]), paired with the balanced RC14 cell as reference.

Figures (saved in csv_export_RC14V3/plots):
  risk_split_mean_p99          : (a) fleet-mean R_bar_u  (b) p99[R_bar_u]
  impossibility_pexceed_aoitot : (a) % UAVs with R_bar_u > 1  (b) mean total AoI

All metrics from drone_summary only (same definitions as the grid scripts).
"""
import os, glob, argparse
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

ap = argparse.ArgumentParser()
ap.add_argument("--dir", default="csv_export_RC14V3")
ap.add_argument("--ref-dir", default="csv_export_RC14")
ap.add_argument("--scenario", default="4r_b7")
ap.add_argument("--K", type=int, default=8)
ap.add_argument("--C", type=int, default=60)
ap.add_argument("--q", type=float, default=0.99)
ap.add_argument("--thr", type=float, default=1.0)
ap.add_argument("--pol", default="round-robin-sw,pf-classic-sw,mw-sw-v8,mw-sw-v9")
args = ap.parse_args()

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
POL   = [p for p in args.pol.split(",") if p]
LABEL = {"round-robin-sw": "RR", "pf-classic-sw": "PF",
         "mw-sw-v8": "MW-V8", "mw-sw-v9": "MW-V9"}
COLORS = {"round-robin-sw": "#F9A825", "pf-classic-sw": "#E53935",
          "mw-sw-v8": "#00897B", "mw-sw-v9": "#7CB342"}
QPCT = int(round(100 * args.q))

def metrics(base, pol):
    fs = sorted(glob.glob(os.path.join(SCRIPT_DIR, base, f"K{args.K}",
                                       f"C{args.C:02d}", args.scenario,
                                       f"{pol}_*_drone_summary.csv")))
    if not fs:
        print(f"  missing: {base} {pol}")
        return dict(mean=np.nan, p99=np.nan, pexc=np.nan, aoi=np.nan)
    d = pd.read_csv(fs[-1], usecols=["mean_h1", "mean_h2", "mean_r_tot"])
    return dict(mean=float(d.mean_r_tot.mean()),
                p99=float(d.mean_r_tot.quantile(args.q)),
                pexc=100.0 * float((d.mean_r_tot > args.thr).mean()),
                aoi=float(((d.mean_h1 + d.mean_h2) / 2.0).mean()))

M_v3  = {p: metrics(args.dir,     p) for p in POL}
M_ref = {p: metrics(args.ref_dir, p) for p in POL}

plt.rcParams.update({"font.family": "serif",
                     "font.serif": ["Times New Roman", "Nimbus Roman", "DejaVu Serif"],
                     "mathtext.fontset": "stix", "font.size": 9, "pdf.fonttype": 42})

x = np.arange(len(POL))
w = 0.36

def bars(ax, key, log=False):
    for i, p in enumerate(POL):
        ax.bar(x[i] - w/2, M_ref[p][key], w, color=COLORS[p], alpha=0.45,
               edgecolor="0.3", lw=0.6)
        ax.bar(x[i] + w/2, M_v3[p][key], w, color=COLORS[p],
               edgecolor="0.15", lw=0.8, hatch="//")
        for dx, v in ((-w/2, M_ref[p][key]), (w/2, M_v3[p][key])):
            if np.isfinite(v):
                ax.annotate(f"{v:.2f}" if key != "pexc" else f"{v:.1f}",
                            (x[i] + dx, v), ha="center", va="bottom",
                            fontsize=6.5, rotation=0)
    if log:
        ax.set_yscale("log")
    ax.set_xticks(x)
    ax.set_xticklabels([LABEL[p] for p in POL], fontsize=8.5)
    ax.grid(True, axis="y", ls=":", lw=0.4, alpha=0.6)
    ax.set_axisbelow(True)

def save(fig, name, outdir):
    os.makedirs(outdir, exist_ok=True)
    base = os.path.join(outdir, name)
    for ext in ("pdf", "png"):
        fig.savefig(f"{base}.{ext}", dpi=200, bbox_inches="tight")
        print("saved", f"{base}.{ext}")

leg = [Patch(facecolor="0.75", edgecolor="0.3",
             label="RC14 balanced (25/25/25/25)"),
       Patch(facecolor="0.55", edgecolor="0.15", hatch="//",
             label="RC14V3 A-heavy (37.5/37.5/12.5/12.5)")]
OUT = os.path.join(SCRIPT_DIR, args.dir, "plots")

# ── Figure 1: risk split (mean + p99) ─────────────────────────────────
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.0, 3.1), constrained_layout=True)
bars(ax1, "mean", log=True)
ax1.set_ylabel(r"Fleet-mean risk  $\mathrm{mean}_u\,\bar R_u$")
ax1.set_title("(a) fleet mean", fontsize=9)
bars(ax2, "p99", log=True)
ax2.axhline(1.0, ls=(0, (4, 3)), lw=0.9, color="0.35")
ax2.text(len(POL) - 0.55, 1.0, " envelope", fontsize=6.5, color="0.35", va="bottom")
ax2.set_ylabel(rf"$p_{{{QPCT}}}\,[\bar R_u]$")
ax2.set_title(f"(b) fleet $p_{{{QPCT}}}$ (TLS target)", fontsize=9)
fig.suptitle(f"K = {args.K},  N = {args.C}", fontsize=10, fontweight="bold")
fig.legend(handles=leg, ncol=2, loc="lower center",
           bbox_to_anchor=(0.5, -0.10), frameon=False, fontsize=8)
save(fig, "risk_split_mean_p99", OUT)

# ── Figure 2: impossibility (P_exceed + AoI total) ────────────────────
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.0, 3.1), constrained_layout=True)
bars(ax1, "pexc")
ax1.set_ylabel(rf"UAVs with $\bar R_u > {args.thr:g}$  [%]")
ax1.set_title("(a) share of fleet above envelope", fontsize=9)
bars(ax2, "aoi", log=True)
ax2.set_ylabel(r"Mean total AoI  $(\bar h_1 + \bar h_2)/2$  [slots]")
ax2.set_title("(b) freshness cost", fontsize=9)
fig.suptitle(f"K = {args.K},  N = {args.C}", fontsize=10, fontweight="bold")
fig.legend(handles=leg, ncol=2, loc="lower center",
           bbox_to_anchor=(0.5, -0.10), frameon=False, fontsize=8)
save(fig, "impossibility_pexceed_aoitot", OUT)
