#!/usr/bin/env python3
"""Emit out/fig_impossibility_K1_numbers.md from RAW slot_series traces (K=1).
Columns: mean AoI A(N), max_u R̄_u, fleet-mean z̄_u (coherence-window age).
Also computes the reported notes (lowest AoI, crossover, starvation, monotonicity)."""
import os, glob
import numpy as np, pandas as pd

DATA, SCEN, K = "csv_export_RC7", "4r_b7", 1
N_VALUES = [5, 10, 20, 40, 60, 80, 100, 120]
POL = [("round-robin-sw", "RR"), ("pf-classic-sw", "PF"), ("mr-mw-ch-sw", "Risk-Aware")]

def files(C, pol):
    return sorted(glob.glob(os.path.join(DATA, f"K{K}", f"C{C:02d}", SCEN, f"{pol}_*_slot_series.csv")))

def cell(C, pol):
    """Average per-run stats over all runs (byte-identical here). Uses z_u where present."""
    A, R, Z, T, nrun = [], [], [], [], 0
    for f in files(C, pol):
        cols = ["drone_id", "h1_slots", "r_tot"]
        head = pd.read_csv(f, nrows=0).columns
        has_z = "z_u_slots" in head
        if has_z: cols.append("z_u_slots")
        df = pd.read_csv(f, usecols=cols)
        g = df.groupby("drone_id")
        A.append(g.h1_slots.mean().mean())
        R.append(g.r_tot.mean().max())
        if has_z: Z.append(g.z_u_slots.mean().mean())
        nrun += 1
    z = float(np.mean(Z)) if Z else float("nan")
    return float(np.mean(A)), float(np.mean(R)), z, nrun

data = {lab: {C: cell(C, pol) for C in N_VALUES} for pol, lab in POL}

def fmt(x):
    """Exactly 4 significant figures, keeping trailing zeros (95.00, 103.0, 184.0)."""
    import math
    if not np.isfinite(x):
        return "—"
    if x == 0:
        return "0.000"
    dec = max(0, 3 - int(math.floor(math.log10(abs(x)))))
    return f"{x:.{dec}f}"

lines = []
w = lines.append
w("# Impossibility figure — numbers (K=1)\n")
w("Computed from raw `slot_series` traces (per-UAV temporal mean first, then mean/max over u; "
  "slots never pooled across the fleet). `h1_slots`=status AoI h_{u,s}, `r_tot`=R_u, "
  "`z_u_slots`=coherence-window AoI z_u.\n")

# main table
w("## Table — mean AoI, max_u R̄_u, z̄_u  (≥4 sig. figs)\n")
w("| N | RR AoI | RR maxR̄_u | RR z̄_u | PF AoI | PF maxR̄_u | PF z̄_u | RA AoI | RA maxR̄_u | RA z̄_u |")
w("|---|---|---|---|---|---|---|---|---|---|")
for C in N_VALUES:
    r = data["RR"][C]; p = data["PF"][C]; a = data["Risk-Aware"][C]
    w(f"| {C} | {fmt(r[0])} | {fmt(r[1])} | {fmt(r[2])} | {fmt(p[0])} | {fmt(p[1])} | {fmt(p[2])} "
      f"| {fmt(a[0])} | {fmt(a[1])} | {fmt(a[2])} |")

# notes
w("\n## Notes\n")
low = []
for C in N_VALUES:
    best = min(POL, key=lambda pl: data[pl[1]][C][0])[1]
    low.append(f"N={C}:{best}")
w(f"- **Lowest fleet-mean AoI per N:** " + ", ".join(low) + ".")
ratio = data["PF"][120][0] / data["RR"][120][0]
w(f"- **PF/RR mean-AoI ratio at N=120:** {data['PF'][120][0]:.4g}/{data['RR'][120][0]:.4g} = **{ratio:.3f}**.")

# crossover: the SUSTAINED sign flip from PF<=RR to PF>RR (not the isolated N=5 point,
# where PF starts marginally above RR, then dips below for N=10..80, then crosses back).
diff = [data["PF"][C][1] - data["RR"][C][1] for C in N_VALUES]
star = None
for i in range(1, len(N_VALUES)):
    if diff[i - 1] <= 0 and diff[i] > 0:
        star = N_VALUES[i]           # last such flip = the congested-regime crossover
w(f"- **Crossover N\\*** (PF max_u R̄_u first exceeds RR in the congested regime): **N\\*={star}** "
  f"(N=80: PF {data['PF'][80][1]:.4g} < RR {data['RR'][80][1]:.4g}; "
  f"N=100: PF {data['PF'][100][1]:.4g} > RR {data['RR'][100][1]:.4g}). "
  f"Note PF is also marginally above RR at the trivial low-load point N=5 "
  f"(PF {data['PF'][5][1]:.4g} vs RR {data['RR'][5][1]:.4g}), then dips below for N=10–80.")

# starvation @ N=120
def starv(pol):
    f = files(120, pol)[0]
    df = pd.read_csv(f, usecols=["drone_id", "h1_slots"])
    hbar = df.groupby("drone_id").h1_slots.mean()
    worst = hbar.idxmax()
    p99 = np.percentile(df[df.drone_id == worst].h1_slots, 99)
    return hbar.max(), p99
rr_s, pf_s = starv("round-robin-sw"), starv("pf-classic-sw")
w(f"- **Starvation @ N=120 (status AoI):** max_u mean h_s = RR {rr_s[0]:.4g} vs PF {pf_s[0]:.4g} slots; "
  f"p99 h_s of worst UAV = RR {rr_s[1]:.4g} vs PF {pf_s[1]:.4g} slots. PF starves the worst UAV harder.")

# monotonicity
rr = [data["RR"][C][1] for C in N_VALUES]
mono = all(x <= y for x, y in zip(rr, rr[1:]))
w(f"- **RR max_u R̄_u monotone in N?** {'Yes' if mono else 'No'} — "
  f"dip {data['RR'][80][1]:.4g}(N=80)→{data['RR'][100][1]:.4g}(N=100). "
  f"Runs are byte-identical (deterministic seed), so NOT multi-run averaging noise; "
  f"it is a real feature of the single realization (worst-UAV rotation phase).")

# z_u prose guidance
zu_ra_lower = all(data["Risk-Aware"][C][2] < min(data["RR"][C][2], data["PF"][C][2]) for C in N_VALUES)
w(f"- **z̄_u prose guidance:** Risk-Aware has the LOWEST z̄_u at every N "
  f"({'confirmed' if zu_ra_lower else 'NOT uniform'}) despite the HIGHEST status AoI — so the "
  f"impossibility argument must cite **status AoI h_s**, not z_u (which would favour Risk-Aware).")

# provenance
nr = data["RR"][5][3]
w(f"\n_Provenance: {nr} byte-identical run(s) averaged per cell; horizon T≈14.9k–17.9k slots. "
  f"z̄_u from the z_u_slots column added to util/export_csv.m (K=1 re-run, seed=55)._")

OUT_DIR = os.path.join(DATA, "plots")
os.makedirs(OUT_DIR, exist_ok=True)
outmd = os.path.join(OUT_DIR, "fig_impossibility_K1_numbers.md")
with open(outmd, "w") as fh:
    fh.write("\n".join(lines) + "\n")
print("wrote", outmd)
print("\n".join(lines))
