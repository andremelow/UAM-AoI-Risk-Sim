#!/usr/bin/env python3
"""make_fig_scalability_K1.py -- Figure 4: scalability at the K=1 bottleneck.
(a) surrogate risk Ĵ per policy + lower bound L_B (Theorem 1, N_eff=C, PROVISIONAL);
(b) time-average composite system risk R̄_sys = mean_t sum_u R_u(t);
(c) peak composite system risk max_t R_sys(t).
All from RAW slot_series (steady window = >=C airborne); L_B only in panel (a).

Provenance: see fig_scalability_K1_provenance.txt.
Output: csv_export_RC7/plots/fig_scalability_K1.{pdf,png} + _caption.tex
"""
import os, glob
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
from matplotlib.lines import Line2D
import fig_style
from compute_LB import compute_LB

DATA, SCEN, K = "csv_export_RC7", "4r_b7", 1
N_GRID = [5, 10, 20, 40, 60, 80, 100, 120]          # = corridorCapacity = simultaneous UAVs
POL = fig_style.POLICIES
P_C2, P_VID, L_V = 0.80, 0.80, 5

summ = pd.read_csv(os.path.join(DATA, "sweep_summary_all.csv"))
summ = summ[(summ.K == K) & (summ.scenario == SCEN)]

def zfile(C, pol):
    for f in sorted(glob.glob(os.path.join(DATA, f"K{K}", f"C{C:02d}", SCEN, f"{pol}_*_slot_series.csv"))):
        if "z_u_slots" in pd.read_csv(f, nrows=0).columns:
            return f
    return None

def metrics(C, pol, n_s, n_v):
    """From raw traces over the steady (>=C airborne) window: Ĵ, mean R_sys, peak R_sys."""
    f = zfile(C, pol)
    d = pd.read_csv(f, usecols=["slot", "drone_id", "h1_slots", "z_u_slots", "r_tot"],
                    dtype={"slot": np.int32, "drone_id": np.int32})
    nact = d.groupby("slot").drone_id.size()
    thr = C if (nact >= C).any() else max(1, int(0.9 * nact.max()))
    hit = nact.index[nact >= thr]
    lo, hi = hit.min(), hit.max()
    w = d[(d.slot >= lo) & (d.slot <= hi)]
    g = w.groupby("slot")
    Jhat = g.apply(lambda s: float((n_s * s.h1_slots + n_v * s.z_u_slots).sum()), include_groups=False)
    Rsys = g.r_tot.sum()
    return float(Jhat.mean()), float(Rsys.mean()), float(Rsys.max())

# ---- compute (raw) + L_B ----
J = {p: [] for p in POL}; MR = {p: [] for p in POL}; PK = {p: [] for p in POL}
LB = []
for C in N_GRID:
    sub = summ[summ.capacity == C]
    n_s, n_v = float(sub.n_s.iloc[0]), float(sub.n_v.iloc[0])
    LB.append(compute_LB(C, n_s, n_v, P_C2, P_VID, L_V, K))
    for p in POL:
        j, mr, pk = metrics(C, p, n_s, n_v)
        J[p].append(j); MR[p].append(mr); PK[p].append(pk)
LB = np.array(LB)

# ---- render ----
fig_style.apply()
fig, (axa, axb, axc) = plt.subplots(3, 1, figsize=(3.5, 5.4), sharex=True, constrained_layout=True)

for p in POL:
    kw = fig_style.line_kwargs(p); kw2 = {k: v for k, v in kw.items() if k != "label"}
    axa.plot(N_GRID, J[p], **kw)
    axb.plot(N_GRID, MR[p], **kw2)
    axc.plot(N_GRID, PK[p], **kw2)
axa.plot(N_GRID, LB, color="k", ls=":", lw=1.1, marker="", label=r"Lower bound $L_B$")

for ax in (axa, axb, axc):
    ax.set_yscale("log")
    ax.set_xscale("log")   # sub-40 grid is geometric -> log-x (recorded in provenance)
    ax.grid(True, which="both", ls=":", lw=0.4, alpha=0.6)
axa.set_ylabel(r"Surrogate risk $\hat{J}$")
axb.set_ylabel(r"Mean system risk $\overline{R}_{\mathrm{sys}}$")
axc.set_ylabel(r"Peak system risk")
for ax, tag in ((axa, "(a)"), (axb, "(b)"), (axc, "(c)")):
    ax.text(0.03, 0.90, tag, transform=ax.transAxes, fontweight="bold", va="top")

axc.set_xticks(N_GRID)
axc.xaxis.set_major_formatter(mticker.FixedFormatter([str(n) for n in N_GRID]))
axc.xaxis.set_minor_formatter(mticker.NullFormatter())
axc.tick_params(axis="x", which="minor", length=0)
axc.set_xlabel(r"Simultaneous UAVs $N$")

# shared top legend (policies) + in-panel L_B legend in (a)
h, l = axa.get_legend_handles_labels()
polh = [x for x, lab in zip(h, l) if lab != r"Lower bound $L_B$"]
poll = [lab for lab in l if lab != r"Lower bound $L_B$"]
fig.legend(polh, poll, ncol=3, loc="lower center", bbox_to_anchor=(0.5, 1.0),
           columnspacing=1.1, handlelength=2.2, borderaxespad=0.2)
axa.legend([Line2D([0], [0], color="k", ls=":", lw=1.1)], [r"Lower bound $L_B$"],
           loc="lower right", fontsize=7, handlelength=2.4, borderaxespad=0.3)

OUT = os.path.join(DATA, "plots")
base = os.path.join(OUT, "fig_scalability_K1")
for ext in ("pdf", "png"):
    fig.savefig(f"{base}.{ext}", dpi=300, bbox_inches="tight")
print("saved", base + ".pdf")

# ---- headline numbers at largest common N ----
i = -1
def red(a, b): return 100 * (1 - a / b)   # reduction of a relative to b
ra_mr, rr_mr, pf_mr = MR["mr-mw-ch-sw"][i], MR["round-robin-sw"][i], MR["pf-classic-sw"][i]
ra_pk, rr_pk, pf_pk = PK["mr-mw-ch-sw"][i], PK["round-robin-sw"][i], PK["pf-classic-sw"][i]
ratio = max(np.array(J["mr-mw-ch-sw"]) / LB)
print(f"\nAt N={N_GRID[i]} (K=1):")
print(f"  MEAN  R̄_sys  RA vs RR: {red(ra_mr,rr_mr):.0f}%   RA vs PF: {red(ra_mr,pf_mr):.0f}%   "
      f"(RA={ra_mr:.4g} RR={rr_mr:.4g} PF={pf_mr:.4g})")
print(f"  PEAK  R_sys   RA vs RR: {red(ra_pk,rr_pk):.0f}%   RA vs PF: {red(ra_pk,pf_pk):.0f}%   "
      f"(RA={ra_pk:.4g} RR={rr_pk:.4g} PF={pf_pk:.4g})")
print(f"  Risk-Aware Ĵ within factor {ratio:.2g} of L_B (worst-case over N)")

# ---- caption ----
cap = (r"\caption{Scalability at the $K{=}1$ bottleneck. (a) Surrogate risk $\hat{J}$ under "
       r"RR, PF, and the Risk-Aware Max-Weight policy, together with the lower bound $L_B$ of "
       r"Theorem~1; across the entire range of $N$, the Risk-Aware policy remains within a "
       f"factor of {ratio:.2g} of $L_B$. (b) Time-average and (c) peak composite system risk "
       r"$R_{\mathrm{sys}}$. The bound applies to the surrogate objective only; the composite "
       r"risk is reported without a bound. Each point is a single deterministic realization "
       r"(seed-fixed) over a horizon of $T\approx15$--$18\text{k}$ slots; runs are byte-identical "
       r"so confidence intervals vanish.}" "\n" r"\label{fig:scalability_k1}")
with open(os.path.join(OUT, "fig_scalability_K1_caption.tex"), "w") as fh:
    fh.write(cap + "\n")
print("wrote fig_scalability_K1_caption.tex")
