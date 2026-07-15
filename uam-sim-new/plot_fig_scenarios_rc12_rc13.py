#!/usr/bin/env python3
"""plot_fig_scenarios_rc12_rc13.py -- RC12 vs RC13 corridor-B scenarios, one figure.

Two map panels (per-panel window, equal 6.6 km span, shared density colorbar):
  (a) RC12 / 4r_b7  : route 9750->14000 (4.26 km), 24 clusters up to the venue
  (b) RC13 / 4r_b13 : route MOVED next to the gNB (start 1 km away, venue at
      5.0 km), 4.00 km long; 24 clusters in zone [300, 3400] m from the venue
sigma=180 m, amp=0.75, +-150 m lateral offsets everywhere.

Usage: python3 plot_fig_scenarios_rc12_rc13.py [--out-dir csv_export_RC13/plots]
Output: <out-dir>/fig_scenarios_rc12_rc13.{pdf,png}
"""
import os, argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patheffects as pe

ap = argparse.ArgumentParser()
ap.add_argument("--out-dir", default="csv_export_RC13/plots")
args = ap.parse_args()

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
GNB = (5000.0, 0.0)                                   # (East, North), 30 m mast
U = np.array([4250.0, -350.0]) / np.hypot(4250.0, 350.0)   # corridor direction
PERP150 = np.array([-12.311, -149.497])               # (dE, dN)
SIGMA, AMP, TO = 180.0, 0.75, 500.0

def clusters(xs, evt):
    out = []
    for x in xs:
        ce, cn = evt[0] - x*U[0], evt[1] - x*U[1]
        for de, dn in (np.zeros(2), PERP150, -PERP150):
            out.append([ce+de, cn+dn])
    return np.array(out)                               # [East, North]

#         title, hotspot xs, start, start label, venue, xlim (6.6 km span)
SCEN = [("(a) RC12 — route B 4.26 km, crowd up to the venue (9.0 km from gNB)",
         np.linspace(0.0, 2843.0, 8), (9750.0, -350.0), "Iguatemi Mall",
         (14000.0, -700.0), (8600, 15200)),
        ("(b) RC13 — route B 4.00 km moved next to the gNB; crowd zone [300, 3400] m",
         np.linspace(300.0, 3400.0, 8), (5999.1, -41.1), "Vertiport\n(1 km from gNB)",
         (9985.6, -369.4), (4400, 11000))]

plt.rcParams.update({"font.family": "serif",
                     "font.serif": ["Times New Roman", "Nimbus Roman", "DejaVu Serif"],
                     "mathtext.fontset": "stix", "font.size": 9, "pdf.fonttype": 42})
fig, axes = plt.subplots(2, 1, figsize=(7.0, 4.9), constrained_layout=True)

pc = None
for ax, (title, xs, sta, sta_lab, evt, xlim) in zip(axes, SCEN):
    hs = clusters(xs, evt)
    gx = np.arange(xlim[0], xlim[1], 15.0)
    gy = np.arange(-1350, 400, 15.0)
    GX, GY = np.meshgrid(gx, gy)
    dens = np.zeros_like(GX)
    for e, n in hs:
        dens += AMP * np.exp(-((GX-e)**2 + (GY-n)**2) / (2*SIGMA**2))
    dens = dens / dens.max()             # in-sim the peak is normalized to rho_op
    d = np.where(dens < 0.02, np.nan, dens)
    pc = ax.pcolormesh(GX, GY, d, cmap="Reds", shading="auto", vmin=0, vmax=1.0,
                       alpha=0.9, rasterized=True)
    ax.scatter(hs[:, 0], hs[:, 1], s=6, color="#B71C1C", marker=".", lw=0)

    rlen = float(np.hypot(evt[0]-sta[0], evt[1]-sta[1]))
    ax.plot([sta[0], evt[0]], [sta[1], evt[1]], color="#C62828", lw=2.2,
            solid_capstyle="round", zorder=3)
    for f in (TO, rlen - TO):                          # ramp marks
        p = np.array(sta) + f*U
        ax.plot(p[0], p[1], marker="|", ms=8, mew=1.5, color="#C62828")
    ax.scatter(*sta, s=40, marker="s", color="#1565C0", zorder=4)
    ax.scatter(*evt, s=65, marker="*", color="#B71C1C", zorder=4)
    ax.annotate(sta_lab, xy=sta, xytext=(-2, -14), textcoords="offset points",
                ha="right", va="top", fontsize=8, color="#1565C0")
    ax.annotate("Event venue", xy=evt, xytext=(4, 10), textcoords="offset points",
                ha="right", fontsize=8, color="#B71C1C",
                path_effects=[pe.withStroke(linewidth=2.2, foreground="white")])
    if xlim[0] <= GNB[0] <= xlim[1]:                   # gNB inside the window
        ax.scatter(*GNB, s=90, marker="^", color="k", zorder=4)
        ax.annotate("gNB (30 m mast)", xy=GNB, xytext=(0, -16),
                    textcoords="offset points", ha="center", fontsize=8)
    else:
        d_gnb = np.hypot(sta[0]-GNB[0], sta[1]-GNB[1]) / 1000
        ax.annotate(f"to gNB ({d_gnb:.1f} km)", xy=(xlim[0]+300, -150),
                    xytext=(xlim[0]+1150, -150), textcoords="data", ha="left",
                    va="center", fontsize=7, color="0.35",
                    arrowprops=dict(arrowstyle="-|>", color="0.35", lw=1.0))

    ax.set_aspect("equal")
    ax.set_xlim(*xlim); ax.set_ylim(-1350, 400)
    ax.set_ylabel("North [m]")
    ax.grid(True, ls=":", lw=0.4, alpha=0.5)
    ax.set_title(title, fontsize=8.5, loc="left", pad=3)

axes[1].set_xlabel("East [m]")
cb = fig.colorbar(pc, ax=axes, pad=0.015, aspect=30)
cb.set_label(r"crowd density (rel. to $\rho_{op}$)", fontsize=7.5)
cb.ax.tick_params(labelsize=7)

OUT = os.path.join(SCRIPT_DIR, args.out_dir)
os.makedirs(OUT, exist_ok=True)
base = os.path.join(OUT, "fig_scenarios_rc12_rc13")
for ext in ("pdf", "png"):
    fig.savefig(f"{base}.{ext}", dpi=200, bbox_inches="tight")
    print("saved", f"{base}.{ext}")
