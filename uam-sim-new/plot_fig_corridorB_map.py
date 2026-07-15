#!/usr/bin/env python3
"""plot_fig_corridorB_map.py -- corridor-B scenario analysis figure.

Panel (a): map (East x North) — gNB, corridor A (context), corridor B with
           endpoints (Event venue / Iguatemi Mall), the 24 Gaussian ground
           hotspots (density field + centers), takeoff/landing ramp marks.
Panel (b): measured SNR along corridor B (slot_series, both directions pooled;
           mean + 10-90% band), SNR threshold, altitude profile, hotspot zone.

Geometry from build_scenario_variavel.m / experiment_RC12.m (scenario 4r_b7).

Usage:
  python3 plot_fig_corridorB_map.py [--dir csv_export_RC12] [--K 16] [--C 80]
Output: <dir>/plots/fig_corridorB_map.{pdf,png}
"""
import os, glob, argparse
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ap = argparse.ArgumentParser()
ap.add_argument("--dir", default="csv_export_RC12")
ap.add_argument("--scenario", default="4r_b7")
ap.add_argument("--K", type=int, default=16)
ap.add_argument("--C", type=int, default=80)
ap.add_argument("--pol", default="mw-sw-v9", help="slot_series source for the SNR panel "
                                                  "(channel is policy-independent)")
ap.add_argument("--hs-xs", default="0,406.1,812.3,1218.4,1624.5,2030.7,2436.8,2843.0",
                help="hotspot cluster positions [m from Event venue] along the corridor; "
                     "each gets 3 clusters (center, +-150 m perpendicular). "
                     "RC13 reduced: 1300,1685.75,2071.5,2457.25,2843")
ap.add_argument("--out", default="fig_corridorB_map", help="output basename")
args = ap.parse_args()

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(SCRIPT_DIR, args.dir)
BASE = os.path.join(DATA, f"K{args.K}", f"C{args.C:02d}", args.scenario)

# ---- geometry (build_scenario_variavel.m, coords [North, East]) -----------
GNB      = (5000.0, 0.0)              # (East, North), 30 m mast
RA_S, RA_G = (1000.0, 1000.0), (9000.0, 1000.0)          # corridor A (E, N)
IGU      = (9750.0, -350.0)           # corridor B start  = Iguatemi Mall
EVT      = (14000.0, -700.0)          # corridor B goal   = Event venue (hotspots)
ROUTE_LEN = float(np.hypot(EVT[0]-IGU[0], EVT[1]-IGU[1]))  # ~4264 m
TO       = 500.0                      # takeoff/landing ramp length [m]
CRUISE   = 60.0                       # cruise altitude [m]
SNR_THR  = 5.0                        # dB
SIGMA, AMP = 180.0, 0.75
# hotspot centers [North, East]: 3 clusters (center, +-150 m perp) per position
# in --hs-xs; the default xs reproduce the 24 manualHotspots of RC12/4r_b7,
# "1300,...,2843" reproduces the 15 of RC13/4r_b13 (experiment_RC13.m)
HS_XS = [float(v) for v in args.hs_xs.split(",") if v]
_u = np.array([EVT[0] - IGU[0], EVT[1] - IGU[1]]) / np.hypot(EVT[0]-IGU[0], EVT[1]-IGU[1])
_perp150 = np.array([-12.311, -149.497])              # (dE, dN), +150 m perpendicular
HS = []                                                # rows [North, East]
for x in HS_XS:
    ce, cn = EVT[0] - x * _u[0], EVT[1] - x * _u[1]
    for de, dn in (np.zeros(2), _perp150, -_perp150):
        HS.append([cn + dn, ce + de])
HS = np.array(HS)

plt.rcParams.update({"font.family": "serif",
                     "font.serif": ["Times New Roman", "Nimbus Roman", "DejaVu Serif"],
                     "mathtext.fontset": "stix", "font.size": 9, "pdf.fonttype": 42})
fig, (axm, axs) = plt.subplots(2, 1, figsize=(7.0, 5.6), constrained_layout=True,
                               gridspec_kw={"height_ratios": [1.25, 1.0]})

# ═══ (a) map ════════════════════════════════════════════════════════════
gx = np.arange(8500, 14900, 20.0)                     # East
gy = np.arange(-1400, 400, 20.0)                      # North
GX, GY = np.meshgrid(gx, gy)
dens = np.zeros_like(GX)
for n, e in HS:
    dens += AMP * np.exp(-((GX - e) ** 2 + (GY - n) ** 2) / (2 * SIGMA ** 2))
dens = np.where(dens < 0.02, np.nan, dens)            # transparent far field
pc = axm.pcolormesh(GX, GY, dens, cmap="Reds", shading="auto",
                    vmin=0, vmax=np.nanmax(dens), alpha=0.85, rasterized=True)
cb = fig.colorbar(pc, ax=axm, pad=0.01, aspect=22, shrink=0.75)
cb.set_label("crowd density (rel.)", fontsize=7.5)
cb.ax.tick_params(labelsize=7)
axm.scatter(HS[:, 1], HS[:, 0], s=6, color="#B71C1C", marker=".", lw=0,
            label="hotspot centers ($\\sigma$=180 m)")

# corridor A (context)
axm.plot([RA_S[0], RA_G[0]], [RA_S[1], RA_G[1]], color="0.65", lw=2.5,
         solid_capstyle="round")
axm.annotate("corridor A (8 km, bidirectional)", xy=(5000, 1000),
             xytext=(0, 6), textcoords="offset points", ha="center",
             fontsize=7.5, color="0.4")

# corridor B
axm.plot([IGU[0], EVT[0]], [IGU[1], EVT[1]], color="#C62828", lw=2.5,
         solid_capstyle="round", zorder=3)
u = np.array([EVT[0] - IGU[0], EVT[1] - IGU[1]]) / ROUTE_LEN
for f, dv in ((0.42, +1), (0.58, -1)):                # opposing direction arrows
    p0 = np.array(IGU) + f * ROUTE_LEN * u
    axm.annotate("", xy=p0 + dv * 260 * u, xytext=p0,
                 arrowprops=dict(arrowstyle="-|>", color="#C62828", lw=1.4))
# takeoff/landing ramp marks (500 m from each end)
for f in (TO, ROUTE_LEN - TO):
    p = np.array(IGU) + f * u
    axm.plot(p[0], p[1], marker="|", ms=9, mew=1.6, color="#C62828",
             transform=axm.transData)

axm.scatter(*IGU, s=45, marker="s", color="#1565C0", zorder=4)
axm.scatter(*EVT, s=70, marker="*", color="#B71C1C", zorder=4)
axm.annotate("Iguatemi Mall\n(vertiport)", xy=IGU, xytext=(-8, 14),
             textcoords="offset points", ha="right", fontsize=8, color="#1565C0")
axm.annotate("Event venue", xy=EVT, xytext=(6, 14), textcoords="offset points",
             ha="right", fontsize=8, color="#B71C1C")

# gNB
axm.scatter(*GNB, s=90, marker="^", color="k", zorder=4)
axm.annotate("gNB (30 m mast)", xy=GNB, xytext=(0, -16), textcoords="offset points",
             ha="center", fontsize=8)
for r in (3000, 6000, 9000):                          # range rings
    axm.add_patch(plt.Circle(GNB, r, fill=False, ls=":", lw=0.6, color="0.55"))
    axm.annotate(f"{r/1000:.0f} km", xy=(GNB[0] + r, -170), fontsize=6,
                 color="0.45", ha="right", va="top")

axm.set_aspect("equal")
axm.set_xlim(400, 14900); axm.set_ylim(-1500, 1500)
axm.set_xlabel("East [m]"); axm.set_ylabel("North [m]")
axm.grid(True, ls=":", lw=0.4, alpha=0.5)
axm.legend(loc="lower left", fontsize=7, frameon=False, borderaxespad=0.2)
axm.text(0.012, 0.95, "(a)", transform=axm.transAxes, fontweight="bold", va="top")

# ═══ (b) SNR + altitude along corridor B ════════════════════════════════
ds = pd.read_csv(sorted(glob.glob(os.path.join(BASE, f"{args.pol}_*_drone_summary.csv")))[-1])
ds["dur"] = ds.exit_slot - ds.entry_slot
b = ds[ds.dur < 0.7 * ds.dur.max()].sort_values("drone_id")
ids_to = set(b.drone_id.values[: len(b) // 2])
ss = pd.read_csv(sorted(glob.glob(os.path.join(BASE, f"{args.pol}_*_slot_series.csv")))[-1],
                 usecols=["slot", "drone_id", "snr_db"])
ss = ss[ss.drone_id.isin(set(b.drone_id))].merge(
        ds[["drone_id", "entry_slot", "dur"]], on="drone_id")
trav = (ss.slot - ss.entry_slot) / ss.dur * ROUTE_LEN
ss["dist"] = np.where(ss.drone_id.isin(ids_to), ROUTE_LEN - trav, trav)

edges = np.arange(0.0, ROUTE_LEN + 50.0, 50.0)
centers = 0.5 * (edges[:-1] + edges[1:])
g = ss.groupby(pd.cut(ss.dist, edges, labels=False), observed=True).snr_db
m  = g.mean().reindex(range(len(centers)))
lo = g.quantile(0.10).reindex(range(len(centers)))
hi = g.quantile(0.90).reindex(range(len(centers)))

axs.fill_between(centers, lo, hi, color="#1565C0", alpha=0.20, lw=0,
                 label="10–90% (fleet, both directions)")
axs.plot(centers, m, color="#1565C0", lw=1.8, label="mean SNR")
axs.axhline(SNR_THR, ls=(0, (4, 3)), lw=1.0, color="#C62828")
axs.text(0.99, SNR_THR, "SNR threshold (5 dB) ", transform=axs.get_yaxis_transform(),
         ha="right", va="bottom", fontsize=7, color="#C62828")

hs_lo, hs_end = min(HS_XS), max(HS_XS)
axs.axvspan(hs_lo, hs_end, color="0.80", alpha=0.5, lw=0, zorder=0)
axs.text(hs_end, 0.97, "hotspot zone ", transform=axs.get_xaxis_transform(),
         ha="right", va="top", fontsize=7, color="0.35")
for xb in (TO, ROUTE_LEN - TO):
    axs.axvline(xb, color="0.45", lw=0.8, ls="-.", alpha=0.8)

# altitude profile (identical for both directions: ramps in first/last 500 m)
alt = np.minimum(CRUISE, np.minimum(centers, ROUTE_LEN - centers) / TO * CRUISE)
axa = axs.twinx()
axa.plot(centers, alt, color="0.35", lw=1.2, ls=":")
axa.set_ylabel("altitude AGL [m]", fontsize=8, color="0.35")
axa.set_ylim(0, 260); axa.set_yticks([0, 60])
axa.tick_params(labelsize=7, colors="0.35")
axa.text(ROUTE_LEN / 2, CRUISE + 8, "altitude profile", ha="center", fontsize=6.5,
         color="0.35", style="italic")

axs.set_xlim(0, ROUTE_LEN)
axs.set_ylim(-25, 40)
axs.set_xlabel("Position along corridor B  [m]")
axs.set_ylabel("SNR [dB]")
axs.grid(True, ls=":", lw=0.4, alpha=0.5)
axs.legend(loc="upper right", fontsize=7, frameon=False)
axs.text(0.012, 0.95, "(b)", transform=axs.transAxes, fontweight="bold", va="top")
for x, name, ha in ((0, "Event venue", "left"), (ROUTE_LEN, "Iguatemi Mall", "right")):
    axs.annotate(name, xy=(x, 0), xycoords=("data", "axes fraction"),
                 xytext=(0, -28), textcoords="offset points",
                 ha=ha, va="top", fontsize=8, style="italic", color="0.25")

OUT = os.path.join(DATA, "plots")
os.makedirs(OUT, exist_ok=True)
base = os.path.join(OUT, args.out)
for ext in ("pdf", "png"):
    fig.savefig(f"{base}.{ext}", dpi=200, bbox_inches="tight")
    print("saved", f"{base}.{ext}")

d_igu = np.hypot(IGU[0]-GNB[0], IGU[1]-GNB[1])
d_evt = np.hypot(EVT[0]-GNB[0], EVT[1]-GNB[1])
print(f"gNB->Iguatemi {d_igu:.0f} m | gNB->Event venue {d_evt:.0f} m | corridor {ROUTE_LEN:.0f} m")
