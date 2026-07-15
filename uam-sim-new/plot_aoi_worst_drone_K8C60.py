"""AoI time series of the WORST drone (highest mean total AoI) at K8/C60, RC14.

One panel per policy (RR, PF, MW-V8, MW-V9); each panel shows h1 (status/C2)
and h2 (video) AoI in seconds vs time since the drone's entry.
Worst drone = argmax over drone_summary of (mean_h1 + mean_h2)/2.
"""
import glob, os, sys
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fig_style as fs

BASE = "/mnt/stg/wrk/interlagos/simulator/v3/uam-sim-new/csv_export_RC14/K8/C60/4r_b7"
OUT = "/mnt/stg/wrk/interlagos/simulator/v3/uam-sim-new/csv_export_RC14/plots"
UPDATE_RATE = 7.0  # slots per second

POLICIES = ["round-robin-sw", "pf-classic-sw", "mw-sw-v8", "mw-sw-v9"]

fig, axes = plt.subplots(2, 2, figsize=(7.0, 4.6), sharex=False, sharey=True)

for ax, pol in zip(axes.flat, POLICIES):
    summ = pd.read_csv(sorted(glob.glob(f"{BASE}/{pol}_*_drone_summary.csv"))[-1])
    summ["aoi_tot"] = (summ.mean_h1 + summ.mean_h2) / 2.0
    worst = summ.loc[summ.aoi_tot.idxmax()]
    uid = int(worst.drone_id)

    ss = pd.read_csv(sorted(glob.glob(f"{BASE}/{pol}_*_slot_series.csv"))[-1],
                     usecols=["slot", "drone_id", "h1_slots", "h2_slots"])
    d = ss[ss.drone_id == uid].sort_values("slot")
    t = (d.slot - d.slot.iloc[0]) / UPDATE_RATE

    color, ls, _ = fs.STYLE.get(pol, ("k", "-", "o"))
    ax.plot(t, d.h1_slots / UPDATE_RATE, color=color, ls="-", lw=0.9,
            label=r"$h^{(1)}$ status")
    ax.plot(t, d.h2_slots / UPDATE_RATE, color=color, ls=":", lw=0.9, alpha=0.75,
            label=r"$h^{(2)}$ vídeo" if False else r"$h^{(2)}$ video")

    ax.set_title(f"{fs.LABEL.get(pol, pol)} — drone {uid} "
                 f"(AoI média {worst.aoi_tot/UPDATE_RATE:.1f} s, "
                 f"pico {max(worst.peak_h1, worst.peak_h2)/UPDATE_RATE:.1f} s)",
                 fontsize=8)
    ax.set_xlim(left=0)
    ax.set_yscale("log")
    ax.grid(True, which="both", lw=0.3, alpha=0.5)
    ax.legend(fontsize=7, loc="upper right", framealpha=0.9)

for ax in axes[1, :]:
    ax.set_xlabel("tempo desde a entrada do drone [s]")
for ax in axes[:, 0]:
    ax.set_ylabel("AoI [s]")

fig.suptitle("RC14 — AoI do pior drone por política (K=8, C=60)", fontsize=10)
fig.tight_layout(rect=[0, 0, 1, 0.97])
fig.savefig(f"{OUT}/aoi_worst_drone_K8C60.png", dpi=200)
fig.savefig(f"{OUT}/aoi_worst_drone_K8C60.pdf")
print("saved", OUT)
