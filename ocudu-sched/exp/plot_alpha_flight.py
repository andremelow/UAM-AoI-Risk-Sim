#!/usr/bin/env python3
"""
plot_alpha_flight.py -- analisa os voos de exp/alpha_flight.py.

Previsao da politica (K=1, sem clipping): share_u(t) = sqrt(alpha_u(t)) / sum_v sqrt(alpha_v(t)).
Figura: (a) alpha(t) dos 4 drones do corredor B; (b) share de grants UL do corredor B, medido (risk e PF) x previsto.
Uso: exp/plot_alpha_flight.py [--risk risk] [--pf pf] [--win 5]
"""
import argparse
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
B = ["48", "49", "4a", "4b"]
INK, MUTED, GRID, SURF = "#1a1a19", "#6b6a63", "#e4e3dd", "#fcfcfb"
C_B = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]
C_RISK, C_PF = "#008300", "#e34948"


def load(tag):
    d = os.path.join(HERE, "logs", "flight", tag)
    m = pd.read_csv(os.path.join(d, "metrics.csv"), dtype={"rnti": str})
    a = pd.read_csv(os.path.join(d, "driver.csv"), dtype={"rnti": str})
    m["sec"] = m.t.round().astype(int)
    a["sec"] = a.t.round().astype(int)
    g = m.pivot_table(index="sec", columns="rnti", values="ul_grants", aggfunc="sum")
    al = a.pivot_table(index="sec", columns="rnti", values="alpha", aggfunc="mean")
    return g, al


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--risk", default="risk")
    ap.add_argument("--pf", default="pf")
    ap.add_argument("--win", type=int, default=5, help="media movel, s")
    args = ap.parse_args()

    g_r, al = load(args.risk)
    g_p, _ = load(args.pf)
    pred = np.sqrt(al).div(np.sqrt(al).sum(axis=1), axis=0)

    def share_b(g):
        return (g[B].sum(axis=1).rolling(args.win, center=True).sum()
                / g.sum(axis=1).rolling(args.win, center=True).sum())

    s_r, s_p = share_b(g_r), share_b(g_p)
    p_b = pred[B].sum(axis=1).rolling(args.win, center=True).mean()

    # Per-UE check over the whole flight: measured vs predicted share, second by second.
    sh = g_r.div(g_r.sum(axis=1), axis=0).rolling(args.win, center=True).mean()
    common = sh.index.intersection(pred.index)
    err = (sh.loc[common] - pred.loc[common].rolling(args.win, center=True).mean()).abs()
    print(f"corredor B, share de grants UL (media do voo): risk {g_r[B].sum().sum()/g_r.sum().sum():.3f} | "
          f"previsto {pred[B].sum(axis=1).mean():.3f} | PF {g_p[B].sum().sum()/g_p.sum().sum():.3f}")
    print(f"pico do share B: risk {s_r.max():.3f} (previsto {p_b.max():.3f}), PF {s_p.max():.3f}")
    print(f"erro absoluto por UE |medido-previsto|: media {np.nanmean(err.values):.4f}, p95 {np.nanpercentile(err.values, 95):.4f}")

    plt.rcParams.update({"font.size": 9, "axes.edgecolor": GRID, "axes.labelcolor": MUTED, "text.color": INK,
                         "xtick.color": MUTED, "ytick.color": MUTED, "figure.facecolor": SURF, "axes.facecolor": SURF})
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7.2, 5.4), sharex=True, constrained_layout=True)
    for r, c in zip(B, C_B):
        ax1.plot(al.index, al[r], color=c, lw=2, label=f"UAV 0x{r}")
    ax1.set_ylabel("risk coefficient α")
    ax1.set_title("(a) α(t) written by the driver — corridor-B drones over the crowd", loc="left", fontsize=9.5)
    ax1.legend(ncol=4, frameon=False, loc="upper left", fontsize=8)
    ax1.set_ylim(0, al.values.max() * 1.25)

    ax2.plot(p_b.index, p_b, color=MUTED, lw=2, ls=(0, (4, 2)), label="predicted  Σ√α_B / Σ√α")
    ax2.plot(s_r.index, s_r, color=C_RISK, lw=2, label="risk-aware (measured)")
    ax2.plot(s_p.index, s_p, color=C_PF, lw=2, label="PF (measured)")
    ax2.set_ylabel("UL grant share, corridor B")
    ax2.set_xlabel("flight time (s)")
    ax2.set_title("(b) Share of UL grants given to the 4 corridor-B drones (8 UEs, K=1)", loc="left", fontsize=9.5)
    ax2.set_ylim(0.3, 0.8)
    ax2.legend(frameon=False, loc="upper left", fontsize=8)
    for ax in (ax1, ax2):
        ax.grid(axis="y", color=GRID, lw=0.8)
        ax.spines[["top", "right"]].set_visible(False)
        ax.tick_params(length=0)
    out = os.path.join(HERE, "logs", "flight", "fig_alpha_flight")
    fig.savefig(out + ".png", dpi=170)
    fig.savefig(out + ".pdf")
    print("figura:", out + ".png")


if __name__ == "__main__":
    main()
