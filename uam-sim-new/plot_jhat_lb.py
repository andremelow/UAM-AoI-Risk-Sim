"""
plot_jhat_lb.py
Plota o custo surrogado empírico Jhat_emp (= sum_u omega_u[n_s h_s + n_v h_v])
das políticas RR/PF/MR vs K e vs C, com o LOWER BOUND TEÓRICO LB_K do LaTeX
sobreposto.

LB_K (doc):
    LB_K = (1/2K)[ sum_u sqrt(w n_s/p_s) + sum_u sqrt(w n_v L/p_v) ]^2
         + sum_u w (n_s + n_v)

Diferença vs o J_LB_doc do simulador: usamos N = nº MÉDIO de drones
SIMULTANEAMENTE no ar (medido), não o total cfg.numDrones — o que corrige o
rho_emp<1 (LB inflado por N^2). omega=1 (uniforme), p_s=p_v=0.8, L=L_vid=5.

Saída: csv_export_mw_otimo_a23_gop_final/plots/jhat_{vs_K,vs_C}_<scenario>.png
"""
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR   = os.path.join(SCRIPT_DIR, 'csv_export_mw_otimo_a23_gop_final')
OUT_DIR    = os.path.join(DATA_DIR, 'plots')
os.makedirs(OUT_DIR, exist_ok=True)

df = pd.read_csv(os.path.join(DATA_DIR, 'sweep_summary_all.csv'))

# Nº médio de drones simultâneos (medido via init_scenario, regime estacionário)
NSIM = {
    ('2r_static', 2): 2.0, ('2r_static', 4): 3.6, ('2r_static', 8): 7.0,
    ('2r_static', 16): 13.7, ('2r_static', 32): 27.1, ('2r_static', 48): 40.7,
    ('2r_static', 64): 54.1,
    ('2r_a23', 2): 2.0, ('2r_a23', 4): 3.4, ('2r_a23', 8): 6.8,
    ('2r_a23', 16): 12.9, ('2r_a23', 32): 26.0, ('2r_a23', 48): 38.5,
    ('2r_a23', 64): 51.3,
}
L, PS, PV = 5.0, 0.8, 0.8


def lb_k(N, ns, nv, K):
    s1 = N * np.sqrt(ns / PS)
    s2 = N * np.sqrt(nv * L / PV)
    return (s1 + s2) ** 2 / (2.0 * K) + N * (ns + nv)


# n_s, n_v por (scenario, C, K): policy-independente -> média entre políticas
nsnv = (df.groupby(['scenario', 'capacity', 'K'], as_index=False)
          [['n_s', 'n_v']].mean())


def lb_value(sc, C, K):
    row = nsnv[(nsnv.scenario == sc) & (nsnv.capacity == C) & (nsnv.K == K)]
    if row.empty or (sc, C) not in NSIM:
        return np.nan
    return lb_k(NSIM[(sc, C)], row.n_s.values[0], row.n_v.values[0], K)


POLICIES  = ['round-robin-sw', 'pf-classic-sw', 'max-risk-sw']
POL_LABEL = {'round-robin-sw': 'RR', 'pf-classic-sw': 'PF', 'max-risk-sw': 'MR (max-risk-sw)'}
COLORS    = {'round-robin-sw': '#9E9E9E', 'pf-classic-sw': '#2196F3', 'max-risk-sw': '#43A047'}
MARKERS   = {'round-robin-sw': 'o', 'pf-classic-sw': 's', 'max-risk-sw': 'D'}
LB_STYLE  = dict(color='k', ls=':', marker='*', ms=9, lw=1.9, label='LB$_K$ teórico (N simultâneo)')

K_VALUES  = sorted(df['K'].unique())
C_VALUES  = sorted(df['capacity'].unique())
SCENARIOS = sorted(df['scenario'].unique())
C_SEL     = [v for v in [4, 16, 48] if v in C_VALUES] or C_VALUES


def save(fig, name):
    p = os.path.join(OUT_DIR, f'{name}.png')
    fig.savefig(p, dpi=150, bbox_inches='tight')
    print(f'Saved: {p}')
    plt.close(fig)


def jhat_vs_K():
    for sc in SCENARIOS:
        sub_sc = df[df.scenario == sc]
        fig, axes = plt.subplots(1, len(C_SEL), figsize=(6 * len(C_SEL), 5),
                                 constrained_layout=True)
        if len(C_SEL) == 1:
            axes = [axes]
        fig.suptitle(f'Custo surrogado $\\widehat{{J}}$ vs K — {sc}', fontsize=13)
        for ci, C in enumerate(C_SEL):
            ax = axes[ci]
            sub = sub_sc[sub_sc.capacity == C]
            for pol in POLICIES:
                d = sub[sub.policy == pol].sort_values('K')
                ax.plot(d.K, d.Jhat_emp, marker=MARKERS[pol], color=COLORS[pol],
                        label=POL_LABEL[pol], lw=2, ms=7)
            lbv = [lb_value(sc, C, K) for K in K_VALUES]
            ax.plot(K_VALUES, lbv, **LB_STYLE)
            ax.set_title(f'C = {C}', fontweight='bold')
            ax.set_xlabel('K (slots por época)'); ax.set_ylabel('$\\widehat{J}$ (surrogate)')
            ax.set_xticks(K_VALUES)
            ax.set_yscale('log')
            ax.grid(True, which='both', ls='--', alpha=0.4)
            ax.legend(fontsize=9)
        save(fig, f'jhat_lb_vs_K_{sc}')


def jhat_vs_C():
    for sc in SCENARIOS:
        sub_sc = df[df.scenario == sc]
        ncols = 3
        nrows = int(np.ceil(len(K_VALUES) / ncols))
        fig, axes = plt.subplots(nrows, ncols, figsize=(16, 5 * nrows),
                                 constrained_layout=True)
        fig.suptitle(f'Custo surrogado $\\widehat{{J}}$ vs Capacidade — {sc}', fontsize=13)
        axes = np.array(axes).flatten()
        for ki, K in enumerate(K_VALUES):
            ax = axes[ki]
            sub = sub_sc[sub_sc.K == K]
            for pol in POLICIES:
                d = sub[sub.policy == pol].sort_values('capacity')
                ax.plot(d.capacity, d.Jhat_emp, marker=MARKERS[pol], color=COLORS[pol],
                        label=POL_LABEL[pol], lw=2, ms=7)
            lbv = [lb_value(sc, C, K) for C in C_VALUES]
            ax.plot(C_VALUES, lbv, **LB_STYLE)
            ax.set_title(f'K = {K}', fontweight='bold')
            ax.set_xlabel('Capacidade C'); ax.set_ylabel('$\\widehat{J}$ (surrogate)')
            ax.set_xticks(C_VALUES)
            ax.set_yscale('log')
            ax.grid(True, which='both', ls='--', alpha=0.4)
            if ki == 0:
                ax.legend(fontsize=9)
        for ax in axes[len(K_VALUES):]:
            ax.set_visible(False)
        save(fig, f'jhat_lb_vs_C_{sc}')


jhat_vs_K()
jhat_vs_C()
print('\nLB_K teórico (N simultâneo medido) sobreposto às curvas de Jhat.')
