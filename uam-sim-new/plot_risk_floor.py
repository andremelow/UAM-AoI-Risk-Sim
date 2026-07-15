"""
plot_risk_floor.py
Plota o risco do sistema R_sys (RR/PF/MR) com os DOIS pisos teóricos de risco
sobrepostos:
    abs  = risco mínimo absoluto (updates perfeitos/instantâneos, h->0)
    real = risco mínimo realizável (canal+quadro: h_s=1/p_s, z=L)
ambos lower bounds verdadeiros de R_sys (risco cresce com AoI).

Pisos vêm de risk_floor.csv (run_risk_floor.m); R_sys das políticas vem do
sweep gop_final. Pisos independem de K (linha horizontal no vs-K; curva no vs-C).

Saída: csv_export_mw_otimo_a23_gop_final/plots/risk_floor_{vs_K,vs_C}_<scenario>.png
"""
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR   = os.path.join(SCRIPT_DIR, 'csv_export_mw_otimo_a23_gop_final')
OUT_DIR    = os.path.join(DATA_DIR, 'plots')
os.makedirs(OUT_DIR, exist_ok=True)

df    = pd.read_csv(os.path.join(DATA_DIR, 'sweep_summary_all.csv'))
floor = pd.read_csv(os.path.join(DATA_DIR, 'risk_floor.csv'))

POLICIES  = ['round-robin-sw', 'pf-classic-sw', 'max-risk-sw']
POL_LABEL = {'round-robin-sw': 'RR', 'pf-classic-sw': 'PF', 'max-risk-sw': 'MR (max-risk-sw)'}
COLORS    = {'round-robin-sw': '#9E9E9E', 'pf-classic-sw': '#2196F3', 'max-risk-sw': '#43A047'}
MARKERS   = {'round-robin-sw': 'o', 'pf-classic-sw': 's', 'max-risk-sw': 'D'}
ABS_STYLE  = dict(color='k',       ls='--', lw=1.8, label='Piso absoluto (h$\\to$0)')
REAL_STYLE = dict(color='#B71C1C', ls=':',  lw=2.0, label='Piso realizável (1/p, L)')

K_VALUES  = sorted(df['K'].unique())
C_VALUES  = sorted(df['capacity'].unique())
SCENARIOS = sorted(df['scenario'].unique())
C_SEL     = [v for v in [4, 16, 48] if v in C_VALUES] or C_VALUES


def fl(sc, C, col):
    r = floor[(floor.scenario == sc) & (floor.capacity == C)]
    return r[col].values[0] if not r.empty else np.nan


def save(fig, name):
    p = os.path.join(OUT_DIR, f'{name}.png')
    fig.savefig(p, dpi=150, bbox_inches='tight')
    print(f'Saved: {p}')
    plt.close(fig)


def vs_K():
    for sc in SCENARIOS:
        sub_sc = df[df.scenario == sc]
        fig, axes = plt.subplots(1, len(C_SEL), figsize=(6 * len(C_SEL), 5),
                                 constrained_layout=True)
        if len(C_SEL) == 1:
            axes = [axes]
        fig.suptitle(f'Risco $R_{{sys}}$ vs K com pisos teóricos — {sc}', fontsize=13)
        for ci, C in enumerate(C_SEL):
            ax = axes[ci]
            sub = sub_sc[sub_sc.capacity == C]
            for pol in POLICIES:
                d = sub[sub.policy == pol].sort_values('K')
                ax.plot(d.K, d.mean_r_sys, marker=MARKERS[pol], color=COLORS[pol],
                        label=POL_LABEL[pol], lw=2, ms=7)
            ax.axhline(fl(sc, C, 'Rsys_floor_real'), **REAL_STYLE)
            ax.axhline(fl(sc, C, 'Rsys_floor_abs'),  **ABS_STYLE)
            ax.set_title(f'C = {C}', fontweight='bold')
            ax.set_xlabel('K (slots por época)'); ax.set_ylabel('$R_{sys}$')
            ax.set_xticks(K_VALUES); ax.set_yscale('log')
            ax.grid(True, which='both', ls='--', alpha=0.4)
            ax.legend(fontsize=9)
        save(fig, f'risk_floor_vs_K_{sc}')


def vs_C():
    for sc in SCENARIOS:
        sub_sc = df[df.scenario == sc]
        ncols = 3
        nrows = int(np.ceil(len(K_VALUES) / ncols))
        fig, axes = plt.subplots(nrows, ncols, figsize=(16, 5 * nrows),
                                 constrained_layout=True)
        fig.suptitle(f'Risco $R_{{sys}}$ vs Capacidade com pisos teóricos — {sc}', fontsize=13)
        axes = np.array(axes).flatten()
        abs_c  = [fl(sc, C, 'Rsys_floor_abs')  for C in C_VALUES]
        real_c = [fl(sc, C, 'Rsys_floor_real') for C in C_VALUES]
        for ki, K in enumerate(K_VALUES):
            ax = axes[ki]
            sub = sub_sc[sub_sc.K == K]
            for pol in POLICIES:
                d = sub[sub.policy == pol].sort_values('capacity')
                ax.plot(d.capacity, d.mean_r_sys, marker=MARKERS[pol], color=COLORS[pol],
                        label=POL_LABEL[pol], lw=2, ms=7)
            ax.plot(C_VALUES, real_c, **REAL_STYLE)
            ax.plot(C_VALUES, abs_c,  **ABS_STYLE)
            ax.set_title(f'K = {K}', fontweight='bold')
            ax.set_xlabel('Capacidade C'); ax.set_ylabel('$R_{sys}$')
            ax.set_xticks(C_VALUES); ax.set_yscale('log')
            ax.grid(True, which='both', ls='--', alpha=0.4)
            if ki == 0:
                ax.legend(fontsize=9)
        for ax in axes[len(K_VALUES):]:
            ax.set_visible(False)
        save(fig, f'risk_floor_vs_C_{sc}')


vs_K()
vs_C()
print('\nPisos teóricos (abs + realizável) sobrepostos ao R_sys.')
