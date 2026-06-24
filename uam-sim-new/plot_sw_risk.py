"""
plot_sw_risk.py
Risco médio e pico de risco do sistema — sweep 2r_variavel
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

CSV = 'csv_export_debug/sw_sweep/sweep_summary_all.csv'
df  = pd.read_csv(CSV)

POLICIES  = ['round-robin-sw', 'pf-classic-sw', 'max-weight-sw', 'aoi-sw']
POL_LABEL = {'round-robin-sw': 'Round-Robin', 'pf-classic-sw': 'PF-Classic',
             'max-weight-sw': 'Max-Weight',  'aoi-sw': 'AoI-SW'}
COLORS    = {'round-robin-sw': '#9E9E9E', 'pf-classic-sw': '#2196F3',
             'max-weight-sw': '#4CAF50',  'aoi-sw': '#FF5722'}
MARKERS   = {'round-robin-sw': 'o', 'pf-classic-sw': 's',
             'max-weight-sw': '^',  'aoi-sw': 'D'}

K_VALUES = sorted(df['K'].unique())
C_VALUES = sorted(df['capacity'].unique())

def make_risk_fig(metric, ylabel, title_prefix, fname):
    fig, axes = plt.subplots(2, 3, figsize=(16, 9), constrained_layout=True)
    fig.suptitle(f'{title_prefix} vs Capacidade — cenário 2r_variavel', fontsize=13)
    axes = axes.flatten()

    for ki, K in enumerate(K_VALUES):
        ax   = axes[ki]
        sub  = df[df['K'] == K]
        for pol in POLICIES:
            d = sub[sub['policy'] == pol].sort_values('capacity')
            ax.plot(d['capacity'], d[metric], marker=MARKERS[pol],
                    color=COLORS[pol], label=POL_LABEL[pol], lw=2, ms=7)
        ax.set_title(f'K = {K}', fontweight='bold')
        ax.set_xlabel('Capacidade C')
        ax.set_ylabel(ylabel)
        ax.set_xticks(C_VALUES)
        ax.yaxis.set_major_formatter(ticker.ScalarFormatter(useMathText=True))
        ax.ticklabel_format(axis='y', style='sci', scilimits=(0, 0))
        ax.grid(True, ls='--', alpha=0.4)
        if ki == 0:
            ax.legend(fontsize=8)

    fig.savefig(fname, dpi=150, bbox_inches='tight')
    print(f'Saved: {fname}')
    return fig

# ── Fig 1: risco médio ────────────────────────────────────────────────────────
make_risk_fig('mean_r_sys', 'Risco médio do sistema  R̄_sys',
              'Risco médio do sistema (mean_r_sys)', 'sweep_risk_mean.png')

# ── Fig 2: pico de risco ──────────────────────────────────────────────────────
make_risk_fig('peak_r_sys', 'Pico de risco  R_peak',
              'Pico de risco do sistema (peak_r_sys)', 'sweep_risk_peak.png')

# ── Fig 3: heatmap — ganho de risco médio MW vs RR ────────────────────────────
fig3, axes = plt.subplots(1, 3, figsize=(16, 5), constrained_layout=True)
fig3.suptitle('Redução de risco médio relativa ao Round-Robin — 2r_variavel', fontsize=13)

compare_pols = ['pf-classic-sw', 'max-weight-sw', 'aoi-sw']
for ci, pol in enumerate(compare_pols):
    ax  = axes[ci]
    mat = np.zeros((len(K_VALUES), len(C_VALUES)))
    for ki, K in enumerate(K_VALUES):
        for cj, C in enumerate(C_VALUES):
            rr = df[(df['K']==K) & (df['capacity']==C) & (df['policy']=='round-robin-sw')]['mean_r_sys']
            pp = df[(df['K']==K) & (df['capacity']==C) & (df['policy']==pol)]['mean_r_sys']
            if len(rr) and len(pp) and rr.values[0] > 0:
                mat[ki, cj] = (rr.values[0] - pp.values[0]) / rr.values[0] * 100

    im = ax.imshow(mat, cmap='RdYlGn', aspect='auto', vmin=-20, vmax=20)
    ax.set_xticks(range(len(C_VALUES))); ax.set_xticklabels(C_VALUES)
    ax.set_yticks(range(len(K_VALUES))); ax.set_yticklabels(K_VALUES)
    ax.set_xlabel('Capacidade C')
    ax.set_ylabel('K (slots/época)')
    ax.set_title(f'{POL_LABEL[pol]} vs RR', fontweight='bold')
    for ki in range(len(K_VALUES)):
        for cj in range(len(C_VALUES)):
            ax.text(cj, ki, f'{mat[ki,cj]:+.1f}%', ha='center', va='center',
                    fontsize=7.5, color='black')
    plt.colorbar(im, ax=ax, label='Redução de risco (%)\n(+ = melhor que RR)')

fig3.savefig('sweep_risk_heatmap.png', dpi=150, bbox_inches='tight')
print('Saved: sweep_risk_heatmap.png')

plt.close('all')
print('\nTodos os plots de risco salvos.')
