"""
plot_sw_sweep_results.py
Resultados do sweep 2r_variavel: 6K × 7C × 4 políticas
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from matplotlib.gridspec import GridSpec

CSV = 'csv_export_debug/sw_sweep/sweep_summary_all.csv'
df = pd.read_csv(CSV)

POLICIES  = ['round-robin-sw', 'pf-classic-sw', 'max-weight-sw', 'aoi-sw']
POL_LABEL = {'round-robin-sw': 'Round-Robin', 'pf-classic-sw': 'PF-Classic',
             'max-weight-sw': 'Max-Weight',  'aoi-sw': 'AoI-SW'}
COLORS    = {'round-robin-sw': '#9E9E9E', 'pf-classic-sw': '#2196F3',
             'max-weight-sw': '#4CAF50',  'aoi-sw': '#FF5722'}
MARKERS   = {'round-robin-sw': 'o', 'pf-classic-sw': 's',
             'max-weight-sw': '^',  'aoi-sw': 'D'}

K_VALUES = sorted(df['K'].unique())
C_VALUES = sorted(df['capacity'].unique())

# ─────────────────────────────────────────────────────────────────────────────
# Fig 1 — mean_h1 (AoI C2) vs C, um subplot por K
# ─────────────────────────────────────────────────────────────────────────────
fig1, axes = plt.subplots(2, 3, figsize=(16, 9), constrained_layout=True)
fig1.suptitle('AoI médio C2 (mean_h1) vs Capacidade — cenário 2r_variavel', fontsize=13)
axes = axes.flatten()

for ki, K in enumerate(K_VALUES):
    ax = axes[ki]
    sub = df[df['K'] == K]
    for pol in POLICIES:
        d = sub[sub['policy'] == pol].sort_values('capacity')
        ax.plot(d['capacity'], d['mean_h1'], marker=MARKERS[pol],
                color=COLORS[pol], label=POL_LABEL[pol], lw=2, ms=7)
    ax.set_title(f'K = {K}', fontweight='bold')
    ax.set_xlabel('Capacidade C')
    ax.set_ylabel('mean h₁ (slots)')
    ax.set_xticks(C_VALUES)
    ax.grid(True, ls='--', alpha=0.4)
    if ki == 0:
        ax.legend(fontsize=8)

fig1.savefig('sweep_h1_vs_C.png', dpi=150, bbox_inches='tight')
print('Saved: sweep_h1_vs_C.png')

# ─────────────────────────────────────────────────────────────────────────────
# Fig 2 — Jhat_emp vs C, um subplot por K
# ─────────────────────────────────────────────────────────────────────────────
fig2, axes = plt.subplots(2, 3, figsize=(16, 9), constrained_layout=True)
fig2.suptitle('Custo empírico Ĵ vs Capacidade — cenário 2r_variavel', fontsize=13)
axes = axes.flatten()

for ki, K in enumerate(K_VALUES):
    ax = axes[ki]
    sub = df[df['K'] == K]
    for pol in POLICIES:
        d = sub[sub['policy'] == pol].sort_values('capacity')
        ax.plot(d['capacity'], d['Jhat_emp'], marker=MARKERS[pol],
                color=COLORS[pol], label=POL_LABEL[pol], lw=2, ms=7)
    ax.set_title(f'K = {K}', fontweight='bold')
    ax.set_xlabel('Capacidade C')
    ax.set_ylabel('Ĵ empírico')
    ax.set_xticks(C_VALUES)
    ax.yaxis.set_major_formatter(ticker.ScalarFormatter(useMathText=True))
    ax.ticklabel_format(axis='y', style='sci', scilimits=(0,0))
    ax.grid(True, ls='--', alpha=0.4)
    if ki == 0:
        ax.legend(fontsize=8)

fig2.savefig('sweep_Jhat_vs_C.png', dpi=150, bbox_inches='tight')
print('Saved: sweep_Jhat_vs_C.png')

# ─────────────────────────────────────────────────────────────────────────────
# Fig 3 — heatmap: ganho MW vs RR  (mean_h1) para cada (K, C)
# ─────────────────────────────────────────────────────────────────────────────
fig3, axes = plt.subplots(1, 3, figsize=(16, 5), constrained_layout=True)
fig3.suptitle('Ganho de AoI C2 relativo ao Round-Robin (mean_h1) — 2r_variavel', fontsize=13)

compare_pols = ['pf-classic-sw', 'max-weight-sw', 'aoi-sw']
for ci, pol in enumerate(compare_pols):
    ax = axes[ci]
    mat = np.zeros((len(K_VALUES), len(C_VALUES)))
    for ki, K in enumerate(K_VALUES):
        for cj, C in enumerate(C_VALUES):
            rr = df[(df['K']==K) & (df['capacity']==C) & (df['policy']=='round-robin-sw')]['mean_h1']
            pp = df[(df['K']==K) & (df['capacity']==C) & (df['policy']==pol)]['mean_h1']
            if len(rr) and len(pp):
                mat[ki, cj] = (rr.values[0] - pp.values[0]) / rr.values[0] * 100

    im = ax.imshow(mat, cmap='RdYlGn', aspect='auto', vmin=-20, vmax=20)
    ax.set_xticks(range(len(C_VALUES))); ax.set_xticklabels(C_VALUES)
    ax.set_yticks(range(len(K_VALUES))); ax.set_yticklabels(K_VALUES)
    ax.set_xlabel('Capacidade C'); ax.set_ylabel('K (slots/época)')
    ax.set_title(f'{POL_LABEL[pol]} vs RR', fontweight='bold')
    for ki in range(len(K_VALUES)):
        for cj in range(len(C_VALUES)):
            ax.text(cj, ki, f'{mat[ki,cj]:+.1f}%', ha='center', va='center',
                    fontsize=7.5, color='black')
    plt.colorbar(im, ax=ax, label='Redução de h₁ (%)\n(+ = melhor que RR)')

fig3.savefig('sweep_heatmap_gain.png', dpi=150, bbox_inches='tight')
print('Saved: sweep_heatmap_gain.png')

# ─────────────────────────────────────────────────────────────────────────────
# Fig 4 — rho_emp vs C (carga normalizada do sistema)
# ─────────────────────────────────────────────────────────────────────────────
fig4, axes = plt.subplots(2, 3, figsize=(16, 9), constrained_layout=True)
fig4.suptitle('Carga empírica ρ vs Capacidade — cenário 2r_variavel', fontsize=13)
axes = axes.flatten()

for ki, K in enumerate(K_VALUES):
    ax = axes[ki]
    sub = df[df['K'] == K]
    for pol in POLICIES:
        d = sub[sub['policy'] == pol].sort_values('capacity')
        ax.plot(d['capacity'], d['rho_emp'], marker=MARKERS[pol],
                color=COLORS[pol], label=POL_LABEL[pol], lw=2, ms=7)
    ax.axhline(1.0, color='red', ls='--', lw=1, label='ρ=1 (limite)')
    ax.set_title(f'K = {K}', fontweight='bold')
    ax.set_xlabel('Capacidade C')
    ax.set_ylabel('ρ empírico')
    ax.set_xticks(C_VALUES)
    ax.grid(True, ls='--', alpha=0.4)
    if ki == 0:
        ax.legend(fontsize=8)

fig4.savefig('sweep_rho_vs_C.png', dpi=150, bbox_inches='tight')
print('Saved: sweep_rho_vs_C.png')

# ─────────────────────────────────────────────────────────────────────────────
# Fig 5 — mean_h1 vs K, fixando C representativos (4, 16, 48)
# ─────────────────────────────────────────────────────────────────────────────
C_sel = [4, 16, 48]
fig5, axes = plt.subplots(1, 3, figsize=(16, 5), constrained_layout=True)
fig5.suptitle('AoI médio C2 vs K (slots/época) — capacidades representativas', fontsize=13)

for ci, C in enumerate(C_sel):
    ax = axes[ci]
    sub = df[df['capacity'] == C]
    for pol in POLICIES:
        d = sub[sub['policy'] == pol].sort_values('K')
        ax.plot(d['K'], d['mean_h1'], marker=MARKERS[pol],
                color=COLORS[pol], label=POL_LABEL[pol], lw=2, ms=7)
    ax.set_title(f'C = {C}', fontweight='bold')
    ax.set_xlabel('K (slots por época)')
    ax.set_ylabel('mean h₁ (slots)')
    ax.set_xticks(K_VALUES)
    ax.grid(True, ls='--', alpha=0.4)
    ax.legend(fontsize=8)

fig5.savefig('sweep_h1_vs_K.png', dpi=150, bbox_inches='tight')
print('Saved: sweep_h1_vs_K.png')

plt.close('all')
print('\nTodos os plots salvos.')
