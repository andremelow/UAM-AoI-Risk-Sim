"""
plot_risk_components.py
Decompõe o risco em r_unc, r_map, r_vid por política/C/K
Usa os slot_series CSVs do experimento anterior (50/50 split).
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import os, re
from pathlib import Path

SWEEP_DIR   = Path('csv_export_debug/sw_sweep')
SUMMARY_REF = Path('csv_export_debug/sweep_summary_50_50.csv')
ref_mtime   = SUMMARY_REF.stat().st_mtime

POLICIES  = ['round-robin-sw', 'pf-classic-sw', 'max-weight-sw', 'aoi-sw']
POL_LABEL = {'round-robin-sw': 'Round-Robin', 'pf-classic-sw': 'PF-Classic',
             'max-weight-sw': 'Max-Weight',  'aoi-sw': 'AoI-SW'}
COLORS    = {'round-robin-sw': '#9E9E9E', 'pf-classic-sw': '#2196F3',
             'max-weight-sw': '#4CAF50',  'aoi-sw': '#FF5722'}
MARKERS   = {'round-robin-sw': 'o', 'pf-classic-sw': 's',
             'max-weight-sw': '^',  'aoi-sw': 'D'}

# ── Coleta todos os slot_series do run anterior ───────────────────────────────
records = []
for f in SWEEP_DIR.rglob('*slot_series*.csv'):
    if f.stat().st_mtime > ref_mtime:
        continue   # ignora run atual
    # extrai K, C, policy do caminho e nome
    m_k   = re.search(r'/K(\d+)/', str(f))
    m_c   = re.search(r'/C(\d+)/', str(f))
    m_pol = re.search(r'(round-robin-sw|pf-classic-sw|max-weight-sw|aoi-sw)_', f.name)
    if not (m_k and m_c and m_pol):
        continue
    K   = int(m_k.group(1))
    C   = int(m_c.group(1))
    pol = m_pol.group(1)
    try:
        tmp = pd.read_csv(f, usecols=['r_unc', 'r_map', 'r_vid', 'r_tot'])
        tmp['K'] = K; tmp['C'] = C; tmp['policy'] = pol
        records.append(tmp)
    except Exception:
        pass

df_all = pd.concat(records, ignore_index=True)

# Para cada (K, C, policy) mantém apenas um conjunto de dados:
# usa a média por grupo (duplicatas de runs abortados têm mesmo comportamento)
agg = (df_all.groupby(['K', 'C', 'policy'])[['r_unc', 'r_map', 'r_vid', 'r_tot']]
             .mean().reset_index())

print(f'Grupos únicos: {len(agg)}  (esperado 168)')

K_VALUES = sorted(agg['K'].unique())
C_VALUES = sorted(agg['C'].unique())

COMPONENTS = [('r_unc', 'Incerteza (r_unc)', '#FF9800'),
              ('r_map', 'Mapa/Área (r_map)',  '#9C27B0'),
              ('r_vid', 'Vídeo (r_vid)',       '#F44336')]

# ── Fig 1: stacked bar — composição do risco por política (médias globais) ───
fig1, axes = plt.subplots(1, len(K_VALUES), figsize=(18, 5), constrained_layout=True)
fig1.suptitle('Composição do risco médio por política — média sobre todos os C', fontsize=13)

for ki, K in enumerate(K_VALUES):
    ax  = axes[ki]
    sub = agg[agg['K'] == K].groupby('policy')[['r_unc','r_map','r_vid']].mean()
    sub = sub.reindex(POLICIES)
    x   = np.arange(len(POLICIES))
    bottom = np.zeros(len(POLICIES))
    for col, lbl, color in COMPONENTS:
        vals = sub[col].values
        ax.bar(x, vals, bottom=bottom, color=color, label=lbl, width=0.6)
        bottom += vals
    ax.set_title(f'K = {K}', fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels([POL_LABEL[p] for p in POLICIES], rotation=20, ha='right', fontsize=8)
    ax.set_ylabel('Risco médio')
    ax.yaxis.set_major_formatter(ticker.ScalarFormatter(useMathText=True))
    ax.ticklabel_format(axis='y', style='sci', scilimits=(0,0))
    ax.grid(True, axis='y', ls='--', alpha=0.4)
    if ki == 0:
        ax.legend(fontsize=8)

fig1.savefig('risk_components_stacked.png', dpi=150, bbox_inches='tight')
print('Saved: risk_components_stacked.png')

# ── Fig 2: cada componente vs C, um painel por K ─────────────────────────────
for col, lbl, color_comp in COMPONENTS:
    fname = f'risk_comp_{col}_vs_C.png'
    fig, axes = plt.subplots(2, 3, figsize=(16, 9), constrained_layout=True)
    fig.suptitle(f'{lbl} vs Capacidade — cenário 2r_variavel (50/50)', fontsize=13)
    axes = axes.flatten()
    for ki, K in enumerate(K_VALUES):
        ax  = axes[ki]
        sub = agg[agg['K'] == K]
        for pol in POLICIES:
            d = sub[sub['policy'] == pol].sort_values('C')
            ax.plot(d['C'], d[col], marker=MARKERS[pol],
                    color=COLORS[pol], label=POL_LABEL[pol], lw=2, ms=7)
        ax.set_title(f'K = {K}', fontweight='bold')
        ax.set_xlabel('Capacidade C')
        ax.set_ylabel(lbl)
        ax.set_xticks(C_VALUES)
        ax.yaxis.set_major_formatter(ticker.ScalarFormatter(useMathText=True))
        ax.ticklabel_format(axis='y', style='sci', scilimits=(0,0))
        ax.grid(True, ls='--', alpha=0.4)
        if ki == 0:
            ax.legend(fontsize=8)
    fig.savefig(fname, dpi=150, bbox_inches='tight')
    print(f'Saved: {fname}')
    plt.close(fig)

# ── Fig 3: frações do risco total por política e K ───────────────────────────
fig3, axes = plt.subplots(2, 3, figsize=(16, 9), constrained_layout=True)
fig3.suptitle('Fração de cada componente no risco total — média sobre todos os C', fontsize=13)
axes = axes.flatten()

for ki, K in enumerate(K_VALUES):
    ax  = axes[ki]
    sub = agg[agg['K'] == K].groupby('policy')[['r_unc','r_map','r_vid','r_tot']].mean()
    sub = sub.reindex(POLICIES)
    x   = np.arange(len(POLICIES))
    bottom = np.zeros(len(POLICIES))
    for col, lbl, color in COMPONENTS:
        frac = (sub[col] / sub['r_tot'].replace(0, np.nan)).fillna(0).values
        ax.bar(x, frac * 100, bottom=bottom, color=color, label=lbl, width=0.6)
        bottom += frac * 100
    ax.set_title(f'K = {K}', fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels([POL_LABEL[p] for p in POLICIES], rotation=20, ha='right', fontsize=8)
    ax.set_ylabel('Fração do risco total (%)')
    ax.set_ylim(0, 100)
    ax.grid(True, axis='y', ls='--', alpha=0.4)
    if ki == 0:
        ax.legend(fontsize=8)

fig3.savefig('risk_components_fraction.png', dpi=150, bbox_inches='tight')
print('Saved: risk_components_fraction.png')

plt.close('all')
print('\nTodos os plots de componentes salvos.')
