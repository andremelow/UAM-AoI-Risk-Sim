"""
plot_risk_2t1.py
Plots de risco para o experimento 2/3 RouteA + comparação com 50/50.
Figs:
  1. mean_r_sys vs C (por K)
  2. peak_r_sys vs C (por K)
  3. heatmap ganho de risco vs RR
  4. componentes r_unc / r_map / r_vid  (stacked bar + frações)
  5. comparação direta 50/50 vs 2/3 para mean_r_sys e peak_r_sys
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import re
from pathlib import Path

CSV_2T1  = Path('csv_export_debug/sweep_summary_2t1.csv')
CSV_5050 = Path('csv_export_debug/sweep_summary_50_50.csv')
SWEEP_DIR = Path('csv_export_debug/sw_sweep')
REF_MTIME = CSV_2T1.stat().st_mtime   # arquivos mais novos que o CSV 50/50 mas não mais novos que o 2t1

df     = pd.read_csv(CSV_2T1)
df5050 = pd.read_csv(CSV_5050)

POLICIES  = ['round-robin-sw', 'pf-classic-sw', 'max-weight-sw', 'aoi-sw']
POL_LABEL = {'round-robin-sw': 'Round-Robin', 'pf-classic-sw': 'PF-Classic',
             'max-weight-sw': 'Max-Weight',  'aoi-sw': 'AoI-SW'}
COLORS    = {'round-robin-sw': '#9E9E9E', 'pf-classic-sw': '#2196F3',
             'max-weight-sw': '#4CAF50',  'aoi-sw': '#FF5722'}
MARKERS   = {'round-robin-sw': 'o', 'pf-classic-sw': 's',
             'max-weight-sw': '^',  'aoi-sw': 'D'}

K_VALUES = sorted(df['K'].unique())
C_VALUES = sorted(df['capacity'].unique())

def fmt_ax(ax, k):
    ax.set_title(f'K = {k}', fontweight='bold')
    ax.set_xlabel('Capacidade C')
    ax.set_xticks(C_VALUES)
    ax.yaxis.set_major_formatter(ticker.ScalarFormatter(useMathText=True))
    ax.ticklabel_format(axis='y', style='sci', scilimits=(0,0))
    ax.grid(True, ls='--', alpha=0.4)

# ── Fig 1: mean_r_sys vs C ────────────────────────────────────────────────────
fig1, axes = plt.subplots(2, 3, figsize=(16, 9), constrained_layout=True)
fig1.suptitle('Risco médio do sistema vs Capacidade — 2r_variavel  (2/3 RouteA)', fontsize=13)
for ki, K in enumerate(K_VALUES):
    ax = axes.flatten()[ki]
    sub = df[df['K'] == K]
    for pol in POLICIES:
        d = sub[sub['policy'] == pol].sort_values('capacity')
        ax.plot(d['capacity'], d['mean_r_sys'], marker=MARKERS[pol],
                color=COLORS[pol], label=POL_LABEL[pol], lw=2, ms=7)
    ax.set_ylabel('mean R_sys')
    fmt_ax(ax, K)
    if ki == 0: ax.legend(fontsize=8)
fig1.savefig('2t1_risk_mean.png', dpi=150, bbox_inches='tight')
print('Saved: 2t1_risk_mean.png')

# ── Fig 2: peak_r_sys vs C ────────────────────────────────────────────────────
fig2, axes = plt.subplots(2, 3, figsize=(16, 9), constrained_layout=True)
fig2.suptitle('Pico de risco vs Capacidade — 2r_variavel  (2/3 RouteA)', fontsize=13)
for ki, K in enumerate(K_VALUES):
    ax = axes.flatten()[ki]
    sub = df[df['K'] == K]
    for pol in POLICIES:
        d = sub[sub['policy'] == pol].sort_values('capacity')
        ax.plot(d['capacity'], d['peak_r_sys'], marker=MARKERS[pol],
                color=COLORS[pol], label=POL_LABEL[pol], lw=2, ms=7)
    ax.set_ylabel('peak R_sys')
    fmt_ax(ax, K)
    if ki == 0: ax.legend(fontsize=8)
fig2.savefig('2t1_risk_peak.png', dpi=150, bbox_inches='tight')
print('Saved: 2t1_risk_peak.png')

# ── Fig 3: heatmap ganho vs RR ────────────────────────────────────────────────
fig3, axes = plt.subplots(1, 3, figsize=(16, 5), constrained_layout=True)
fig3.suptitle('Redução de risco médio vs Round-Robin — 2/3 RouteA', fontsize=13)
for ci, pol in enumerate(['pf-classic-sw', 'max-weight-sw', 'aoi-sw']):
    ax = axes[ci]
    mat = np.zeros((len(K_VALUES), len(C_VALUES)))
    for ki, K in enumerate(K_VALUES):
        for cj, C in enumerate(C_VALUES):
            rr = df[(df['K']==K)&(df['capacity']==C)&(df['policy']=='round-robin-sw')]['mean_r_sys']
            pp = df[(df['K']==K)&(df['capacity']==C)&(df['policy']==pol)]['mean_r_sys']
            if len(rr) and len(pp) and rr.values[0] > 0:
                mat[ki, cj] = (rr.values[0] - pp.values[0]) / rr.values[0] * 100
    im = ax.imshow(mat, cmap='RdYlGn', aspect='auto', vmin=-20, vmax=20)
    ax.set_xticks(range(len(C_VALUES))); ax.set_xticklabels(C_VALUES)
    ax.set_yticks(range(len(K_VALUES))); ax.set_yticklabels(K_VALUES)
    ax.set_xlabel('Capacidade C'); ax.set_ylabel('K')
    ax.set_title(f'{POL_LABEL[pol]} vs RR', fontweight='bold')
    for ki in range(len(K_VALUES)):
        for cj in range(len(C_VALUES)):
            ax.text(cj, ki, f'{mat[ki,cj]:+.1f}%', ha='center', va='center', fontsize=7.5)
    plt.colorbar(im, ax=ax, label='Redução (%)\n(+ = melhor que RR)')
fig3.savefig('2t1_risk_heatmap.png', dpi=150, bbox_inches='tight')
print('Saved: 2t1_risk_heatmap.png')

# ── Fig 4: componentes de risco — slot_series do run 2t1 ─────────────────────
ref5050_mtime = CSV_5050.stat().st_mtime
records = []
for f in SWEEP_DIR.rglob('*slot_series*.csv'):
    mt = f.stat().st_mtime
    if mt <= ref5050_mtime or mt > REF_MTIME:
        continue
    m_k   = re.search(r'/K(\d+)/', str(f))
    m_c   = re.search(r'/C(\d+)/', str(f))
    m_pol = re.search(r'(round-robin-sw|pf-classic-sw|max-weight-sw|aoi-sw)_', f.name)
    if not (m_k and m_c and m_pol): continue
    try:
        tmp = pd.read_csv(f, usecols=['r_unc','r_map','r_vid','r_tot'])
        tmp['K'] = int(m_k.group(1)); tmp['C'] = int(m_c.group(1))
        tmp['policy'] = m_pol.group(1)
        records.append(tmp)
    except Exception:
        pass

if records:
    agg = (pd.concat(records, ignore_index=True)
             .groupby(['K','C','policy'])[['r_unc','r_map','r_vid','r_tot']]
             .mean().reset_index())
    print(f'Componentes: {len(agg)} grupos')

    COMP = [('r_unc','Incerteza','#FF9800'), ('r_map','Mapa/Área','#9C27B0'), ('r_vid','Vídeo','#F44336')]

    # stacked bar por política
    fig4, axes = plt.subplots(2, 3, figsize=(16, 9), constrained_layout=True)
    fig4.suptitle('Composição do risco — 2/3 RouteA  (média sobre todos os C)', fontsize=13)
    for ki, K in enumerate(K_VALUES):
        ax  = axes.flatten()[ki]
        sub = agg[agg['K']==K].groupby('policy')[['r_unc','r_map','r_vid']].mean().reindex(POLICIES)
        x   = np.arange(len(POLICIES)); bot = np.zeros(len(POLICIES))
        for col, lbl, clr in COMP:
            ax.bar(x, sub[col].values, bottom=bot, color=clr, label=lbl, width=0.6)
            bot += sub[col].values
        ax.set_title(f'K = {K}', fontweight='bold')
        ax.set_xticks(x)
        ax.set_xticklabels([POL_LABEL[p] for p in POLICIES], rotation=20, ha='right', fontsize=8)
        ax.set_ylabel('Risco médio')
        ax.yaxis.set_major_formatter(ticker.ScalarFormatter(useMathText=True))
        ax.ticklabel_format(axis='y', style='sci', scilimits=(0,0))
        ax.grid(True, axis='y', ls='--', alpha=0.4)
        if ki == 0: ax.legend(fontsize=8)
    fig4.savefig('2t1_risk_components.png', dpi=150, bbox_inches='tight')
    print('Saved: 2t1_risk_components.png')

    # frações
    fig5, axes = plt.subplots(2, 3, figsize=(16, 9), constrained_layout=True)
    fig5.suptitle('Fração de cada componente no risco total — 2/3 RouteA', fontsize=13)
    for ki, K in enumerate(K_VALUES):
        ax  = axes.flatten()[ki]
        sub = agg[agg['K']==K].groupby('policy')[['r_unc','r_map','r_vid','r_tot']].mean().reindex(POLICIES)
        x   = np.arange(len(POLICIES)); bot = np.zeros(len(POLICIES))
        for col, lbl, clr in COMP:
            frac = (sub[col] / sub['r_tot'].replace(0, np.nan)).fillna(0).values * 100
            ax.bar(x, frac, bottom=bot, color=clr, label=lbl, width=0.6)
            bot += frac
        ax.set_title(f'K = {K}', fontweight='bold')
        ax.set_xticks(x)
        ax.set_xticklabels([POL_LABEL[p] for p in POLICIES], rotation=20, ha='right', fontsize=8)
        ax.set_ylabel('Fração do risco total (%)')
        ax.set_ylim(0, 100)
        ax.grid(True, axis='y', ls='--', alpha=0.4)
        if ki == 0: ax.legend(fontsize=8)
    fig5.savefig('2t1_risk_fractions.png', dpi=150, bbox_inches='tight')
    print('Saved: 2t1_risk_fractions.png')
else:
    print('Aviso: nenhum slot_series do run 2t1 encontrado para componentes.')

# ── Fig 6: comparação 50/50 vs 2/3 — mean_r_sys e peak_r_sys ────────────────
# Fixa K representativos: 1, 4, 16
K_sel = [1, 4, 16]
fig6, axes = plt.subplots(2, 3, figsize=(16, 10), constrained_layout=True)
fig6.suptitle('Comparação 50/50 vs 2/3 RouteA — Risco médio e Pico (Max-Weight)', fontsize=13)

for col_i, metric in enumerate(['mean_r_sys', 'peak_r_sys']):
    ylabel = 'Risco médio R_sys' if metric == 'mean_r_sys' else 'Pico de risco R_peak'
    for ki, K in enumerate(K_sel):
        ax = axes[col_i][ki]
        for label, dframe, ls, alpha in [('50/50', df5050, '-', 1.0), ('2/3 RouteA', df, '--', 0.85)]:
            sub = dframe[dframe['K'] == K]
            for pol in POLICIES:
                d = sub[sub['policy'] == pol].sort_values('capacity')
                lw = 2.5 if label == '2/3 RouteA' else 1.5
                mk = MARKERS[pol] if label == '2/3 RouteA' else ''
                ax.plot(d['capacity'], d[metric], ls=ls, marker=mk,
                        color=COLORS[pol], lw=lw, alpha=alpha,
                        label=f'{POL_LABEL[pol]} ({label})' if col_i==0 and ki==0 else '_')
        ax.set_title(f'K={K}', fontweight='bold')
        ax.set_xlabel('Capacidade C')
        ax.set_ylabel(ylabel)
        ax.set_xticks(C_VALUES)
        ax.yaxis.set_major_formatter(ticker.ScalarFormatter(useMathText=True))
        ax.ticklabel_format(axis='y', style='sci', scilimits=(0,0))
        ax.grid(True, ls='--', alpha=0.4)

axes[0][0].legend(fontsize=7, ncol=2)
# legenda de estilo
from matplotlib.lines import Line2D
style_leg = [Line2D([0],[0], color='k', lw=1.5, ls='-',  label='50/50'),
             Line2D([0],[0], color='k', lw=2.5, ls='--', label='2/3 RouteA')]
axes[1][2].legend(handles=style_leg, fontsize=9, loc='upper right')

fig6.savefig('2t1_risk_compare.png', dpi=150, bbox_inches='tight')
print('Saved: 2t1_risk_compare.png')

plt.close('all')
print('\nTodos os plots salvos.')
