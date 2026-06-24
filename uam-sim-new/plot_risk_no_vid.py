"""
plot_risk_no_vid.py
Compara risco médio com e sem componente R_vid — experimento 2t1 (2/3 RouteA).

r_with_vid = r_tot           = w_unc*r_unc + w_map*r_map + w_vid*r_vid
r_no_vid   = r_tot - r_vid/3 = w_unc*r_unc + w_map*r_map
(pesos = 1/3 cada)
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import re
from pathlib import Path

SWEEP_DIR  = Path('csv_export_debug/sw_sweep')
CSV_5050   = Path('csv_export_debug/sweep_summary_50_50.csv')
CSV_2T1    = Path('csv_export_debug/sweep_summary_2t1.csv')
W_VID      = 1/3

ref_lo = CSV_5050.stat().st_mtime   # depois deste = run 2t1
ref_hi = CSV_2T1.stat().st_mtime    # antes deste  = run 2t1

POLICIES  = ['round-robin-sw', 'pf-classic-sw', 'max-weight-sw', 'aoi-sw']
POL_LABEL = {'round-robin-sw': 'Round-Robin', 'pf-classic-sw': 'PF-Classic',
             'max-weight-sw': 'Max-Weight',  'aoi-sw': 'AoI-SW'}
COLORS    = {'round-robin-sw': '#9E9E9E', 'pf-classic-sw': '#2196F3',
             'max-weight-sw': '#4CAF50',  'aoi-sw': '#FF5722'}
MARKERS   = {'round-robin-sw': 'o', 'pf-classic-sw': 's',
             'max-weight-sw': '^',  'aoi-sw': 'D'}

# ── Lê slot_series do run 2t1 ─────────────────────────────────────────────────
records = []
for f in SWEEP_DIR.rglob('*slot_series*.csv'):
    mt = f.stat().st_mtime
    if not (ref_lo < mt <= ref_hi):
        continue
    m_k   = re.search(r'/K(\d+)/', str(f))
    m_c   = re.search(r'/C(\d+)/', str(f))
    m_pol = re.search(r'(round-robin-sw|pf-classic-sw|max-weight-sw|aoi-sw)_', f.name)
    if not (m_k and m_c and m_pol): continue
    try:
        tmp = pd.read_csv(f, usecols=['r_vid', 'r_tot'])
        tmp['K']      = int(m_k.group(1))
        tmp['C']      = int(m_c.group(1))
        tmp['policy'] = m_pol.group(1)
        records.append(tmp)
    except Exception:
        pass

df_all = pd.concat(records, ignore_index=True)
df_all['r_no_vid'] = df_all['r_tot'] - W_VID * df_all['r_vid']

agg = (df_all.groupby(['K', 'C', 'policy'])[['r_tot', 'r_no_vid']]
             .mean().reset_index())
print(f'Grupos: {len(agg)}  (esperado 168)')

K_VALUES = sorted(agg['K'].unique())
C_VALUES = sorted(agg['C'].unique())

def fmt_ax(ax, K):
    ax.set_title(f'K = {K}', fontweight='bold')
    ax.set_xlabel('Capacidade C')
    ax.set_xticks(C_VALUES)
    ax.yaxis.set_major_formatter(ticker.ScalarFormatter(useMathText=True))
    ax.ticklabel_format(axis='y', style='sci', scilimits=(0, 0))
    ax.grid(True, ls='--', alpha=0.4)

# ── Fig 1: com vs sem R_vid  — sobreposição linha sólida/tracejada ────────────
fig1, axes = plt.subplots(2, 3, figsize=(16, 9), constrained_layout=True)
fig1.suptitle('Risco médio: com R_vid (sólido) vs sem R_vid (tracejado) — 2/3 RouteA', fontsize=13)
for ki, K in enumerate(K_VALUES):
    ax  = axes.flatten()[ki]
    sub = agg[agg['K'] == K]
    for pol in POLICIES:
        d = sub[sub['policy'] == pol].sort_values('C')
        ax.plot(d['C'], d['r_tot'],    ls='-',  marker=MARKERS[pol],
                color=COLORS[pol], lw=2,   ms=7, label=f'{POL_LABEL[pol]}')
        ax.plot(d['C'], d['r_no_vid'], ls='--', marker=MARKERS[pol],
                color=COLORS[pol], lw=1.5, ms=5, alpha=0.7, label='_')
    ax.set_ylabel('Risco médio')
    fmt_ax(ax, K)
    if ki == 0:
        ax.legend(fontsize=8)

from matplotlib.lines import Line2D
style_leg = [Line2D([0],[0], color='k', lw=2,   ls='-',  label='com R_vid'),
             Line2D([0],[0], color='k', lw=1.5, ls='--', label='sem R_vid')]
axes.flatten()[-1].legend(handles=style_leg, fontsize=9, loc='upper right')
fig1.savefig('risk_novid_overlay.png', dpi=150, bbox_inches='tight')
print('Saved: risk_novid_overlay.png')

# ── Fig 2: delta = r_tot - r_no_vid  (contribuição absoluta de R_vid) ─────────
fig2, axes = plt.subplots(2, 3, figsize=(16, 9), constrained_layout=True)
fig2.suptitle('Contribuição absoluta de R_vid ao risco total — 2/3 RouteA', fontsize=13)
for ki, K in enumerate(K_VALUES):
    ax  = axes.flatten()[ki]
    sub = agg[agg['K'] == K]
    for pol in POLICIES:
        d = sub[sub['policy'] == pol].sort_values('C')
        delta = d['r_tot'] - d['r_no_vid']
        ax.plot(d['C'], delta, marker=MARKERS[pol],
                color=COLORS[pol], lw=2, ms=7, label=POL_LABEL[pol])
    ax.set_ylabel('ΔR  (= w_vid · R̄_vid)')
    fmt_ax(ax, K)
    if ki == 0: ax.legend(fontsize=8)
fig2.savefig('risk_novid_delta.png', dpi=150, bbox_inches='tight')
print('Saved: risk_novid_delta.png')

# ── Fig 3: fração de R_vid no total, por política e K ─────────────────────────
fig3, axes = plt.subplots(2, 3, figsize=(16, 9), constrained_layout=True)
fig3.suptitle('Fração de R_vid no risco total (%) — 2/3 RouteA', fontsize=13)
for ki, K in enumerate(K_VALUES):
    ax  = axes.flatten()[ki]
    sub = agg[agg['K'] == K]
    for pol in POLICIES:
        d     = sub[sub['policy'] == pol].sort_values('C')
        frac  = (d['r_tot'] - d['r_no_vid']) / d['r_tot'].replace(0, np.nan) * 100
        ax.plot(d['C'], frac, marker=MARKERS[pol],
                color=COLORS[pol], lw=2, ms=7, label=POL_LABEL[pol])
    ax.set_ylabel('Fração R_vid (%)')
    ax.set_ylim(0, 100)
    fmt_ax(ax, K)
    if ki == 0: ax.legend(fontsize=8)
fig3.savefig('risk_novid_fraction.png', dpi=150, bbox_inches='tight')
print('Saved: risk_novid_fraction.png')

# ── Fig 4: heatmap — fração de R_vid por (K, C) para cada política ────────────
fig4, axes = plt.subplots(1, 4, figsize=(20, 5), constrained_layout=True)
fig4.suptitle('Fração de R_vid no risco total (%) por K × C', fontsize=13)
for pi, pol in enumerate(POLICIES):
    ax  = axes[pi]
    mat = np.full((len(K_VALUES), len(C_VALUES)), np.nan)
    for ki, K in enumerate(K_VALUES):
        for ci, C in enumerate(C_VALUES):
            row = agg[(agg['K']==K) & (agg['C']==C) & (agg['policy']==pol)]
            if len(row) and row['r_tot'].values[0] > 0:
                mat[ki, ci] = (row['r_tot'].values[0] - row['r_no_vid'].values[0]) \
                              / row['r_tot'].values[0] * 100
    im = ax.imshow(mat, cmap='YlOrRd', aspect='auto', vmin=0, vmax=100)
    ax.set_xticks(range(len(C_VALUES))); ax.set_xticklabels(C_VALUES, fontsize=8)
    ax.set_yticks(range(len(K_VALUES))); ax.set_yticklabels(K_VALUES)
    ax.set_xlabel('C'); ax.set_ylabel('K')
    ax.set_title(POL_LABEL[pol], fontweight='bold')
    for ki in range(len(K_VALUES)):
        for ci in range(len(C_VALUES)):
            if not np.isnan(mat[ki, ci]):
                ax.text(ci, ki, f'{mat[ki,ci]:.0f}%', ha='center', va='center', fontsize=7)
    plt.colorbar(im, ax=ax, label='% R_vid')
fig4.savefig('risk_novid_heatmap.png', dpi=150, bbox_inches='tight')
print('Saved: risk_novid_heatmap.png')

plt.close('all')
print('\nTodos os plots salvos.')
