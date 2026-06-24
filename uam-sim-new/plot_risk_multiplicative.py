"""
plot_risk_multiplicative.py
Nova equação de risco multiplicativa — experimento 2t1 (2/3 RouteA):

    R_u(t) = (w_unc * R~_unc + w_map * R~_map) * (1 + κ * R~_vid)

    w_unc + w_map = 1   (default: 0.5 / 0.5)
    R~_unc = r_unc / max_global(r_unc)       ∈ [0,1]
    R~_map = r_map / max_global(r_map)       ∈ [0,1]
    R~_vid = clip(r_vid, 0, 1)              ∈ [0,1]
                          (r_vid = max(z_u/H_nom - 1, 0); 1 ≡ z_u = 2·H_nom)

Plota κ ∈ {0, 0.5, 1, 2} para mostrar sensibilidade ao acoplamento.
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import re
from pathlib import Path

SWEEP_DIR = Path('csv_export_debug/sw_sweep')
CSV_5050  = Path('csv_export_debug/sweep_summary_50_50.csv')
CSV_2T1   = Path('csv_export_debug/sweep_summary_2t1.csv')

ref_lo = CSV_5050.stat().st_mtime
ref_hi = CSV_2T1.stat().st_mtime

W_UNC = 0.5
W_MAP = 0.5
KAPPAS = [0, 0.5, 1.0, 2.0]

POLICIES  = ['round-robin-sw', 'pf-classic-sw', 'max-weight-sw', 'aoi-sw']
POL_LABEL = {'round-robin-sw': 'Round-Robin', 'pf-classic-sw': 'PF-Classic',
             'max-weight-sw': 'Max-Weight',  'aoi-sw': 'AoI-SW'}
COLORS    = {'round-robin-sw': '#9E9E9E', 'pf-classic-sw': '#2196F3',
             'max-weight-sw': '#4CAF50',  'aoi-sw': '#FF5722'}
MARKERS   = {'round-robin-sw': 'o', 'pf-classic-sw': 's',
             'max-weight-sw': '^',  'aoi-sw': 'D'}
K_ALPHA   = {0: 1.0, 0.5: 0.85, 1.0: 0.65, 2.0: 0.45}
K_LS      = {0: '-', 0.5: '--', 1.0: '-.', 2.0: ':'}

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
        tmp = pd.read_csv(f, usecols=['r_unc', 'r_map', 'r_vid'])
        tmp['K']      = int(m_k.group(1))
        tmp['C']      = int(m_c.group(1))
        tmp['policy'] = m_pol.group(1)
        records.append(tmp)
    except Exception:
        pass

df_all = pd.concat(records, ignore_index=True)
print(f'Slots carregados: {len(df_all):,}')

# ── Normalização global ────────────────────────────────────────────────────────
max_unc = df_all['r_unc'].max()
max_map = df_all['r_map'].max()
print(f'max r_unc={max_unc:.4e}  max r_map={max_map:.4e}  max r_vid={df_all["r_vid"].max():.4f}')

df_all['r_unc_n'] = df_all['r_unc'] / max_unc if max_unc > 0 else 0
df_all['r_map_n'] = df_all['r_map'] / max_map if max_map > 0 else 0
df_all['r_vid_n'] = df_all['r_vid'].clip(0, 1)

# ── Calcula R_new para cada κ e agrega ────────────────────────────────────────
K_VALUES = sorted(df_all['K'].unique())
C_VALUES = sorted(df_all['C'].unique())

agg_dict = {}
for kappa in KAPPAS:
    df_all[f'R_new_{kappa}'] = (
        (W_UNC * df_all['r_unc_n'] + W_MAP * df_all['r_map_n']) *
        (1 + kappa * df_all['r_vid_n'])
    )
    agg_dict[kappa] = (df_all.groupby(['K', 'C', 'policy'])[f'R_new_{kappa}']
                              .mean().reset_index()
                              .rename(columns={f'R_new_{kappa}': 'R_new'}))

print(f'Grupos por κ: {len(agg_dict[1.0])}')

# ── Fig 1: R_new vs C por K — um painel por K, curvas por política (κ=1) ─────
kappa_main = 1.0
agg = agg_dict[kappa_main]

fig1, axes = plt.subplots(2, 3, figsize=(16, 9), constrained_layout=True)
fig1.suptitle(
    r'Risco multiplicativo $\bar{R}_u = (\tilde{R}_{unc}+\tilde{R}_{map})/2 \cdot (1+\kappa\tilde{R}_{vid})$'
    f'  —  κ={kappa_main}  —  2/3 RouteA', fontsize=12)

for ki, K in enumerate(K_VALUES):
    ax  = axes.flatten()[ki]
    sub = agg[agg['K'] == K]
    for pol in POLICIES:
        d = sub[sub['policy'] == pol].sort_values('C')
        ax.plot(d['C'], d['R_new'], marker=MARKERS[pol],
                color=COLORS[pol], lw=2, ms=7, label=POL_LABEL[pol])
    ax.set_title(f'K = {K}', fontweight='bold')
    ax.set_xlabel('Capacidade C')
    ax.set_ylabel(r'$\bar{R}_u$  (normalizado)')
    ax.set_xticks(C_VALUES)
    ax.grid(True, ls='--', alpha=0.4)
    if ki == 0: ax.legend(fontsize=8)

fig1.savefig('risk_mult_vs_C.png', dpi=150, bbox_inches='tight')
print('Saved: risk_mult_vs_C.png')

# ── Fig 2: sensibilidade a κ — fixa K={1,4,16}, mostra todas as políticas ────
K_sel = [1, 4, 16]
fig2, axes = plt.subplots(len(K_sel), len(KAPPAS), figsize=(18, 12), constrained_layout=True)
fig2.suptitle(
    r'Sensibilidade ao acoplamento κ — $\bar{R}_u$ vs C  —  2/3 RouteA', fontsize=13)

for ki, K in enumerate(K_sel):
    for kai, kappa in enumerate(KAPPAS):
        ax  = axes[ki][kai]
        sub = agg_dict[kappa][agg_dict[kappa]['K'] == K]
        for pol in POLICIES:
            d = sub[sub['policy'] == pol].sort_values('C')
            ax.plot(d['C'], d['R_new'], marker=MARKERS[pol],
                    color=COLORS[pol], lw=2, ms=6,
                    label=POL_LABEL[pol] if (ki == 0 and kai == 0) else '_')
        ax.set_title(f'K={K},  κ={kappa}', fontweight='bold', fontsize=9)
        ax.set_xlabel('C', fontsize=8)
        ax.set_xticks(C_VALUES)
        ax.set_xticklabels(C_VALUES, fontsize=7)
        ax.grid(True, ls='--', alpha=0.4)
        ax.yaxis.set_major_formatter(ticker.FormatStrFormatter('%.3f'))

axes[0][0].legend(fontsize=8)
fig2.savefig('risk_mult_kappa_sensitivity.png', dpi=150, bbox_inches='tight')
print('Saved: risk_mult_kappa_sensitivity.png')

# ── Fig 3: heatmap — ganho vs RR para κ=1, por (K, C) ───────────────────────
agg1 = agg_dict[1.0]
fig3, axes = plt.subplots(1, 3, figsize=(16, 5), constrained_layout=True)
fig3.suptitle(r'Redução de $\bar{R}_u$ vs Round-Robin  (κ=1) — 2/3 RouteA', fontsize=13)

for ci, pol in enumerate(['pf-classic-sw', 'max-weight-sw', 'aoi-sw']):
    ax  = axes[ci]
    mat = np.zeros((len(K_VALUES), len(C_VALUES)))
    for ki, K in enumerate(K_VALUES):
        for cj, C in enumerate(C_VALUES):
            rr = agg1[(agg1['K']==K)&(agg1['C']==C)&(agg1['policy']=='round-robin-sw')]['R_new']
            pp = agg1[(agg1['K']==K)&(agg1['C']==C)&(agg1['policy']==pol)]['R_new']
            if len(rr) and len(pp) and rr.values[0] > 0:
                mat[ki, cj] = (rr.values[0] - pp.values[0]) / rr.values[0] * 100
    im = ax.imshow(mat, cmap='RdYlGn', aspect='auto', vmin=-20, vmax=20)
    ax.set_xticks(range(len(C_VALUES))); ax.set_xticklabels(C_VALUES)
    ax.set_yticks(range(len(K_VALUES))); ax.set_yticklabels(K_VALUES)
    ax.set_xlabel('C'); ax.set_ylabel('K')
    ax.set_title(f'{POL_LABEL[pol]} vs RR', fontweight='bold')
    for ki in range(len(K_VALUES)):
        for cj in range(len(C_VALUES)):
            ax.text(cj, ki, f'{mat[ki,cj]:+.1f}%', ha='center', va='center', fontsize=7.5)
    plt.colorbar(im, ax=ax, label='Redução (%)')
fig3.savefig('risk_mult_heatmap.png', dpi=150, bbox_inches='tight')
print('Saved: risk_mult_heatmap.png')

# ── Fig 4: comparação aditivo (w=1/3 cada) vs multiplicativo (κ=1) ────────────
csv_2t1 = pd.read_csv(CSV_2T1)
agg_add = (df_all.groupby(['K','C','policy'])[['r_unc','r_map','r_vid']]
                 .mean().reset_index())
agg_add['R_add']  = (1/3)*agg_add['r_unc'] + (1/3)*agg_add['r_map'] + (1/3)*agg_add['r_vid']
agg_add = agg_add.merge(agg_dict[1.0], on=['K','C','policy'])

K_sel2 = [1, 4, 16]
fig4, axes = plt.subplots(1, len(K_sel2), figsize=(16, 5), constrained_layout=True)
fig4.suptitle('Aditivo (w=1/3 cada) vs Multiplicativo (κ=1) — Max-Weight  —  2/3 RouteA', fontsize=12)

for ki, K in enumerate(K_sel2):
    ax  = axes[ki]
    sub = agg_add[agg_add['K'] == K]
    for pol in POLICIES:
        d = sub[sub['policy'] == pol].sort_values('C')
        ax.plot(d['C'], d['R_add'] / d['R_add'].max(), ls='-',  marker=MARKERS[pol],
                color=COLORS[pol], lw=2, ms=7,
                label=f'{POL_LABEL[pol]} add.' if ki == 0 else '_')
        ax.plot(d['C'], d['R_new'] / d['R_new'].max(), ls='--', marker=MARKERS[pol],
                color=COLORS[pol], lw=1.5, ms=5, alpha=0.75,
                label=f'{POL_LABEL[pol]} mult.' if ki == 0 else '_')
    ax.set_title(f'K = {K}', fontweight='bold')
    ax.set_xlabel('Capacidade C')
    ax.set_ylabel('Risco normalizado (max=1)')
    ax.set_xticks(C_VALUES)
    ax.grid(True, ls='--', alpha=0.4)

from matplotlib.lines import Line2D
style = [Line2D([0],[0], color='k', lw=2,   ls='-',  label='Aditivo'),
         Line2D([0],[0], color='k', lw=1.5, ls='--', label='Multiplicativo (κ=1)')]
axes[0].legend(fontsize=7, ncol=2)
axes[-1].legend(handles=style, fontsize=9)
fig4.savefig('risk_mult_vs_add.png', dpi=150, bbox_inches='tight')
print('Saved: risk_mult_vs_add.png')

plt.close('all')
print('\nTodos os plots salvos.')
