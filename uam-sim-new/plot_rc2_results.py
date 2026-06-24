"""
plot_rc2_results.py
Resultados do experimento rc2 (5 políticas: +reserve-c2-sw) — 2/3 RouteA.
Foco em: AoI C2, risco médio, pico de risco, heatmaps de ganho vs RR.
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from matplotlib.lines import Line2D
from pathlib import Path

CSV = Path('csv_export_debug/sweep_summary_rc2.csv')
df  = pd.read_csv(CSV)

POLICIES  = ['round-robin-sw', 'pf-classic-sw', 'max-weight-sw', 'aoi-sw', 'reserve-c2-sw']
POL_LABEL = {'round-robin-sw':  'Round-Robin',
             'pf-classic-sw':   'PF-Classic',
             'max-weight-sw':   'Max-Weight',
             'aoi-sw':          'AoI-SW',
             'reserve-c2-sw':   'Reserve-C2'}
COLORS    = {'round-robin-sw':  '#9E9E9E',
             'pf-classic-sw':   '#2196F3',
             'max-weight-sw':   '#4CAF50',
             'aoi-sw':          '#FF5722',
             'reserve-c2-sw':   '#9C27B0'}
MARKERS   = {'round-robin-sw':  'o',
             'pf-classic-sw':   's',
             'max-weight-sw':   '^',
             'aoi-sw':          'D',
             'reserve-c2-sw':   'P'}

K_VALUES = sorted(df['K'].unique())
C_VALUES = sorted(df['capacity'].unique())

def fmt_ax(ax, K, ylabel):
    ax.set_title(f'K = {K}', fontweight='bold')
    ax.set_xlabel('Capacidade C')
    ax.set_ylabel(ylabel)
    ax.set_xticks(C_VALUES)
    ax.yaxis.set_major_formatter(ticker.ScalarFormatter(useMathText=True))
    ax.ticklabel_format(axis='y', style='sci', scilimits=(0, 0))
    ax.grid(True, ls='--', alpha=0.4)

def line_plot(metric, ylabel, title, fname):
    fig, axes = plt.subplots(2, 3, figsize=(16, 9), constrained_layout=True)
    fig.suptitle(f'{title} — 2r_variavel  (2/3 RouteA, 5 políticas)', fontsize=13)
    for ki, K in enumerate(K_VALUES):
        ax  = axes.flatten()[ki]
        sub = df[df['K'] == K]
        for pol in POLICIES:
            d = sub[sub['policy'] == pol].sort_values('capacity')
            lw = 2.5 if pol == 'reserve-c2-sw' else 1.8
            ax.plot(d['capacity'], d[metric], marker=MARKERS[pol],
                    color=COLORS[pol], lw=lw, ms=7,
                    label=POL_LABEL[pol],
                    zorder=3 if pol == 'reserve-c2-sw' else 2)
        fmt_ax(ax, K, ylabel)
        if ki == 0: ax.legend(fontsize=8)
    fig.savefig(fname, dpi=150, bbox_inches='tight')
    print(f'Saved: {fname}')
    plt.close(fig)

def heatmap_gain(metric, title, fname):
    compare = [p for p in POLICIES if p != 'round-robin-sw']
    fig, axes = plt.subplots(1, len(compare), figsize=(20, 5), constrained_layout=True)
    fig.suptitle(f'{title} — redução relativa vs Round-Robin', fontsize=13)
    for ci, pol in enumerate(compare):
        ax  = axes[ci]
        mat = np.full((len(K_VALUES), len(C_VALUES)), np.nan)
        for ki, K in enumerate(K_VALUES):
            for cj, C in enumerate(C_VALUES):
                rr = df[(df['K']==K)&(df['capacity']==C)&(df['policy']=='round-robin-sw')][metric]
                pp = df[(df['K']==K)&(df['capacity']==C)&(df['policy']==pol)][metric]
                if len(rr) and len(pp) and rr.values[0] > 0:
                    mat[ki, cj] = (rr.values[0] - pp.values[0]) / rr.values[0] * 100
        im = ax.imshow(mat, cmap='RdYlGn', aspect='auto', vmin=-20, vmax=20)
        ax.set_xticks(range(len(C_VALUES))); ax.set_xticklabels(C_VALUES)
        ax.set_yticks(range(len(K_VALUES))); ax.set_yticklabels(K_VALUES)
        ax.set_xlabel('C'); ax.set_ylabel('K')
        bld = pol == 'reserve-c2-sw'
        ax.set_title(POL_LABEL[pol], fontweight='bold',
                     color=COLORS[pol] if bld else 'black',
                     fontsize=11 if bld else 10)
        for ki in range(len(K_VALUES)):
            for cj in range(len(C_VALUES)):
                if not np.isnan(mat[ki, cj]):
                    ax.text(cj, ki, f'{mat[ki,cj]:+.1f}%', ha='center', va='center',
                            fontsize=7, fontweight='bold' if bld else 'normal')
        plt.colorbar(im, ax=ax, label='Redução (%)\n(+ = melhor que RR)')
    fig.savefig(fname, dpi=150, bbox_inches='tight')
    print(f'Saved: {fname}')
    plt.close(fig)

# ── Plots de linha ────────────────────────────────────────────────────────────
line_plot('mean_h1',   'AoI médio C2 (slots)',  'AoI médio C2 (mean_h1)',   'rc2_h1_vs_C.png')
line_plot('mean_r_sys','Risco médio R_sys',      'Risco médio do sistema',   'rc2_risk_mean.png')
line_plot('peak_r_sys','Pico de risco R_peak',   'Pico de risco do sistema', 'rc2_risk_peak.png')
line_plot('Jhat_emp',  'Ĵ empírico',             'Custo total Ĵ',            'rc2_Jhat.png')

# ── Heatmaps ──────────────────────────────────────────────────────────────────
heatmap_gain('mean_h1',    'AoI C2',        'rc2_heatmap_h1.png')
heatmap_gain('mean_r_sys', 'Risco médio',   'rc2_heatmap_risk.png')

# ── AoI vs K para C representativos ──────────────────────────────────────────
C_sel = [4, 16, 48]
fig, axes = plt.subplots(1, 3, figsize=(16, 5), constrained_layout=True)
fig.suptitle('AoI médio C2 vs K — capacidades representativas  (2/3 RouteA)', fontsize=13)
for ci, C in enumerate(C_sel):
    ax  = axes[ci]
    sub = df[df['capacity'] == C]
    for pol in POLICIES:
        d = sub[sub['policy'] == pol].sort_values('K')
        lw = 2.5 if pol == 'reserve-c2-sw' else 1.8
        ax.plot(d['K'], d['mean_h1'], marker=MARKERS[pol],
                color=COLORS[pol], lw=lw, ms=7, label=POL_LABEL[pol],
                zorder=3 if pol == 'reserve-c2-sw' else 2)
    ax.set_title(f'C = {C}', fontweight='bold')
    ax.set_xlabel('K (slots/época)')
    ax.set_ylabel('mean h₁ (slots)')
    ax.set_xticks(K_VALUES)
    ax.grid(True, ls='--', alpha=0.4)
    ax.legend(fontsize=8)
plt.savefig('rc2_h1_vs_K.png', dpi=150, bbox_inches='tight')
print('Saved: rc2_h1_vs_K.png')
plt.close()

# ── Comparação direta: reserve-c2-sw vs melhor concorrente por (K,C) ─────────
fig, axes = plt.subplots(2, 3, figsize=(16, 9), constrained_layout=True)
fig.suptitle('Reserve-C2 vs melhor concorrente — AoI C2  (2/3 RouteA)', fontsize=13)
competitors = ['round-robin-sw', 'pf-classic-sw', 'max-weight-sw', 'aoi-sw']
for ki, K in enumerate(K_VALUES):
    ax  = axes.flatten()[ki]
    sub = df[df['K'] == K]
    rc2 = sub[sub['policy'] == 'reserve-c2-sw'].sort_values('capacity')
    # melhor concorrente por C (menor mean_h1)
    best_comp = (sub[sub['policy'].isin(competitors)]
                   .loc[sub[sub['policy'].isin(competitors)]
                        .groupby('capacity')['mean_h1'].idxmin()])
    ax.plot(rc2['capacity'], rc2['mean_h1'],
            marker='P', color=COLORS['reserve-c2-sw'], lw=2.5, ms=9,
            label='Reserve-C2', zorder=4)
    for pol in competitors:
        d = sub[sub['policy'] == pol].sort_values('capacity')
        ax.plot(d['capacity'], d['mean_h1'], marker=MARKERS[pol],
                color=COLORS[pol], lw=1.2, ms=5, alpha=0.5, label=POL_LABEL[pol])
    ax.plot(best_comp['capacity'], best_comp['mean_h1'],
            'k--', lw=2, label='Melhor concorrente', zorder=3)
    ax.set_title(f'K = {K}', fontweight='bold')
    ax.set_xlabel('Capacidade C')
    ax.set_ylabel('mean h₁ (slots)')
    ax.set_xticks(C_VALUES)
    ax.grid(True, ls='--', alpha=0.4)
    if ki == 0: ax.legend(fontsize=7.5, ncol=2)
plt.savefig('rc2_h1_vs_best.png', dpi=150, bbox_inches='tight')
print('Saved: rc2_h1_vs_best.png')
plt.close()

print('\nTodos os plots salvos.')
