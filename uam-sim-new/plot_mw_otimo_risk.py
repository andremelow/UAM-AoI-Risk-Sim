"""
plot_mw_otimo_risk.py
Plots de risco empírico (mean_r_sys e peak_r_sys) do experimento mw_otimo.
"""

import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

import argparse
parser = argparse.ArgumentParser()
parser.add_argument('--dir', default='csv_export_mw_otimo',
                    help='Diretório de resultados (default: csv_export_mw_otimo)')
args = parser.parse_args()

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR   = os.path.join(SCRIPT_DIR, args.dir)
CSV        = os.path.join(DATA_DIR, 'sweep_summary_all.csv')
OUT_DIR    = os.path.join(DATA_DIR, 'plots')
os.makedirs(OUT_DIR, exist_ok=True)

import glob as _glob
extra_csvs = sorted(_glob.glob(os.path.join(DATA_DIR, 'sweep_summary_*.csv')))
extra_csvs = [f for f in extra_csvs if os.path.basename(f) != 'sweep_summary_all.csv']
frames = [pd.read_csv(CSV)]
for f in extra_csvs:
    frames.append(pd.read_csv(f))
    print(f'[INFO] Merged extra CSV: {os.path.basename(f)}')
df = pd.concat(frames, ignore_index=True).drop_duplicates(
    subset=['capacity', 'K', 'scenario', 'policy'])

POLICIES  = ['round-robin-sw', 'pf-classic-sw', 'mw-otimo', 'mw-sw-otimo', 'max-weight-sw', 'max-risk-nsw', 'max-risk-sw']
# keep only policies actually present in the data (clean legend per dataset)
_present = set(df['policy'].unique())
POLICIES  = [p for p in POLICIES if p in _present]
POL_LABEL = {
    'round-robin-sw': 'Round-Robin',
    'pf-classic-sw':  'PF-Classic',
    'mw-otimo':       'MW-Otimo (non-switching)',
    'mw-sw-otimo':    'MW-Otimo (switching)',
    'max-weight-sw':  'MW-SW (quadratico)',
    'max-risk-nsw':   'Max-Risk (non-switching)',
    'max-risk-sw':    'Max-Risk (switching)',
}
COLORS = {
    'round-robin-sw': '#9E9E9E',
    'pf-classic-sw':  '#2196F3',
    'mw-otimo':       '#FB8C00',
    'mw-sw-otimo':    '#E53935',
    'max-weight-sw':  '#8E24AA',
    'max-risk-nsw':   '#795548',
    'max-risk-sw':    '#43A047',
}
MARKERS = {
    'round-robin-sw': 'o',
    'pf-classic-sw':  's',
    'mw-otimo':       '>',
    'mw-sw-otimo':    '^',
    'max-weight-sw':  'v',
    'max-risk-nsw':   'X',
    'max-risk-sw':    'D',
}

K_VALUES  = sorted(df['K'].unique())
C_VALUES  = sorted(df['capacity'].unique())
SCENARIOS = sorted(df['scenario'].unique())
C_SEL     = [v for v in [4, 16, 48] if v in C_VALUES]


def save(fig, name):
    path = os.path.join(OUT_DIR, f'{name}.png')
    fig.savefig(path, dpi=150, bbox_inches='tight')
    print(f'Saved: {path}')
    plt.close(fig)


def grid_by_K(metric, ylabel, title_prefix, fname_prefix, sci=True):
    """mean_r_sys ou peak_r_sys vs C, um subplot por K — para cada cenário."""
    for sc in SCENARIOS:
        sub_sc = df[df['scenario'] == sc]
        ncols = 3; nrows = int(np.ceil(len(K_VALUES) / ncols))
        fig, axes = plt.subplots(nrows, ncols, figsize=(16, 5*nrows),
                                 constrained_layout=True)
        fig.suptitle(f'{title_prefix} — {sc}', fontsize=13)
        axes = np.array(axes).flatten()

        for ki, K in enumerate(K_VALUES):
            ax = axes[ki]
            sub = sub_sc[sub_sc['K'] == K]
            for pol in POLICIES:
                d = sub[sub['policy'] == pol].sort_values('capacity')
                ax.plot(d['capacity'], d[metric], marker=MARKERS[pol],
                        color=COLORS[pol], label=POL_LABEL[pol], lw=2, ms=7)
            ax.set_title(f'K = {K}', fontweight='bold')
            ax.set_xlabel('Capacidade C')
            ax.set_ylabel(ylabel)
            ax.set_xticks(C_VALUES)
            if sci:
                ax.yaxis.set_major_formatter(ticker.ScalarFormatter(useMathText=True))
                ax.ticklabel_format(axis='y', style='sci', scilimits=(0, 0))
            ax.grid(True, ls='--', alpha=0.4)
            if ki == 0:
                ax.legend(fontsize=9)

        for ax in axes[len(K_VALUES):]:
            ax.set_visible(False)
        save(fig, f'{fname_prefix}_{sc}')


def grid_by_C(metric, ylabel, title_prefix, fname_prefix, sci=True):
    """mean_r_sys ou peak_r_sys vs K, C representativos — para cada cenário."""
    for sc in SCENARIOS:
        sub_sc = df[df['scenario'] == sc]
        fig, axes = plt.subplots(1, len(C_SEL), figsize=(6*len(C_SEL), 5),
                                 constrained_layout=True)
        if len(C_SEL) == 1:
            axes = [axes]
        fig.suptitle(f'{title_prefix} — {sc}', fontsize=13)

        for ci, C in enumerate(C_SEL):
            ax = axes[ci]
            sub = sub_sc[sub_sc['capacity'] == C]
            for pol in POLICIES:
                d = sub[sub['policy'] == pol].sort_values('K')
                ax.plot(d['K'], d[metric], marker=MARKERS[pol],
                        color=COLORS[pol], label=POL_LABEL[pol], lw=2, ms=7)
            ax.set_title(f'C = {C}', fontweight='bold')
            ax.set_xlabel('K (slots por época)')
            ax.set_ylabel(ylabel)
            ax.set_xticks(K_VALUES)
            if sci:
                ax.yaxis.set_major_formatter(ticker.ScalarFormatter(useMathText=True))
                ax.ticklabel_format(axis='y', style='sci', scilimits=(0, 0))
            ax.grid(True, ls='--', alpha=0.4)
            ax.legend(fontsize=9)

        save(fig, f'{fname_prefix}_{sc}')


def heatmap_gain(metric, title_prefix, fname_prefix):
    """Ganho percentual de PF e MW-Otimo vs RR — heatmap (K × C) por cenário."""
    compare_pols = [p for p in POLICIES if p != 'round-robin-sw']
    for sc in SCENARIOS:
        sub_sc = df[df['scenario'] == sc]
        fig, axes = plt.subplots(1, 2, figsize=(14, 5), constrained_layout=True)
        fig.suptitle(f'{title_prefix} — ganho vs Round-Robin — {sc}', fontsize=13)

        for ci, pol in enumerate(compare_pols):
            ax = axes[ci]
            mat = np.full((len(K_VALUES), len(C_VALUES)), np.nan)
            for ki, K in enumerate(K_VALUES):
                for cj, C in enumerate(C_VALUES):
                    rr = sub_sc[(sub_sc['K'] == K) & (sub_sc['capacity'] == C) &
                                (sub_sc['policy'] == 'round-robin-sw')][metric]
                    pp = sub_sc[(sub_sc['K'] == K) & (sub_sc['capacity'] == C) &
                                (sub_sc['policy'] == pol)][metric]
                    if len(rr) and len(pp) and rr.values[0] > 0:
                        mat[ki, cj] = (rr.values[0] - pp.values[0]) / rr.values[0] * 100

            vmax = np.nanpercentile(np.abs(mat), 95)
            vmax = max(vmax, 5)
            im = ax.imshow(mat, cmap='RdYlGn', aspect='auto', vmin=-vmax, vmax=vmax)
            ax.set_xticks(range(len(C_VALUES))); ax.set_xticklabels(C_VALUES)
            ax.set_yticks(range(len(K_VALUES))); ax.set_yticklabels(K_VALUES)
            ax.set_xlabel('Capacidade C'); ax.set_ylabel('K (slots/época)')
            ax.set_title(f'{POL_LABEL[pol]} vs RR', fontweight='bold')
            for ki in range(len(K_VALUES)):
                for cj in range(len(C_VALUES)):
                    v = mat[ki, cj]
                    if not np.isnan(v):
                        ax.text(cj, ki, f'{v:+.1f}%', ha='center', va='center',
                                fontsize=7.5, color='black')
            plt.colorbar(im, ax=ax, label='Redução de risco (%)\n(+ = melhor que RR)')

        save(fig, f'{fname_prefix}_{sc}')


def mean_vs_peak(sc):
    """Scatter: mean_r_sys vs peak_r_sys por política, C e K fixos."""
    sub_sc = df[df['scenario'] == sc]
    fig, axes = plt.subplots(1, len(C_SEL), figsize=(6*len(C_SEL), 5),
                             constrained_layout=True)
    if len(C_SEL) == 1:
        axes = [axes]
    fig.suptitle(f'Risco médio vs pico — {sc}', fontsize=13)

    for ci, C in enumerate(C_SEL):
        ax = axes[ci]
        sub = sub_sc[sub_sc['capacity'] == C]
        for pol in POLICIES:
            d = sub[sub['policy'] == pol]
            ax.scatter(d['mean_r_sys'], d['peak_r_sys'],
                       marker=MARKERS[pol], color=COLORS[pol],
                       label=POL_LABEL[pol], s=60, alpha=0.75)
        ax.set_title(f'C = {C}  (todos K)', fontweight='bold')
        ax.set_xlabel('Risco médio R_sys')
        ax.set_ylabel('Risco pico R_sys')
        ax.xaxis.set_major_formatter(ticker.ScalarFormatter(useMathText=True))
        ax.yaxis.set_major_formatter(ticker.ScalarFormatter(useMathText=True))
        ax.ticklabel_format(axis='both', style='sci', scilimits=(0, 0))
        ax.grid(True, ls='--', alpha=0.4)
        ax.legend(fontsize=9)

    save(fig, f'risk_scatter_{sc}')


# ── Gera todos os plots ────────────────────────────────────────────────────────
grid_by_K('mean_r_sys', 'Risco médio R_sys',
          'Risco médio do sistema vs Capacidade', 'risk_mean_vs_C')

grid_by_K('peak_r_sys', 'Risco pico R_sys',
          'Risco pico do sistema vs Capacidade', 'risk_peak_vs_C')

grid_by_C('mean_r_sys', 'Risco médio R_sys',
          'Risco médio do sistema vs K', 'risk_mean_vs_K')

grid_by_C('peak_r_sys', 'Risco pico R_sys',
          'Risco pico do sistema vs K', 'risk_peak_vs_K')

heatmap_gain('mean_r_sys', 'Risco médio', 'heatmap_risk_mean')
heatmap_gain('peak_r_sys', 'Risco pico',  'heatmap_risk_peak')

for sc in SCENARIOS:
    mean_vs_peak(sc)

print(f'\nTodos os plots salvos em: {OUT_DIR}')
