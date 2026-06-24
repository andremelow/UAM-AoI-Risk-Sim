"""
plot_mw_otimo.py
Plots do experimento experiment_mw_otimo:
  Round-Robin-SW vs PF-Classic-SW vs MW-SW-Otimo

Uso:
  cd uam-sim-new
  python plot_mw_otimo.py                        # todos os cenários
  python plot_mw_otimo.py --scenario 2r_variavel # filtrado
"""

import argparse
import os
import sys

import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np
import pandas as pd

# ── CLI ──────────────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser()
parser.add_argument('--scenario', default=None,
                    help='Filtrar por cenário (2r_static | 2r_variavel | 2r_a23)')
parser.add_argument('--dir', default='csv_export_mw_otimo',
                    help='Diretório de resultados (default: csv_export_mw_otimo)')
args = parser.parse_args()

# ── Caminhos ──────────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR   = os.path.join(SCRIPT_DIR, args.dir)
CSV        = os.path.join(DATA_DIR, 'sweep_summary_all.csv')
OUT_DIR    = os.path.join(DATA_DIR, 'plots')
os.makedirs(OUT_DIR, exist_ok=True)

if not os.path.exists(CSV):
    print(f'[ERRO] CSV não encontrado: {CSV}')
    print(f'Rode experiment_mw_otimo() no MATLAB primeiro (ou --dir correto).')
    sys.exit(1)

# Lê sweep_summary_all.csv e concatena qualquer sweep_summary_*.csv extra
# (ex: sweep_summary_maxrisk.csv gerado pelo experimento complementar).
import glob as _glob
extra_csvs = sorted(_glob.glob(os.path.join(DATA_DIR, 'sweep_summary_*.csv')))
extra_csvs = [f for f in extra_csvs if os.path.basename(f) != 'sweep_summary_all.csv']
frames = [pd.read_csv(CSV)]
for f in extra_csvs:
    frames.append(pd.read_csv(f))
    print(f'[INFO] Merged extra CSV: {os.path.basename(f)}')
df = pd.concat(frames, ignore_index=True).drop_duplicates(
    subset=['capacity', 'K', 'scenario', 'policy'])

if args.scenario:
    df = df[df['scenario'] == args.scenario]
    if df.empty:
        print(f'[ERRO] Nenhum dado para cenário "{args.scenario}".')
        sys.exit(1)

POLICIES  = ['round-robin-sw', 'pf-classic-sw', 'mw-sw-otimo', 'max-risk-sw']
POL_LABEL = {
    'round-robin-sw': 'Round-Robin',
    'pf-classic-sw':  'PF-Classic',
    'mw-sw-otimo':    'MW-Otimo (ψ≤2)',
    'max-risk-sw':    'Max-Risk',
}
COLORS = {
    'round-robin-sw': '#9E9E9E',
    'pf-classic-sw':  '#2196F3',
    'mw-sw-otimo':    '#E53935',
    'max-risk-sw':    '#43A047',
}
MARKERS = {
    'round-robin-sw': 'o',
    'pf-classic-sw':  's',
    'mw-sw-otimo':    '^',
    'max-risk-sw':    'D',
}

K_VALUES = sorted(df['K'].unique())
C_VALUES = sorted(df['capacity'].unique())
SCENARIOS = sorted(df['scenario'].unique())

suffix = f'_{args.scenario}' if args.scenario else ''


def save(fig, name):
    path = os.path.join(OUT_DIR, f'{name}{suffix}.png')
    fig.savefig(path, dpi=150, bbox_inches='tight')
    print(f'Saved: {path}')
    plt.close(fig)


# ─────────────────────────────────────────────────────────────────────────────
# Fig 1 — mean_h1 (AoI C2) vs C, um subplot por K
# ─────────────────────────────────────────────────────────────────────────────
for sc in SCENARIOS:
    sub_sc = df[df['scenario'] == sc]
    nK = len(K_VALUES)
    ncols = 3; nrows = int(np.ceil(nK / ncols))
    fig, axes = plt.subplots(nrows, ncols, figsize=(16, 5*nrows),
                             constrained_layout=True)
    fig.suptitle(f'AoI médio C2 (h₁) vs Capacidade — {sc}', fontsize=13)
    axes = np.array(axes).flatten()

    for ki, K in enumerate(K_VALUES):
        ax = axes[ki]
        sub = sub_sc[sub_sc['K'] == K]
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
            ax.legend(fontsize=9)

    for ax in axes[len(K_VALUES):]:
        ax.set_visible(False)
    save(fig, f'h1_vs_C_{sc}')

# ─────────────────────────────────────────────────────────────────────────────
# Fig 2 — Jhat_emp vs C, um subplot por K  (+ L_B do Theorem 1)
# ─────────────────────────────────────────────────────────────────────────────
for sc in SCENARIOS:
    sub_sc = df[df['scenario'] == sc]
    nK = len(K_VALUES)
    ncols = 3; nrows = int(np.ceil(nK / ncols))
    fig, axes = plt.subplots(nrows, ncols, figsize=(16, 5*nrows),
                             constrained_layout=True)
    fig.suptitle(f'Custo empírico Ĵ vs Capacidade — {sc}', fontsize=13)
    axes = np.array(axes).flatten()

    for ki, K in enumerate(K_VALUES):
        ax = axes[ki]
        sub = sub_sc[sub_sc['K'] == K]
        for pol in POLICIES:
            d = sub[sub['policy'] == pol].sort_values('capacity')
            ax.plot(d['capacity'], d['Jhat_emp'], marker=MARKERS[pol],
                    color=COLORS[pol], label=POL_LABEL[pol], lw=2, ms=7)
        # Lower bound L_B from Theorem 1 (same value for all policies at same K,C)
        lb = (sub.groupby('capacity')['J_LB_doc'].mean()
                 .reset_index().sort_values('capacity'))
        lb = lb[lb['J_LB_doc'] > 0]
        if not lb.empty:
            ax.plot(lb['capacity'], lb['J_LB_doc'], 'k--', lw=1.5,
                    label=r'$L_B$ (Theorem 1)')
        ax.set_title(f'K = {K}', fontweight='bold')
        ax.set_xlabel('Capacidade C')
        ax.set_ylabel('Ĵ empírico')
        ax.set_xticks(C_VALUES)
        ax.yaxis.set_major_formatter(ticker.ScalarFormatter(useMathText=True))
        ax.ticklabel_format(axis='y', style='sci', scilimits=(0, 0))
        ax.grid(True, ls='--', alpha=0.4)
        if ki == 0:
            ax.legend(fontsize=9)

    for ax in axes[len(K_VALUES):]:
        ax.set_visible(False)
    save(fig, f'Jhat_vs_C_{sc}')

# ─────────────────────────────────────────────────────────────────────────────
# Fig 3 — heatmap ganho MW-Otimo vs RR  (mean_h1) por (K, C)  — por cenário
# ─────────────────────────────────────────────────────────────────────────────
compare_pols = ['pf-classic-sw', 'mw-sw-otimo']
for sc in SCENARIOS:
    sub_sc = df[df['scenario'] == sc]
    fig, axes = plt.subplots(1, 2, figsize=(14, 5), constrained_layout=True)
    fig.suptitle(f'Ganho de h₁ relativo ao Round-Robin — {sc}', fontsize=13)

    for ci, pol in enumerate(compare_pols):
        ax = axes[ci]
        mat = np.full((len(K_VALUES), len(C_VALUES)), np.nan)
        for ki, K in enumerate(K_VALUES):
            for cj, C in enumerate(C_VALUES):
                rr = sub_sc[(sub_sc['K'] == K) & (sub_sc['capacity'] == C) &
                            (sub_sc['policy'] == 'round-robin-sw')]['mean_h1']
                pp = sub_sc[(sub_sc['K'] == K) & (sub_sc['capacity'] == C) &
                            (sub_sc['policy'] == pol)]['mean_h1']
                if len(rr) and len(pp) and rr.values[0] > 0:
                    mat[ki, cj] = (rr.values[0] - pp.values[0]) / rr.values[0] * 100

        im = ax.imshow(mat, cmap='RdYlGn', aspect='auto', vmin=-25, vmax=25)
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
        plt.colorbar(im, ax=ax, label='Redução de h₁ (%)\n(+ = melhor que RR)')

    save(fig, f'heatmap_gain_{sc}')

# ─────────────────────────────────────────────────────────────────────────────
# Fig 4 — razão de otimalidade empírica  ρ_emp = Jhat / J_LB
# ─────────────────────────────────────────────────────────────────────────────
for sc in SCENARIOS:
    sub_sc = df[df['scenario'] == sc].copy()
    sub_sc = sub_sc[sub_sc['J_LB_doc'] > 0]
    sub_sc['psi_emp'] = sub_sc['Jhat_emp'] / sub_sc['J_LB_doc']

    nK = len(K_VALUES)
    ncols = 3; nrows = int(np.ceil(nK / ncols))
    fig, axes = plt.subplots(nrows, ncols, figsize=(16, 5*nrows),
                             constrained_layout=True)
    fig.suptitle(f'Razão de otimalidade empírica ψ = Ĵ / J_LB — {sc}', fontsize=13)
    axes = np.array(axes).flatten()

    for ki, K in enumerate(K_VALUES):
        ax = axes[ki]
        sub = sub_sc[sub_sc['K'] == K]
        for pol in POLICIES:
            d = sub[sub['policy'] == pol].sort_values('capacity')
            ax.plot(d['capacity'], d['psi_emp'], marker=MARKERS[pol],
                    color=COLORS[pol], label=POL_LABEL[pol], lw=2, ms=7)
        ax.axhline(2.0, color='black', ls='--', lw=1.2, label=r'$\psi = 2$ (Theorem 2)')
        ax.axhline(1.0, color='black', ls=':',  lw=1.2, label=r'$\psi = 1$ ($\hat{J} = L_B$)')
        ax.set_title(f'K = {K}', fontweight='bold')
        ax.set_xlabel('Capacidade C')
        ax.set_ylabel(r'$\psi$ empírico $= \hat{J} / L_B$')
        ax.set_xticks(C_VALUES)
        ax.grid(True, ls='--', alpha=0.4)
        if ki == 0:
            ax.legend(fontsize=8)

    for ax in axes[len(K_VALUES):]:
        ax.set_visible(False)
    save(fig, f'psi_vs_C_{sc}')

# ─────────────────────────────────────────────────────────────────────────────
# Fig 5 — mean_h1 vs K, C representativos
# ─────────────────────────────────────────────────────────────────────────────
C_sel = [v for v in [4, 16, 48] if v in C_VALUES]
for sc in SCENARIOS:
    sub_sc = df[df['scenario'] == sc]
    fig, axes = plt.subplots(1, len(C_sel), figsize=(6*len(C_sel), 5),
                             constrained_layout=True)
    if len(C_sel) == 1:
        axes = [axes]
    fig.suptitle(f'AoI médio C2 vs K — {sc}', fontsize=13)

    for ci, C in enumerate(C_sel):
        ax = axes[ci]
        sub = sub_sc[sub_sc['capacity'] == C]
        for pol in POLICIES:
            d = sub[sub['policy'] == pol].sort_values('K')
            ax.plot(d['K'], d['mean_h1'], marker=MARKERS[pol],
                    color=COLORS[pol], label=POL_LABEL[pol], lw=2, ms=7)
        ax.set_title(f'C = {C}', fontweight='bold')
        ax.set_xlabel('K (slots por época)')
        ax.set_ylabel('mean h₁ (slots)')
        ax.set_xticks(K_VALUES)
        ax.grid(True, ls='--', alpha=0.4)
        ax.legend(fontsize=9)

    save(fig, f'h1_vs_K_{sc}')

# ─────────────────────────────────────────────────────────────────────────────
# Fig 6 — Jhat_emp vs K, C representativos  (+ L_B do Theorem 1)
# ─────────────────────────────────────────────────────────────────────────────
for sc in SCENARIOS:
    sub_sc = df[df['scenario'] == sc]
    fig, axes = plt.subplots(1, len(C_sel), figsize=(6*len(C_sel), 5),
                             constrained_layout=True)
    if len(C_sel) == 1:
        axes = [axes]
    fig.suptitle(f'Custo empírico Ĵ vs K — {sc}', fontsize=13)

    for ci, C in enumerate(C_sel):
        ax = axes[ci]
        sub = sub_sc[sub_sc['capacity'] == C]
        for pol in POLICIES:
            d = sub[sub['policy'] == pol].sort_values('K')
            ax.plot(d['K'], d['Jhat_emp'], marker=MARKERS[pol],
                    color=COLORS[pol], label=POL_LABEL[pol], lw=2, ms=7)
        # Lower bound L_B varies with K (1/(2K) factor in A_sys term)
        lb = (sub.groupby('K')['J_LB_doc'].mean()
                 .reset_index().sort_values('K'))
        lb = lb[lb['J_LB_doc'] > 0]
        if not lb.empty:
            ax.plot(lb['K'], lb['J_LB_doc'], 'k--', lw=1.5,
                    label=r'$L_B$ (Theorem 1)')
        ax.set_title(f'C = {C}', fontweight='bold')
        ax.set_xlabel('K (slots por época)')
        ax.set_ylabel('Ĵ empírico')
        ax.set_xticks(K_VALUES)
        ax.yaxis.set_major_formatter(ticker.ScalarFormatter(useMathText=True))
        ax.ticklabel_format(axis='y', style='sci', scilimits=(0, 0))
        ax.grid(True, ls='--', alpha=0.4)
        ax.legend(fontsize=9)

    save(fig, f'Jhat_vs_K_{sc}')

print(f'\nTodos os plots salvos em: {OUT_DIR}')
