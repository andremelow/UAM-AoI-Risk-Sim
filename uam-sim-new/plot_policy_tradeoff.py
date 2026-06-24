"""
plot_policy_tradeoff.py
Trade-off das 4 politicas de scheduling no cenario 2r_a23:

  Painel 1 — Risco de pico   peak_r_sys  vs C   (max-risk vence)
  Painel 2 — Objetivo formal Jhat_emp    vs C   (mw-otimo vence; max-risk perde)
  Painel 3 — Frescura de C2  mean_h1      vs C   (custo de AoI do max-risk)

Mostra visualmente que max-risk minimiza o risco fisico mas e desalinhado
com o surrogate AoI-risco Jhat (= n_s*h1 + n_v*z) que o paper otimiza.

Usage:
  python plot_policy_tradeoff.py --dir csv_export_mw_otimo_a23 --scenario 2r_a23 --K 8
"""
import os, glob, argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

parser = argparse.ArgumentParser()
parser.add_argument('--dir',      default='csv_export_mw_otimo_a23')
parser.add_argument('--scenario', default='2r_a23')
parser.add_argument('--K',        default='2,8',
                    help='lista de K (linhas do grid), ex: 2,8')
args = parser.parse_args()
K_ROWS = [int(k) for k in str(args.K).split(',')]

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR   = os.path.join(SCRIPT_DIR, args.dir)
OUT_DIR    = os.path.join(DATA_DIR, 'plots')
os.makedirs(OUT_DIR, exist_ok=True)

# ── Carrega e funde todos os sweep_summary_*.csv (igual plot_mw_otimo_risk.py)
csvs = sorted(glob.glob(os.path.join(DATA_DIR, 'sweep_summary_*.csv')))
df = pd.concat([pd.read_csv(f) for f in csvs], ignore_index=True).drop_duplicates(
    subset=['capacity', 'K', 'scenario', 'policy'])
df = df[df['scenario'] == args.scenario]

POLICIES  = ['round-robin-sw', 'pf-classic-sw', 'mw-otimo', 'mw-sw-otimo', 'max-weight-sw', 'max-risk-nsw', 'max-risk-sw']
POL_LABEL = {'round-robin-sw': 'Round-Robin', 'pf-classic-sw': 'PF-Classic',
             'mw-otimo': 'MW-Otimo (non-switching)', 'mw-sw-otimo': 'MW-Otimo (switching)',
             'max-weight-sw': 'MW-SW (quadratico)', 'max-risk-nsw': 'Max-Risk (non-switching)',
             'max-risk-sw': 'Max-Risk (switching)'}
COLORS    = {'round-robin-sw': '#9E9E9E', 'pf-classic-sw': '#2196F3', 'mw-otimo': '#FB8C00',
             'mw-sw-otimo': '#E53935', 'max-weight-sw': '#8E24AA', 'max-risk-nsw': '#795548',
             'max-risk-sw': '#43A047'}
MARKERS   = {'round-robin-sw': 'o', 'pf-classic-sw': 's', 'mw-otimo': '>',
             'mw-sw-otimo': '^', 'max-weight-sw': 'v', 'max-risk-nsw': 'X', 'max-risk-sw': 'D'}

PANELS = [
    ('peak_r_sys', 'Risco de pico  R_sys',  'Risco de PICO',  True),
    ('Jhat_emp',   r'$\hat{J}=\langle n_s h_1 + n_v z\rangle$', 'Objetivo formal $\\hat{J}$', True),
    ('mean_h1',    r'$\overline{h_1}$ (slots)', 'Frescura de C2 ($\\overline{h_1}$)', False),
]

nrows = len(K_ROWS)
fig, axes = plt.subplots(nrows, 3, figsize=(18, 5.0*nrows),
                         constrained_layout=True, squeeze=False)
fig.suptitle(f'Trade-off das politicas de scheduling — {args.scenario}  '
             f'(em todos os paineis: menor = melhor)',
             fontsize=14, fontweight='bold')

for ri, K in enumerate(K_ROWS):
    sub_K = df[df['K'] == K]
    C_VALUES = sorted(sub_K['capacity'].unique())
    for ci, (metric, ylabel, title, sci) in enumerate(PANELS):
        ax = axes[ri][ci]
        for pol in POLICIES:
            d = sub_K[sub_K['policy'] == pol].sort_values('capacity')
            ax.plot(d['capacity'], d[metric], marker=MARKERS[pol], color=COLORS[pol],
                    label=POL_LABEL[pol], lw=2.2, ms=7)
        ax.set_title(f'{title}  —  K = {K}', fontsize=11, fontweight='bold')
        ax.set_xlabel('Capacidade C (drones no corredor)')
        ax.set_ylabel(ylabel)
        ax.set_xscale('log', base=2)
        ax.set_xticks(C_VALUES)
        ax.get_xaxis().set_major_formatter(ticker.ScalarFormatter())
        if sci:
            ax.ticklabel_format(axis='y', style='sci', scilimits=(0, 0))
        ax.grid(True, which='both', ls='--', alpha=0.4)
        if ri == 0 and ci == 0:
            ax.legend(fontsize=9)

ktag = '_'.join(str(k) for k in K_ROWS)
out = os.path.join(OUT_DIR, f'policy_tradeoff_{args.scenario}_K{ktag}.png')
fig.savefig(out, dpi=150, bbox_inches='tight')
print(f'Saved: {out}')
