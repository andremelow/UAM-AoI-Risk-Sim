"""
plot_jhat_by_K.py
Objetivo formal Jhat_emp (= <n_s*h1 + n_v*z>) vs Capacidade C,
um subplot por K, as 4 politicas de scheduling.

Usage:
  python plot_jhat_by_K.py --dir csv_export_mw_otimo_a23 --scenario 2r_a23
"""
import os, glob, argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

parser = argparse.ArgumentParser()
parser.add_argument('--dir',      default='csv_export_mw_otimo_a23')
parser.add_argument('--scenario', default='2r_a23')
parser.add_argument('--logy',     action='store_true',
                    help='eixo Y em log (melhor para comparar todas as faixas de C)')
args = parser.parse_args()

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR   = os.path.join(SCRIPT_DIR, args.dir)
OUT_DIR    = os.path.join(DATA_DIR, 'plots')
os.makedirs(OUT_DIR, exist_ok=True)

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

K_VALUES = sorted(df['K'].unique())
ncols = 3
nrows = int(np.ceil(len(K_VALUES) / ncols))
fig, axes = plt.subplots(nrows, ncols, figsize=(6*ncols, 4.6*nrows),
                         constrained_layout=True, squeeze=False)
axes = axes.flatten()
fig.suptitle(f'Objetivo formal $\\hat{{J}} = \\langle n_s h_1 + n_v z\\rangle$ vs C  '
             f'— {args.scenario}  (menor = melhor)', fontsize=14, fontweight='bold')

for ki, K in enumerate(K_VALUES):
    ax = axes[ki]
    sub = df[df['K'] == K]
    C_VALUES = sorted(sub['capacity'].unique())
    for pol in POLICIES:
        d = sub[sub['policy'] == pol].sort_values('capacity')
        ax.plot(d['capacity'], d['Jhat_emp'], marker=MARKERS[pol], color=COLORS[pol],
                label=POL_LABEL[pol], lw=2.2, ms=7)
    ax.set_title(f'K = {K}', fontweight='bold')
    ax.set_xlabel('Capacidade C (drones no corredor)')
    ax.set_ylabel(r'$\hat{J}_{emp}$')
    ax.set_xscale('log', base=2)
    ax.set_xticks(C_VALUES)
    ax.get_xaxis().set_major_formatter(ticker.ScalarFormatter())
    if args.logy:
        ax.set_yscale('log')
    else:
        ax.ticklabel_format(axis='y', style='sci', scilimits=(0, 0))
    ax.grid(True, which='both', ls='--', alpha=0.4)
    if ki == 0:
        ax.legend(fontsize=9)

for ax in axes[len(K_VALUES):]:
    ax.set_visible(False)

tag = '_logy' if args.logy else ''
out = os.path.join(OUT_DIR, f'jhat_by_K_{args.scenario}{tag}.png')
fig.savefig(out, dpi=150, bbox_inches='tight')
print(f'Saved: {out}')
