"""
plot_risk_top.py
Top-of-page figure: mean AND peak system risk vs load, one panel per K,
for three policies: Round-Robin (RR), PF, and Risk-Aware (max-risk-sw).

Solid line = mean risk, dashed line = peak risk (same color per policy).

Usage:
  python plot_risk_top.py --dir csv_export_mw_otimo_a23 --scenario 2r_a23
"""
import os, glob, argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from matplotlib.lines import Line2D

parser = argparse.ArgumentParser()
parser.add_argument('--dir',      default='csv_export_mw_otimo_a23')
parser.add_argument('--scenario', default='2r_a23')
parser.add_argument('--linear',   action='store_true',
                    help='linear Y axis (default: log, so mean & peak are both legible)')
args = parser.parse_args()

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR   = os.path.join(SCRIPT_DIR, args.dir)
OUT_DIR    = os.path.join(DATA_DIR, 'plots')
os.makedirs(OUT_DIR, exist_ok=True)

csvs = sorted(glob.glob(os.path.join(DATA_DIR, 'sweep_summary_*.csv')))
df = pd.concat([pd.read_csv(f) for f in csvs], ignore_index=True).drop_duplicates(
    subset=['capacity', 'K', 'scenario', 'policy'])
df = df[df['scenario'] == args.scenario]

# Three policies only
POLICIES  = ['round-robin-sw', 'pf-classic-sw', 'max-risk-sw']
POL_LABEL = {'round-robin-sw': 'RR', 'pf-classic-sw': 'PF', 'max-risk-sw': 'Risk-Aware'}
COLORS    = {'round-robin-sw': '#9E9E9E', 'pf-classic-sw': '#2196F3', 'max-risk-sw': '#43A047'}
MARKERS   = {'round-robin-sw': 'o', 'pf-classic-sw': 's', 'max-risk-sw': 'D'}

K_VALUES = sorted(df['K'].unique())
ncols = 3
nrows = int(np.ceil(len(K_VALUES) / ncols))
fig, axes = plt.subplots(nrows, ncols, figsize=(5.2*ncols, 3.9*nrows),
                         constrained_layout=True, squeeze=False)
axes = axes.flatten()
fig.suptitle(f'Mean and peak system risk vs load — {args.scenario}',
             fontsize=14, fontweight='bold')

for ki, K in enumerate(K_VALUES):
    ax = axes[ki]
    sub = df[df['K'] == K]
    loads = sorted(sub['capacity'].unique())
    for pol in POLICIES:
        d = sub[sub['policy'] == pol].sort_values('capacity')
        ax.plot(d['capacity'], d['mean_r_sys'], color=COLORS[pol],
                marker=MARKERS[pol], ms=6, lw=2.0, ls='-')                # mean
        ax.plot(d['capacity'], d['peak_r_sys'], color=COLORS[pol],
                marker=MARKERS[pol], ms=5, lw=1.8, ls='--', alpha=0.9)    # peak
    ax.set_title(f'K = {K}', fontweight='bold')
    ax.set_xlabel('Load (C)')
    ax.set_ylabel('System risk  $R_{sys}$')
    ax.set_xscale('log', base=2)
    ax.set_xticks(loads)
    ax.get_xaxis().set_major_formatter(ticker.ScalarFormatter())
    if args.linear:
        ax.ticklabel_format(axis='y', style='sci', scilimits=(0, 0))
    else:
        ax.set_yscale('log')
    ax.grid(True, which='both', ls='--', alpha=0.4)

for ax in axes[len(K_VALUES):]:
    ax.set_visible(False)

# Combined legend: policy colors + line-style meaning (mean vs peak)
policy_handles = [Line2D([0], [0], color=COLORS[p], marker=MARKERS[p], lw=2,
                         label=POL_LABEL[p]) for p in POLICIES]
style_handles  = [Line2D([0], [0], color='0.3', lw=2, ls='-',  label='Mean risk'),
                  Line2D([0], [0], color='0.3', lw=2, ls='--', label='Peak risk')]
fig.legend(handles=policy_handles + style_handles, ncol=5, loc='lower center',
           bbox_to_anchor=(0.5, -0.04), frameon=False, fontsize=10)

tag = '_linear' if args.linear else ''
out = os.path.join(OUT_DIR, f'risk_top_{args.scenario}{tag}.png')
fig.savefig(out, dpi=150, bbox_inches='tight')
print(f'Saved: {out}')
