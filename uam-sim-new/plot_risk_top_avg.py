"""
plot_risk_top_avg.py
Top-of-page figure: mean AND peak system risk vs load, averaged over the two
scenarios (2r_static + 2r_a23), one panel per K, side by side (1 x 6).

Three policies: Round-Robin (RR), PF, Risk-Aware (max-risk-sw).
Colors: RR = yellow, PF = red, Risk-Aware = blue.
Solid line = mean risk, dashed line = peak risk (same color per policy).

Usage:
  python plot_risk_top_avg.py --dir csv_export_mw_otimo_a23
"""
import os, glob, argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from matplotlib.lines import Line2D

parser = argparse.ArgumentParser()
parser.add_argument('--dir', default='csv_export_mw_otimo_a23')
parser.add_argument('--scenarios', default='2r_static,2r_a23')
parser.add_argument('--linear', action='store_true',
                    help='linear Y axis (default: log)')
parser.add_argument('--ncols', type=int, default=6,
                    help='panels per row (6 = wide top-of-page; 2 = single-column 2-up)')
parser.add_argument('--K', default='',
                    help='comma list of K to include (default: all), e.g. 2,4,8')
parser.add_argument('--C', default='',
                    help='comma list of loads (capacity) to include (default: all), e.g. 2,4,8')
parser.add_argument('--no-peak', dest='no_peak', action='store_true',
                    help='plot only the mean risk (omit the peak/dashed curves)')
parser.add_argument('--twin', action='store_true',
                    help='dual y-axis: mean on left axis, peak on right axis (own scale each)')
parser.add_argument('--split', action='store_true',
                    help='two rows: top = mean, bottom = peak (K across columns, own scale each)')
args = parser.parse_args()

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR   = os.path.join(SCRIPT_DIR, args.dir)
OUT_DIR    = os.path.join(DATA_DIR, 'plots')
os.makedirs(OUT_DIR, exist_ok=True)
SCEN = args.scenarios.split(',')

csvs = sorted(glob.glob(os.path.join(DATA_DIR, 'sweep_summary_*.csv')))
df = pd.concat([pd.read_csv(f) for f in csvs], ignore_index=True).drop_duplicates(
    subset=['capacity', 'K', 'scenario', 'policy'])
df = df[df['scenario'].isin(SCEN)]
if args.C:
    keepC = [int(c) for c in args.C.split(',')]
    df = df[df['capacity'].isin(keepC)]

# Average the two scenarios per (policy, K, capacity)
df = (df.groupby(['policy', 'K', 'capacity'], as_index=False)
        [['mean_r_sys', 'peak_r_sys']].mean())

POLICIES  = ['round-robin-sw', 'pf-classic-sw', 'max-risk-sw']
POL_LABEL = {'round-robin-sw': 'RR', 'pf-classic-sw': 'PF', 'max-risk-sw': 'Risk-Aware'}
COLORS    = {'round-robin-sw': '#F9A825', 'pf-classic-sw': '#E53935', 'max-risk-sw': '#1E88E5'}
MARKERS   = {'round-robin-sw': 'o', 'pf-classic-sw': 's', 'max-risk-sw': 'D'}

K_VALUES = sorted(df['K'].unique())
if args.K:
    keep = [int(k) for k in args.K.split(',')]
    K_VALUES = [k for k in K_VALUES if k in keep]

policy_handles = [Line2D([0], [0], color=COLORS[p], marker=MARKERS[p], lw=2,
                         label=POL_LABEL[p]) for p in POLICIES]

# ── Split mode: row 1 = mean, row 2 = peak; K across columns ───────────────
if args.split:
    metrics = [('mean_r_sys', 'Mean risk', '-'), ('peak_r_sys', 'Peak risk', '--')]
    ncols = len(K_VALUES)
    fig, axes = plt.subplots(2, ncols, figsize=(3.0*ncols, 6.6),
                             constrained_layout=True, squeeze=False)
    for ri, (metric, mlabel, ls) in enumerate(metrics):
        for ci, K in enumerate(K_VALUES):
            ax = axes[ri][ci]
            sub = df[df['K'] == K]
            loads = sorted(sub['capacity'].unique())
            for pol in POLICIES:
                d = sub[sub['policy'] == pol].sort_values('capacity')
                ax.plot(d['capacity'], d[metric], color=COLORS[pol],
                        marker=MARKERS[pol], ms=5, lw=2.0, ls=ls)
            if ri == 0:
                ax.set_title(f'K = {K}', fontweight='bold')
            else:
                ax.set_xlabel('Simultaneous UAVs')
            if ci == 0:
                ax.set_ylabel(f'{mlabel}  $R_{{sys}}$')
            ax.set_xticks(loads)
            ax.tick_params(axis='x', labelsize=8)
            ax.ticklabel_format(axis='y', style='sci', scilimits=(0, 0))
            ax.grid(True, ls='--', alpha=0.4)
    fig.legend(handles=policy_handles, ncol=len(POLICIES), loc='lower center',
               bbox_to_anchor=(0.5, -0.04), frameon=False, fontsize=10)
    ktag = f'_K{"-".join(str(k) for k in K_VALUES)}' if args.K else ''
    ctag = f'_C{args.C.replace(",", "-")}' if args.C else ''
    base = os.path.join(OUT_DIR, f'risk_top_avg_linear_split{ktag}{ctag}')
    for ext in ('pdf', 'png'):
        fig.savefig(f'{base}.{ext}', dpi=150, bbox_inches='tight')
        print(f'Saved: {base}.{ext}')
    raise SystemExit

ncols = max(1, min(args.ncols, len(K_VALUES)))
nrows = int(np.ceil(len(K_VALUES) / ncols))
fig, axes = plt.subplots(nrows, ncols, figsize=(3.6*ncols, 3.4*nrows),
                         constrained_layout=True, squeeze=False)
axes = axes.flatten()
# No title: it goes in the paper caption.

twin = args.twin and not args.no_peak
for ki, K in enumerate(K_VALUES):
    ax = axes[ki]
    axp = ax.twinx() if twin else ax          # right axis for peak (own scale)
    sub = df[df['K'] == K]
    loads = sorted(sub['capacity'].unique())
    for pol in POLICIES:
        d = sub[sub['policy'] == pol].sort_values('capacity')
        ax.plot(d['capacity'], d['mean_r_sys'], color=COLORS[pol],
                marker=MARKERS[pol], ms=5, lw=2.0, ls='-')               # mean (left)
        if not args.no_peak:
            axp.plot(d['capacity'], d['peak_r_sys'], color=COLORS[pol],
                     marker=MARKERS[pol], ms=4, lw=1.8, ls='--', alpha=0.9)  # peak (right)
    ax.set_title(f'K = {K}', fontweight='bold')
    ax.set_xlabel('Simultaneous UAVs')
    ax.set_xticks(loads)            # linear x-axis (true load spacing)
    ax.tick_params(axis='x', labelsize=8)
    if args.linear or twin:
        ax.ticklabel_format(axis='y', style='sci', scilimits=(0, 0))
    else:
        ax.set_yscale('log')
    if twin:
        axp.ticklabel_format(axis='y', style='sci', scilimits=(0, 0))
        if ki % ncols == 0:
            ax.set_ylabel('Mean risk  $R_{sys}$')
        if ki % ncols == ncols - 1 or ki == len(K_VALUES) - 1:
            axp.set_ylabel('Peak risk  $R_{sys}$')
    elif ki % ncols == 0:
        ax.set_ylabel('System risk  $R_{sys}$')
    ax.grid(True, which='both', ls='--', alpha=0.4)

policy_handles = [Line2D([0], [0], color=COLORS[p], marker=MARKERS[p], lw=2,
                         label=POL_LABEL[p]) for p in POLICIES]
if args.no_peak:
    style_handles = []
elif twin:
    style_handles = [Line2D([0], [0], color='0.3', lw=2, ls='-',  label='Mean (left axis)'),
                     Line2D([0], [0], color='0.3', lw=2, ls='--', label='Peak (right axis)')]
else:
    style_handles = [Line2D([0], [0], color='0.3', lw=2, ls='-',  label='Mean risk'),
                     Line2D([0], [0], color='0.3', lw=2, ls='--', label='Peak risk')]
fig.legend(handles=policy_handles + style_handles, ncol=5, loc='lower center',
           bbox_to_anchor=(0.5, -0.06), frameon=False, fontsize=10)

tag  = '_linear' if args.linear else ''
tag += '' if ncols == len(K_VALUES) else f'_{ncols}col'
tag += f'_K{"-".join(str(k) for k in K_VALUES)}' if args.K else ''
tag += f'_C{args.C.replace(",", "-")}' if args.C else ''
tag += '_meanonly' if args.no_peak else ''
tag += '_twin' if twin else ''
base = os.path.join(OUT_DIR, f'risk_top_avg{tag}')
for ext in ('pdf', 'png'):                       # PDF (vetorial p/ paper) + PNG preview
    fig.savefig(f'{base}.{ext}', dpi=150, bbox_inches='tight')
    print(f'Saved: {base}.{ext}')
