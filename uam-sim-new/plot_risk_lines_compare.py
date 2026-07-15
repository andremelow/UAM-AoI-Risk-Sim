"""
plot_risk_lines_compare.py
Plots de LINHA de risco médio (mean_r_sys) e pico (peak_r_sys) comparando
políticas que vivem em diretórios diferentes (mesmo schema de CSV).

Por padrão: baseline RR/PF/MR do gop_final + AoI-sw do sweep complementar.
Gera APENAS as linhas de risco (vs C, subplot por K; e vs K, C representativos),
por cenário. Heatmaps/scatter omitidos.

Uso:
    python3 plot_risk_lines_compare.py
    python3 plot_risk_lines_compare.py \
        --dirs csv_export_mw_otimo_a23_gop_final,csv_export_mw_otimo_a23_gop_aoisw \
        --policies round-robin-sw,pf-classic-sw,max-risk-sw,aoi-sw \
        --out csv_export_mw_otimo_a23_gop_aoisw/plots
"""

import os
import glob
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

parser = argparse.ArgumentParser()
parser.add_argument('--dirs',
                    default='csv_export_mw_otimo_a23_gop_final,'
                            'csv_export_mw_otimo_a23_gop_aoisw',
                    help='Lista de diretórios a mesclar (vírgula)')
parser.add_argument('--policies',
                    default='round-robin-sw,pf-classic-sw,max-risk-sw,aoi-sw',
                    help='Políticas a plotar, na ordem (vírgula)')
parser.add_argument('--out', default=None,
                    help='Dir de saída (default: <último dir>/plots)')
parser.add_argument('--lb_dir', default='csv_export_mw_otimo_a23_gop_lb',
                    help='Dir do lower-bound prático (RR @ K=128); "" desativa')
args = parser.parse_args()

DIRS = [os.path.join(SCRIPT_DIR, d.strip()) for d in args.dirs.split(',')]
OUT_DIR = args.out or os.path.join(DIRS[-1], 'plots')
if not os.path.isabs(OUT_DIR):
    OUT_DIR = os.path.join(SCRIPT_DIR, OUT_DIR)
os.makedirs(OUT_DIR, exist_ok=True)


def load_dir(d):
    frames = []
    main = os.path.join(d, 'sweep_summary_all.csv')
    if os.path.exists(main):
        frames.append(pd.read_csv(main))
    for f in sorted(glob.glob(os.path.join(d, 'sweep_summary_*.csv'))):
        if os.path.basename(f) != 'sweep_summary_all.csv':
            frames.append(pd.read_csv(f))
    return frames


frames = []
for d in DIRS:
    frames += load_dir(d)
if not frames:
    raise SystemExit(f'Nenhum CSV encontrado em: {DIRS}')
df = pd.concat(frames, ignore_index=True).drop_duplicates(
    subset=['capacity', 'K', 'scenario', 'policy'])

# ── Dicionário de estilo (cobre todas as políticas conhecidas) ──────────────────
POL_LABEL = {
    'round-robin-sw': 'RR',
    'pf-classic-sw':  'PF',
    'max-risk-sw':    'MR-sw (heur.)',
    'max-risk-nsw':   'MR-nsw (heur.)',
    'max-risk-v2':    'MR-v2 (sw)',
    'max-risk-v2-nsw':'MR-v2-nsw (non-sw)',
    'aoi-sw':         'AoI-sw',
    'mw-sw-otimo':    'MW-Otimo (sw)',
    'mw-otimo':       'MW-Otimo (non-sw)',
    'mw-sw-otimo-ch': 'MW-Opt-CH (sw)',
    'mr-mw-ch-sw':    'MR-MW-CH (sw)',
    'mw-otimo-ch':    'MW-Opt-CH (ns)',
    'mw3-sn':         'MW3-sn (chan)',
    'mw3-nsw':        'MW3-nsw (chan)',
}
COLORS = {
    'round-robin-sw': '#9E9E9E',
    'pf-classic-sw':  '#2196F3',
    'max-risk-sw':    '#43A047',
    'max-risk-nsw':   '#00897B',
    'max-risk-v2':    '#E53935',
    'max-risk-v2-nsw':'#6A1B9A',
    'aoi-sw':         '#8E24AA',
    'mw-sw-otimo':    '#FB8C00',
    'mw-otimo':       '#6D4C41',
    'mw-sw-otimo-ch': '#1565C0',
    'mr-mw-ch-sw':    '#8E24AA',
    'mw-otimo-ch':    '#00897B',
    'mw3-sn':         '#E53935',
    'mw3-nsw':        '#6A1B9A',
}
MARKERS = {
    'round-robin-sw': 'o',
    'pf-classic-sw':  's',
    'max-risk-sw':    'D',
    'max-risk-nsw':   'X',
    'max-risk-v2':    '^',
    'max-risk-v2-nsw':'v',
    'aoi-sw':         'v',
    'mw-sw-otimo':    '>',
    'mw-otimo':       '<',
    'mw-sw-otimo-ch': '>',
    'mr-mw-ch-sw':    '*',
    'mw-otimo-ch':    'P',
    'mw3-sn':         '^',
    'mw3-nsw':        'v',
}

POLICIES = [p.strip() for p in args.policies.split(',')]
_present = set(df['policy'].unique())
missing  = [p for p in POLICIES if p not in _present]
if missing:
    print(f'[AVISO] políticas ausentes nos dados: {missing}')
POLICIES = [p for p in POLICIES if p in _present]

K_VALUES  = sorted(df['K'].unique())
C_VALUES  = sorted(df['capacity'].unique())
SCENARIOS = sorted(df['scenario'].unique())
C_SEL     = [v for v in [4, 16, 48] if v in C_VALUES] or C_VALUES

# ── Lower-bound prático: RR @ K=128 (sem filas) ─────────────────────────────────
LB_LABEL = 'LB prático (RR, K=128)'
lb_df = None
if args.lb_dir:
    lb_frames = load_dir(os.path.join(SCRIPT_DIR, args.lb_dir))
    if lb_frames:
        lb_df = pd.concat(lb_frames, ignore_index=True)
        lb_df = lb_df[lb_df['policy'] == 'round-robin-sw']
        print(f'[LB] {len(lb_df)} linhas RR @ K={sorted(lb_df["K"].unique())} '
              f'de {args.lb_dir}')
    else:
        print(f'[AVISO] LB dir sem CSV: {args.lb_dir}')


def lb_curve(sc, metric):
    """(C ordenado, valor LB) para o cenário sc; None se indisponível."""
    if lb_df is None:
        return None
    d = lb_df[lb_df['scenario'] == sc].sort_values('capacity')
    return (d['capacity'].values, d[metric].values) if not d.empty else None


def save(fig, name):
    path = os.path.join(OUT_DIR, f'{name}.png')
    fig.savefig(path, dpi=150, bbox_inches='tight')
    print(f'Saved: {path}')
    plt.close(fig)


def line_vs_C(metric, ylabel, title_prefix, fname_prefix):
    for sc in SCENARIOS:
        sub_sc = df[df['scenario'] == sc]
        ncols = 3
        nrows = int(np.ceil(len(K_VALUES) / ncols))
        fig, axes = plt.subplots(nrows, ncols, figsize=(16, 5 * nrows),
                                 constrained_layout=True)
        fig.suptitle(f'{title_prefix} — {sc}', fontsize=13)
        axes = np.array(axes).flatten()
        for ki, K in enumerate(K_VALUES):
            ax = axes[ki]
            sub = sub_sc[sub_sc['K'] == K]
            for pol in POLICIES:
                d = sub[sub['policy'] == pol].sort_values('capacity')
                if d.empty:
                    continue
                ax.plot(d['capacity'], d[metric], marker=MARKERS[pol],
                        color=COLORS[pol], label=POL_LABEL[pol], lw=2, ms=7)
            lb = lb_curve(sc, metric)
            if lb is not None:
                ax.plot(lb[0], lb[1], color='k', ls=':', marker='*', ms=9,
                        lw=1.8, label=LB_LABEL)
            ax.set_title(f'K = {K}', fontweight='bold')
            ax.set_xlabel('Capacidade C')
            ax.set_ylabel(ylabel)
            ax.set_xticks(C_VALUES)
            ax.yaxis.set_major_formatter(ticker.ScalarFormatter(useMathText=True))
            ax.ticklabel_format(axis='y', style='sci', scilimits=(0, 0))
            ax.grid(True, ls='--', alpha=0.4)
            if ki == 0:
                ax.legend(fontsize=10)
        for ax in axes[len(K_VALUES):]:
            ax.set_visible(False)
        save(fig, f'{fname_prefix}_{sc}')


def line_vs_K(metric, ylabel, title_prefix, fname_prefix):
    for sc in SCENARIOS:
        sub_sc = df[df['scenario'] == sc]
        fig, axes = plt.subplots(1, len(C_SEL), figsize=(6 * len(C_SEL), 5),
                                 constrained_layout=True)
        if len(C_SEL) == 1:
            axes = [axes]
        fig.suptitle(f'{title_prefix} — {sc}', fontsize=13)
        for ci, C in enumerate(C_SEL):
            ax = axes[ci]
            sub = sub_sc[sub_sc['capacity'] == C]
            for pol in POLICIES:
                d = sub[sub['policy'] == pol].sort_values('K')
                if d.empty:
                    continue
                ax.plot(d['K'], d[metric], marker=MARKERS[pol],
                        color=COLORS[pol], label=POL_LABEL[pol], lw=2, ms=7)
            lb = lb_curve(sc, metric)
            if lb is not None:
                cs, vs = lb
                idx = np.where(cs == C)[0]
                if len(idx):
                    ax.axhline(vs[idx[0]], color='k', ls=':', lw=1.8, label=LB_LABEL)
            ax.set_title(f'C = {C}', fontweight='bold')
            ax.set_xlabel('K (slots por época)')
            ax.set_ylabel(ylabel)
            ax.set_xticks(K_VALUES)
            ax.yaxis.set_major_formatter(ticker.ScalarFormatter(useMathText=True))
            ax.ticklabel_format(axis='y', style='sci', scilimits=(0, 0))
            ax.grid(True, ls='--', alpha=0.4)
            ax.legend(fontsize=10)
        save(fig, f'{fname_prefix}_{sc}')


line_vs_C('mean_r_sys', 'Risco médio R_sys',
          'Risco médio do sistema vs Capacidade', 'risk_mean_vs_C')
line_vs_C('peak_r_sys', 'Risco pico R_sys',
          'Risco pico do sistema vs Capacidade', 'risk_peak_vs_C')
line_vs_K('mean_r_sys', 'Risco médio R_sys',
          'Risco médio do sistema vs K', 'risk_mean_vs_K')
line_vs_K('peak_r_sys', 'Risco pico R_sys',
          'Risco pico do sistema vs K', 'risk_peak_vs_K')

print(f'\nPolíticas plotadas: {", ".join(POLICIES)}')
print(f'Plots salvos em: {OUT_DIR}')
