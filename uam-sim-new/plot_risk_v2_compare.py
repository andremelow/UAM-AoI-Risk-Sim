"""
plot_risk_v2_compare.py
Plots de LINHA de risco médio (mean_r_sys) e pico (peak_r_sys) comparando
4 políticas: RR, PF, Max-Risk (max-risk-sw) e Max-Risk-v2 (max-risk-v2).

Os dados de RR/PF/MR vêm do sweep GOP original (csv_export_mw_otimo_a23_gop)
e os de MR-v2 do sweep complementar (csv_export_mw_otimo_a23_gop_v2). Ambos
têm o mesmo schema de CSV, então são mesclados pela coluna `policy`.

Gera APENAS as linhas de risco médio e pico (vs C e vs K), por cenário.
Heatmaps e scatter do plot_mw_otimo_risk.py são omitidos de propósito.

Uso:
    python3 plot_risk_v2_compare.py
    python3 plot_risk_v2_compare.py --gop csv_export_mw_otimo_a23_gop \
                                    --v2  csv_export_mw_otimo_a23_gop_v2
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
parser.add_argument('--gop', default='csv_export_mw_otimo_a23_gop_final',
                    help='Dir com RR/PF/MR (sweep GOP FINAL — baseline canônico)')
parser.add_argument('--v2', default='csv_export_mw_otimo_a23_gop_v2',
                    help='Dir com MR-v2 (sweep complementar)')
parser.add_argument('--out', default=None,
                    help='Dir de saída dos plots (default: <v2>/plots)')
args = parser.parse_args()

GOP_DIR = os.path.join(SCRIPT_DIR, args.gop)
V2_DIR  = os.path.join(SCRIPT_DIR, args.v2)
OUT_DIR = args.out or os.path.join(V2_DIR, 'plots')
os.makedirs(OUT_DIR, exist_ok=True)


def load_dir(d):
    """Carrega sweep_summary_all.csv + quaisquer sweep_summary_*.csv do dir."""
    frames = []
    main = os.path.join(d, 'sweep_summary_all.csv')
    if os.path.exists(main):
        frames.append(pd.read_csv(main))
    for f in sorted(glob.glob(os.path.join(d, 'sweep_summary_*.csv'))):
        if os.path.basename(f) != 'sweep_summary_all.csv':
            frames.append(pd.read_csv(f))
    return frames


frames = load_dir(GOP_DIR) + load_dir(V2_DIR)
if not frames:
    raise SystemExit(f'Nenhum CSV encontrado em {GOP_DIR} ou {V2_DIR}')
df = pd.concat(frames, ignore_index=True).drop_duplicates(
    subset=['capacity', 'K', 'scenario', 'policy'])

# ── 4 políticas pedidas ────────────────────────────────────────────────────────
POLICIES  = ['round-robin-sw', 'pf-classic-sw', 'max-risk-sw', 'max-risk-v2']
_present  = set(df['policy'].unique())
missing   = [p for p in POLICIES if p not in _present]
if missing:
    print(f'[AVISO] políticas ausentes nos dados: {missing}')
POLICIES  = [p for p in POLICIES if p in _present]

POL_LABEL = {
    'round-robin-sw': 'RR',
    'pf-classic-sw':  'PF',
    'max-risk-sw':    'MR (max-risk-sw)',
    'max-risk-v2':    'MR-v2 (drift-derived)',
}
COLORS = {
    'round-robin-sw': '#9E9E9E',
    'pf-classic-sw':  '#2196F3',
    'max-risk-sw':    '#43A047',
    'max-risk-v2':    '#E53935',
}
MARKERS = {
    'round-robin-sw': 'o',
    'pf-classic-sw':  's',
    'max-risk-sw':    'D',
    'max-risk-v2':    '^',
}

K_VALUES  = sorted(df['K'].unique())
C_VALUES  = sorted(df['capacity'].unique())
SCENARIOS = sorted(df['scenario'].unique())
C_SEL     = [v for v in [4, 16, 48] if v in C_VALUES] or C_VALUES


def save(fig, name):
    path = os.path.join(OUT_DIR, f'{name}.png')
    fig.savefig(path, dpi=150, bbox_inches='tight')
    print(f'Saved: {path}')
    plt.close(fig)


def line_vs_C(metric, ylabel, title_prefix, fname_prefix):
    """Risco vs Capacidade C, um subplot por K — por cenário."""
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
    """Risco vs K, C representativos — por cenário."""
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
            ax.set_title(f'C = {C}', fontweight='bold')
            ax.set_xlabel('K (slots por época)')
            ax.set_ylabel(ylabel)
            ax.set_xticks(K_VALUES)
            ax.yaxis.set_major_formatter(ticker.ScalarFormatter(useMathText=True))
            ax.ticklabel_format(axis='y', style='sci', scilimits=(0, 0))
            ax.grid(True, ls='--', alpha=0.4)
            ax.legend(fontsize=10)
        save(fig, f'{fname_prefix}_{sc}')


# ── Apenas as linhas de risco médio e pico ──────────────────────────────────────
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
