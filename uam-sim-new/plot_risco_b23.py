"""
plot_risco_b23.py
Plots de risco (mean_r_sys e peak_r_sys) — experimento mw_otimo_b23
(2/3 drones em RouteBlong, apenas cenário 2r_variavel).
Saída: csv_export_mw_otimo_b23/risco/
"""

import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CSV    = os.path.join(SCRIPT_DIR, 'csv_export_mw_otimo_b23', 'sweep_summary_all.csv')
OUT    = os.path.join(SCRIPT_DIR, 'csv_export_mw_otimo_b23', 'risco')
os.makedirs(OUT, exist_ok=True)

df = pd.read_csv(CSV)

POLICIES  = ['round-robin-sw', 'pf-classic-sw', 'mw-sw-otimo']
POL_LABEL = {'round-robin-sw': 'Round-Robin',
             'pf-classic-sw':  'PF-Classic',
             'mw-sw-otimo':    'MW-Ótimo (ψ≤2)'}
COLORS    = {'round-robin-sw': '#9E9E9E',
             'pf-classic-sw':  '#2196F3',
             'mw-sw-otimo':    '#E53935'}
MARKERS   = {'round-robin-sw': 'o', 'pf-classic-sw': 's', 'mw-sw-otimo': '^'}

K_ALL = sorted(df['K'].unique())
C_ALL = sorted(df['capacity'].unique())
C_SEL = [v for v in [4, 16, 48] if v in C_ALL]


def save(fig, name):
    p = os.path.join(OUT, f'{name}.png')
    fig.savefig(p, dpi=150, bbox_inches='tight')
    print(f'Saved: {p}')
    plt.close(fig)


# ── 1. Risco médio vs C  (um subplot por K) ───────────────────────────────────
for metric, ylabel, title, fname in [
    ('mean_r_sys', 'Risco médio R_sys',  'Risco médio do sistema vs Capacidade', 'risk_mean_vs_C'),
    ('peak_r_sys', 'Risco pico R_sys',   'Risco pico do sistema vs Capacidade',  'risk_peak_vs_C'),
]:
    ncols = 3
    nrows = int(np.ceil(len(K_ALL) / ncols))
    fig, axes = plt.subplots(nrows, ncols, figsize=(16, 5*nrows), constrained_layout=True)
    fig.suptitle(f'{title} — 2r_variavel (2/3 drones RouteBlong)', fontsize=13)
    axes = np.array(axes).flatten()

    for ki, K in enumerate(K_ALL):
        ax = axes[ki]
        sub = df[df['K'] == K]
        for pol in POLICIES:
            d = sub[sub['policy'] == pol].sort_values('capacity')
            ax.plot(d['capacity'], d[metric], marker=MARKERS[pol],
                    color=COLORS[pol], label=POL_LABEL[pol], lw=2, ms=7)
        ax.set_title(f'K = {K}', fontweight='bold')
        ax.set_xlabel('Capacidade C')
        ax.set_ylabel(ylabel)
        ax.set_xticks(C_ALL)
        ax.yaxis.set_major_formatter(ticker.ScalarFormatter(useMathText=True))
        ax.ticklabel_format(axis='y', style='sci', scilimits=(0, 0))
        ax.grid(True, ls='--', alpha=0.4)
        if ki == 0:
            ax.legend(fontsize=9)

    for ax in axes[len(K_ALL):]:
        ax.set_visible(False)
    save(fig, fname)


# ── 2. Risco médio vs K  (C representativos) ──────────────────────────────────
for metric, ylabel, title, fname in [
    ('mean_r_sys', 'Risco médio R_sys', 'Risco médio vs K', 'risk_mean_vs_K'),
    ('peak_r_sys', 'Risco pico R_sys',  'Risco pico vs K',  'risk_peak_vs_K'),
]:
    fig, axes = plt.subplots(1, len(C_SEL), figsize=(6*len(C_SEL), 5), constrained_layout=True)
    if len(C_SEL) == 1:
        axes = [axes]
    fig.suptitle(f'{title} — 2r_variavel (2/3 drones RouteBlong)', fontsize=13)

    for ci, C in enumerate(C_SEL):
        ax = axes[ci]
        sub = df[df['capacity'] == C]
        for pol in POLICIES:
            d = sub[sub['policy'] == pol].sort_values('K')
            ax.plot(d['K'], d[metric], marker=MARKERS[pol],
                    color=COLORS[pol], label=POL_LABEL[pol], lw=2, ms=7)
        ax.set_title(f'C = {C}', fontweight='bold')
        ax.set_xlabel('K (slots por época)')
        ax.set_ylabel(ylabel)
        ax.set_xticks(K_ALL)
        ax.yaxis.set_major_formatter(ticker.ScalarFormatter(useMathText=True))
        ax.ticklabel_format(axis='y', style='sci', scilimits=(0, 0))
        ax.grid(True, ls='--', alpha=0.4)
        ax.legend(fontsize=9)

    save(fig, fname)


# ── 3. Heatmap ganho vs Round-Robin (K × C) ───────────────────────────────────
for metric, title, fname in [
    ('mean_r_sys', 'Redução de risco médio vs RR',  'heatmap_risk_mean'),
    ('peak_r_sys', 'Redução de risco pico vs RR',   'heatmap_risk_peak'),
]:
    fig, axes = plt.subplots(1, 2, figsize=(14, 5), constrained_layout=True)
    fig.suptitle(f'{title} — 2r_variavel (2/3 drones RouteBlong)', fontsize=13)

    for ci, pol in enumerate(['pf-classic-sw', 'mw-sw-otimo']):
        ax = axes[ci]
        mat = np.full((len(K_ALL), len(C_ALL)), np.nan)
        for ki, K in enumerate(K_ALL):
            for cj, C in enumerate(C_ALL):
                rr = df[(df['K']==K) & (df['capacity']==C) &
                        (df['policy']=='round-robin-sw')][metric]
                pp = df[(df['K']==K) & (df['capacity']==C) &
                        (df['policy']==pol)][metric]
                if len(rr) and len(pp) and rr.values[0] > 0:
                    mat[ki, cj] = (rr.values[0] - pp.values[0]) / rr.values[0] * 100

        vmax = max(np.nanpercentile(np.abs(mat), 95), 5)
        im = ax.imshow(mat, cmap='RdYlGn', aspect='auto', vmin=-vmax, vmax=vmax)
        ax.set_xticks(range(len(C_ALL))); ax.set_xticklabels(C_ALL)
        ax.set_yticks(range(len(K_ALL))); ax.set_yticklabels(K_ALL)
        ax.set_xlabel('Capacidade C'); ax.set_ylabel('K (slots/época)')
        ax.set_title(f'{POL_LABEL[pol]} vs RR', fontweight='bold')
        for ki in range(len(K_ALL)):
            for cj in range(len(C_ALL)):
                v = mat[ki, cj]
                if not np.isnan(v):
                    ax.text(cj, ki, f'{v:+.1f}%', ha='center', va='center',
                            fontsize=7.5, color='black')
        plt.colorbar(im, ax=ax, label='Redução de risco (%)\n(+ = melhor que RR)')

    save(fig, fname)


# ── 4. Scatter risco médio vs pico  (todos K, C representativos) ──────────────
fig, axes = plt.subplots(1, len(C_SEL), figsize=(6*len(C_SEL), 5), constrained_layout=True)
if len(C_SEL) == 1:
    axes = [axes]
fig.suptitle('Risco médio vs pico — 2r_variavel (2/3 drones RouteBlong)', fontsize=13)

for ci, C in enumerate(C_SEL):
    ax = axes[ci]
    sub = df[df['capacity'] == C]
    for pol in POLICIES:
        d = sub[sub['policy'] == pol]
        sc = ax.scatter(d['mean_r_sys'], d['peak_r_sys'],
                        marker=MARKERS[pol], color=COLORS[pol],
                        label=POL_LABEL[pol], s=70, alpha=0.8,
                        edgecolors='white', linewidths=0.5)
    ax.set_title(f'C = {C}  (todos K)', fontweight='bold')
    ax.set_xlabel('Risco médio R_sys')
    ax.set_ylabel('Risco pico R_sys')
    ax.xaxis.set_major_formatter(ticker.ScalarFormatter(useMathText=True))
    ax.yaxis.set_major_formatter(ticker.ScalarFormatter(useMathText=True))
    ax.ticklabel_format(axis='both', style='sci', scilimits=(0, 0))
    ax.grid(True, ls='--', alpha=0.4)
    ax.legend(fontsize=9)

save(fig, 'risk_scatter')


# ── 5. Linha de sobreposição de incerteza  (h1 médio vs C, com limiar) ────────
# limiar para RouteBlong com 2/3C drones: separação = 8529/(2C/3) = 12794/C
# h_max = (sep - 20) * 7/20
thresh_h = {C: max(0, (8529 / round(2*C/3) - 20) * 7 / 20) for C in C_ALL}

fig, axes = plt.subplots(2, 3, figsize=(16, 10), constrained_layout=True)
fig.suptitle('AoI médio C2 (h₁) vs Capacidade — com limiar de sobreposição de incerteza\n'
             '2r_variavel — 2/3 drones RouteBlong', fontsize=12)
axes = axes.flatten()

for ki, K in enumerate(K_ALL):
    ax = axes[ki]
    sub = df[df['K'] == K]
    for pol in POLICIES:
        d = sub[sub['policy'] == pol].sort_values('capacity')
        ax.plot(d['capacity'], d['mean_h1'], marker=MARKERS[pol],
                color=COLORS[pol], label=POL_LABEL[pol], lw=2, ms=7)

    # Limiar de sobreposição (RouteBlong — rota mais crítica)
    th_vals = [thresh_h[C] for C in C_ALL]
    ax.plot(C_ALL, th_vals, 'k--', lw=1.5, label='h₁ max (sem sobreposição)')
    ax.fill_between(C_ALL, th_vals,
                    [max(df['mean_h1'].max(), max(th_vals))*1.1]*len(C_ALL),
                    alpha=0.08, color='red', label='_nolegend_')

    ax.set_title(f'K = {K}', fontweight='bold')
    ax.set_xlabel('Capacidade C')
    ax.set_ylabel('mean h₁ (slots)')
    ax.set_xticks(C_ALL)
    ax.grid(True, ls='--', alpha=0.4)
    if ki == 0:
        ax.legend(fontsize=8)

save(fig, 'h1_vs_C_com_limiar')

print(f'\nTodos os plots em: {OUT}')
