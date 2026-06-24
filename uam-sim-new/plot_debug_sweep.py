"""
plot_debug_sweep.py
Plots for experiment_capacity_debug — RouteA (SNR≈38dB) vs RouteB (SNR≈7dB).
Generates 4 figures saved as PNG in csv_export/capacity_debug/.
"""
import csv, os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from matplotlib.colors import LogNorm

# ── data ──────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CSV_PATH   = os.path.join(SCRIPT_DIR, 'csv_export', 'capacity_debug',
                          'sweep_capacity_summary_all.csv')
OUT_DIR    = os.path.join(SCRIPT_DIR, 'csv_export', 'capacity_debug')

with open(CSV_PATH) as f:
    rows = list(csv.DictReader(f))

K_vals = sorted({int(r['K'])        for r in rows})
C_vals = sorted({int(r['capacity']) for r in rows})
POLS   = ['round-robin-sw', 'pf-classic-sw', 'max-weight-sw', 'aoi-sw']
LABELS = {'round-robin-sw': 'RR', 'pf-classic-sw': 'PF',
          'max-weight-sw':  'MW', 'aoi-sw':         'AoI'}
COLORS = {'round-robin-sw': '#4878CF', 'pf-classic-sw': '#6ACC65',
          'max-weight-sw':  '#D65F5F', 'aoi-sw':        '#B47CC7'}
MARKS  = {'round-robin-sw': 'o', 'pf-classic-sw': 's',
          'max-weight-sw':  '^', 'aoi-sw':        'D'}

def get(rows, K, C, pol, field):
    for r in rows:
        if int(r['K']) == K and int(r['capacity']) == C and r['policy'] == pol:
            v = r[field]
            return float(v) if v not in ('', 'nan') else np.nan
    return np.nan

plt.rcParams.update({
    'font.family': 'sans-serif', 'font.size': 10,
    'axes.titlesize': 11, 'axes.labelsize': 10,
    'legend.fontsize': 9,  'figure.dpi': 150,
    'axes.spines.top': False, 'axes.spines.right': False,
})

# ══════════════════════════════════════════════════════════════════════
# Fig 1 — Jhat vs C  (one sub-plot per K)
# ══════════════════════════════════════════════════════════════════════
fig1, axes = plt.subplots(2, 3, figsize=(13, 7), sharey=False)
fig1.suptitle('Jhat empirico vs Capacidade  (RouteA+RouteB, cenário debug)', fontweight='bold')

for ax, K in zip(axes.flat, K_vals):
    for pol in POLS:
        y = [get(rows, K, C, pol, 'Jhat_emp') for C in C_vals]
        ax.plot(C_vals, y, marker=MARKS[pol], color=COLORS[pol],
                label=LABELS[pol], linewidth=1.8, markersize=6)
    ax.set_title(f'K = {K}')
    ax.set_xlabel('Capacidade C')
    ax.set_ylabel('$\\hat{J}$ (slots²)')
    ax.set_yscale('log')
    ax.set_xticks(C_vals)
    ax.yaxis.set_major_formatter(ticker.LogFormatterSciNotation())
    ax.grid(True, which='both', linestyle=':', alpha=0.4)

handles, labels = axes.flat[0].get_legend_handles_labels()
fig1.legend(handles, labels, loc='upper center', ncol=4, framealpha=0.9,
            bbox_to_anchor=(0.5, 0.97))
fig1.tight_layout(rect=[0, 0, 1, 0.93])
out1 = os.path.join(OUT_DIR, 'fig1_jhat_vs_C.png')
fig1.savefig(out1, bbox_inches='tight')
print(f'Saved: {out1}')

# ══════════════════════════════════════════════════════════════════════
# Fig 2a — Heatmap: razão vs RR  (K × C)
# ══════════════════════════════════════════════════════════════════════
fig2, axes2 = plt.subplots(1, 3, figsize=(13, 4))
fig2.suptitle('Razão Jhat  (política / RR)  — < 1 = melhor que RR', fontweight='bold')

def ratio_heatmap(fig, axes, compare_pols, ref_pol, ref_label):
    for ax, pol in zip(axes, compare_pols):
        mat = np.zeros((len(K_vals), len(C_vals)))
        for ki, K in enumerate(K_vals):
            for ci, C in enumerate(C_vals):
                jref = get(rows, K, C, ref_pol, 'Jhat_emp')
                jp   = get(rows, K, C, pol, 'Jhat_emp')
                mat[ki, ci] = jp / jref if (jref > 0 and not np.isnan(jp)) else np.nan

        vmin, vmax = 0.3, 1.3
        im = ax.imshow(mat, aspect='auto', cmap='RdYlGn_r',
                       vmin=vmin, vmax=vmax, origin='lower')
        ax.set_xticks(range(len(C_vals))); ax.set_xticklabels(C_vals)
        ax.set_yticks(range(len(K_vals))); ax.set_yticklabels(K_vals)
        ax.set_xlabel('Capacidade C'); ax.set_ylabel('K (slots/época)')
        ax.set_title(LABELS[pol])

        for ki in range(len(K_vals)):
            for ci in range(len(C_vals)):
                v = mat[ki, ci]
                if not np.isnan(v):
                    color = 'white' if (v < 0.65 or v > 1.15) else 'black'
                    ax.text(ci, ki, f'{v:.2f}', ha='center', va='center',
                            fontsize=8, color=color, fontweight='bold')

        cb = fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
        cb.set_label(f'Jhat / Jhat_{ref_label}')
        best = np.nanargmin(mat, axis=None)
        bi, bj = np.unravel_index(best, mat.shape)
        ax.add_patch(plt.Rectangle((bj-0.5, bi-0.5), 1, 1,
                     fill=False, edgecolor='blue', linewidth=2))

ratio_heatmap(fig2, axes2,
              compare_pols=['pf-classic-sw', 'max-weight-sw', 'aoi-sw'],
              ref_pol='round-robin-sw', ref_label='RR')
fig2.tight_layout()
out2 = os.path.join(OUT_DIR, 'fig2_ratio_heatmap.png')
fig2.savefig(out2, bbox_inches='tight')
print(f'Saved: {out2}')

# ══════════════════════════════════════════════════════════════════════
# Fig 2b — Heatmap: razão vs PF  (K × C)
# ══════════════════════════════════════════════════════════════════════
fig2b, axes2b = plt.subplots(1, 3, figsize=(13, 4))
fig2b.suptitle('Razão Jhat  (política / PF)  — < 1 = melhor que PF', fontweight='bold')

ratio_heatmap(fig2b, axes2b,
              compare_pols=['round-robin-sw', 'max-weight-sw', 'aoi-sw'],
              ref_pol='pf-classic-sw', ref_label='PF')
fig2b.tight_layout()
out2b = os.path.join(OUT_DIR, 'fig2b_ratio_vs_pf.png')
fig2b.savefig(out2b, bbox_inches='tight')
print(f'Saved: {out2b}')

# ══════════════════════════════════════════════════════════════════════
# Fig 3 — AoI por rota: h1 (RouteA) vs h2 (RouteB)  para K=8
# ══════════════════════════════════════════════════════════════════════
K_focus = 8
fig3, axes3 = plt.subplots(1, 2, figsize=(11, 4.5), sharey=False)
fig3.suptitle(f'AoI por rota  (K={K_focus})  —  RouteA (SNR≈38dB)  vs  RouteB (SNR≈7dB)',
              fontweight='bold')

for ax, (field, route_lbl) in zip(axes3, [('mean_h1','RouteA'), ('mean_h2','RouteB')]):
    for pol in POLS:
        y = [get(rows, K_focus, C, pol, field) for C in C_vals]
        ax.plot(C_vals, y, marker=MARKS[pol], color=COLORS[pol],
                label=LABELS[pol], linewidth=1.8, markersize=6)
    ax.set_title(route_lbl)
    ax.set_xlabel('Capacidade C')
    ax.set_ylabel('$\\bar{h}$ média (slots)')
    ax.set_xticks(C_vals)
    ax.grid(True, linestyle=':', alpha=0.4)

handles, labels = axes3[0].get_legend_handles_labels()
fig3.legend(handles, labels, loc='upper center', ncol=4, framealpha=0.9,
            bbox_to_anchor=(0.5, 0.98))
fig3.tight_layout(rect=[0, 0, 1, 0.92])
out3 = os.path.join(OUT_DIR, 'fig3_aoi_per_route_K8.png')
fig3.savefig(out3, bbox_inches='tight')
print(f'Saved: {out3}')

# ══════════════════════════════════════════════════════════════════════
# Fig 4 — Winner map  (K × C)
# ══════════════════════════════════════════════════════════════════════
fig4, ax4 = plt.subplots(figsize=(7, 5))
fig4.suptitle('Política vencedora (menor Jhat) por K × C', fontweight='bold')

pol_idx = {p: i for i, p in enumerate(POLS)}
cmap4   = matplotlib.colors.ListedColormap([COLORS[p] for p in POLS])

win_mat = np.full((len(K_vals), len(C_vals)), np.nan)
for ki, K in enumerate(K_vals):
    for ci, C in enumerate(C_vals):
        jvals = [get(rows, K, C, p, 'Jhat_emp') for p in POLS]
        valid = [(j, i) for i, j in enumerate(jvals) if not np.isnan(j)]
        if valid:
            win_mat[ki, ci] = min(valid)[1]

im4 = ax4.imshow(win_mat, aspect='auto', cmap=cmap4,
                 vmin=-0.5, vmax=len(POLS)-0.5, origin='lower')
ax4.set_xticks(range(len(C_vals))); ax4.set_xticklabels(C_vals)
ax4.set_yticks(range(len(K_vals))); ax4.set_yticklabels(K_vals)
ax4.set_xlabel('Capacidade C'); ax4.set_ylabel('K (slots/época)')

for ki in range(len(K_vals)):
    for ci in range(len(C_vals)):
        idx = win_mat[ki, ci]
        if not np.isnan(idx):
            ax4.text(ci, ki, LABELS[POLS[int(idx)]],
                     ha='center', va='center', fontsize=9,
                     fontweight='bold', color='white')

cb4 = fig4.colorbar(im4, ax=ax4,
                    ticks=range(len(POLS)), fraction=0.046, pad=0.04)
cb4.ax.set_yticklabels([LABELS[p] for p in POLS])
cb4.set_label('Política')

fig4.tight_layout()
out4 = os.path.join(OUT_DIR, 'fig4_winner_map.png')
fig4.savefig(out4, bbox_inches='tight')
print(f'Saved: {out4}')

print('\nDone.')
