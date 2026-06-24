"""
plot_variavel_analysis.py
Análise comparativa: cenário 2r_static vs 2r_variavel
Foco: exposição da fraqueza do PF em canal com gradiente espacial.

Figuras:
  fig_v1 — Jhat normalizado por RR: static vs variavel (barras agrupadas)
  fig_v2 — Heatmap razão MW/PF nos dois cenários (vantagem relativa)
  fig_v3 — Delta-vantagem MW sobre PF: variavel minus static  (K×C heatmap)
  fig_v4 — Razão Jhat_variavel / Jhat_static por política (quem degrada mais?)
"""
import csv, os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

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
          'max-weight-sw':  'MW', 'aoi-sw':        'AoI'}
COLORS = {'round-robin-sw': '#4878CF', 'pf-classic-sw': '#6ACC65',
          'max-weight-sw':  '#D65F5F', 'aoi-sw':        '#B47CC7'}
SCEN_LABELS = {'2r_static': 'Estático\n(SNR≈38dB)', '2r_variavel': 'Gradiente\n(SNR 35→7dB)'}

def get(field, K, C, pol, sc):
    for r in rows:
        if (int(r['K']) == K and int(r['capacity']) == C
                and r['policy'] == pol and r['scenario'] == sc):
            v = r[field]
            return float(v) if v not in ('', 'nan') else np.nan
    return np.nan

plt.rcParams.update({
    'font.family': 'sans-serif', 'font.size': 10,
    'axes.titlesize': 11, 'axes.labelsize': 10,
    'legend.fontsize': 9,  'figure.dpi': 150,
    'axes.spines.top': False, 'axes.spines.right': False,
})

# ══════════════════════════════════════════════════════════════════════════════
# Fig V1 — Jhat normalizado por RR: barras agrupadas static vs variavel
#           Um subplot por (K,C) selecionado  →  grade 2×4
# ══════════════════════════════════════════════════════════════════════════════
selected = [(8, 8), (8, 16), (16, 8), (16, 16),
            (32, 8), (32, 16), (8, 32), (16, 32)]

fig_v1, axes = plt.subplots(2, 4, figsize=(16, 7), sharey=False)
fig_v1.suptitle(
    'Jhat / Jhat_RR  por política — Estático vs Gradiente de canal\n'
    '(barras mais baixas = melhor; MW deve ganhar mais vantagem no gradiente)',
    fontweight='bold')

BAR_W  = 0.18
XPOS   = np.arange(len(POLS))
SCENS  = ['2r_static', '2r_variavel']
HATCH  = ['', '///']
EDGE   = ['#333', '#666']

for ax, (K, C) in zip(axes.flat, selected):
    rr_s = get('Jhat_emp', K, C, 'round-robin-sw', '2r_static')
    rr_v = get('Jhat_emp', K, C, 'round-robin-sw', '2r_variavel')

    for si, (sc, ref) in enumerate(zip(SCENS, [rr_s, rr_v])):
        for pi, pol in enumerate(POLS):
            v  = get('Jhat_emp', K, C, pol, sc)
            xo = XPOS[pi] + (si - 0.5) * BAR_W
            bar = ax.bar(xo, v / ref, width=BAR_W,
                         color=COLORS[pol], alpha=0.85 if si == 0 else 0.5,
                         hatch=HATCH[si], edgecolor=EDGE[si], linewidth=0.6)

    ax.axhline(1.0, color='k', linewidth=0.6, linestyle='--', alpha=0.4)
    ax.set_title(f'K={K}, C={C}')
    ax.set_xticks(XPOS)
    ax.set_xticklabels([LABELS[p] for p in POLS])
    ax.set_ylabel('Jhat / Jhat_RR')
    ax.grid(True, axis='y', linestyle=':', alpha=0.35)

# Legend
from matplotlib.patches import Patch
legend_elems = [
    Patch(facecolor='gray', alpha=0.85, label='Estático (SNR≈38dB)'),
    Patch(facecolor='gray', alpha=0.50, hatch='///', label='Gradiente (SNR 35→7dB)'),
]
fig_v1.legend(handles=legend_elems, loc='upper center', ncol=2,
              bbox_to_anchor=(0.5, 0.97), framealpha=0.9)
fig_v1.tight_layout(rect=[0, 0, 1, 0.92])
out = os.path.join(OUT_DIR, 'fig_v1_jhat_norm_static_vs_variavel.png')
fig_v1.savefig(out, bbox_inches='tight')
print(f'Saved: {out}')

# ══════════════════════════════════════════════════════════════════════════════
# Fig V2 — Heatmap razão MW/PF  nos dois cenários
#           Azul = MW melhor, Verde = PF melhor
# ══════════════════════════════════════════════════════════════════════════════
fig_v2, axes2 = plt.subplots(1, 2, figsize=(12, 5))
fig_v2.suptitle(
    'Razão Jhat_MW / Jhat_PF  —  valores < 1 indicam MW melhor\n'
    'O gradiente de canal deve ampliar a vantagem do MW sobre o PF',
    fontweight='bold')

for ax, sc in zip(axes2, SCENS):
    mat = np.full((len(K_vals), len(C_vals)), np.nan)
    for ki, K in enumerate(K_vals):
        for ci, C in enumerate(C_vals):
            jmw = get('Jhat_emp', K, C, 'max-weight-sw', sc)
            jpf = get('Jhat_emp', K, C, 'pf-classic-sw', sc)
            if jpf > 0 and not np.isnan(jmw):
                mat[ki, ci] = jmw / jpf

    # center colormap at 1.0: < 1 = blue (MW melhor), > 1 = red (PF melhor)
    vmax = max(np.nanmax(np.abs(mat - 1.0)), 0.05) + 1.0
    vmin = 2.0 - vmax
    im = ax.imshow(mat, aspect='auto', cmap='RdBu',
                   vmin=vmin, vmax=vmax, origin='lower')
    ax.set_xticks(range(len(C_vals))); ax.set_xticklabels(C_vals)
    ax.set_yticks(range(len(K_vals))); ax.set_yticklabels(K_vals)
    ax.set_xlabel('Capacidade C'); ax.set_ylabel('K (slots/época)')
    ax.set_title(SCEN_LABELS[sc].replace('\n', ' '))

    for ki in range(len(K_vals)):
        for ci in range(len(C_vals)):
            v = mat[ki, ci]
            if not np.isnan(v):
                txt_color = 'white' if abs(v - 1.0) > 0.25 else 'black'
                ax.text(ci, ki, f'{v:.2f}', ha='center', va='center',
                        fontsize=9, color=txt_color, fontweight='bold')

    cb = fig_v2.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    cb.set_label('MW / PF  (< 1 = MW melhor)')
    cb.ax.axhline(1.0, color='k', linewidth=1.0)

fig_v2.tight_layout()
out = os.path.join(OUT_DIR, 'fig_v2_mw_pf_ratio_heatmap.png')
fig_v2.savefig(out, bbox_inches='tight')
print(f'Saved: {out}')

# ══════════════════════════════════════════════════════════════════════════════
# Fig V3 — Delta-vantagem MW sobre PF: (ratio_variavel - ratio_static)
#           Positivo = MW ganhou mais vantagem no cenário de gradiente
# ══════════════════════════════════════════════════════════════════════════════
fig_v3, axes3 = plt.subplots(1, 2, figsize=(12, 5))
fig_v3.suptitle(
    'Ganho incremental de MW sobre PF ao mudar para canal com gradiente\n'
    'Δ = (MW/PF)_static − (MW/PF)_variavel  (positivo = MW ganha mais no gradiente)',
    fontweight='bold')

# left: heatmap delta K×C
ax = axes3[0]
mat_s = np.full((len(K_vals), len(C_vals)), np.nan)
mat_v = np.full((len(K_vals), len(C_vals)), np.nan)
for ki, K in enumerate(K_vals):
    for ci, C in enumerate(C_vals):
        for mat, sc in [(mat_s, '2r_static'), (mat_v, '2r_variavel')]:
            jmw = get('Jhat_emp', K, C, 'max-weight-sw', sc)
            jpf = get('Jhat_emp', K, C, 'pf-classic-sw', sc)
            if jpf > 0:
                mat[ki, ci] = jmw / jpf

delta = mat_s - mat_v   # positive = MW gained advantage in variavel

im = ax.imshow(delta, aspect='auto', cmap='PuOr_r',
               vmin=-0.3, vmax=0.3, origin='lower')
ax.set_xticks(range(len(C_vals))); ax.set_xticklabels(C_vals)
ax.set_yticks(range(len(K_vals))); ax.set_yticklabels(K_vals)
ax.set_xlabel('Capacidade C'); ax.set_ylabel('K')
ax.set_title('Δ razão MW/PF  (roxo = MW mais vantajoso no gradiente)')

for ki in range(len(K_vals)):
    for ci in range(len(C_vals)):
        v = delta[ki, ci]
        if not np.isnan(v):
            txt_color = 'white' if abs(v) > 0.18 else 'black'
            ax.text(ci, ki, f'{v:+.2f}', ha='center', va='center',
                    fontsize=8, color=txt_color, fontweight='bold')

cb = fig_v3.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
cb.set_label('Δ(MW/PF)  positivo = MW mais forte no gradiente')

# right: line plot — MW/PF ratio vs C for K=8,16 in both scenarios
ax2 = axes3[1]
for K, ls in [(8, '-'), (16, '--'), (32, ':')]:
    for sc, color in [('2r_static', '#2166AC'), ('2r_variavel', '#D6604D')]:
        ratios = []
        for C in C_vals:
            jmw = get('Jhat_emp', K, C, 'max-weight-sw', sc)
            jpf = get('Jhat_emp', K, C, 'pf-classic-sw', sc)
            ratios.append(jmw / jpf if jpf > 0 else np.nan)
        label = f'K={K} {"grad" if sc=="2r_variavel" else "stat"}'
        ax2.plot(C_vals, ratios, linestyle=ls, color=color,
                 marker='o', markersize=5, linewidth=1.8, label=label)

ax2.axhline(1.0, color='k', linewidth=0.7, linestyle='--', alpha=0.5)
ax2.set_xlabel('Capacidade C')
ax2.set_ylabel('Jhat_MW / Jhat_PF')
ax2.set_title('MW/PF vs C — estático (azul) vs gradiente (vermelho)')
ax2.set_xticks(C_vals)
ax2.legend(fontsize=8, ncol=2)
ax2.grid(True, linestyle=':', alpha=0.35)

fig_v3.tight_layout()
out = os.path.join(OUT_DIR, 'fig_v3_mw_advantage_delta.png')
fig_v3.savefig(out, bbox_inches='tight')
print(f'Saved: {out}')

# ══════════════════════════════════════════════════════════════════════════════
# Fig V4 — Razão Jhat_variavel / Jhat_static por política
#           Quem degrada mais ao mudar para canal heterogêneo?
#           PF deve degradar mais (relação sobe ou mantém, já que PF perde share)
# ══════════════════════════════════════════════════════════════════════════════
fig_v4, axes4 = plt.subplots(1, 4, figsize=(16, 4))
fig_v4.suptitle(
    'Razão Jhat_variavel / Jhat_static por política  —  > 1 = pior no gradiente\n'
    'PF deve degradar mais (perde eficiência no canal heterogêneo)',
    fontweight='bold')

for ax, pol in zip(axes4, POLS):
    mat = np.full((len(K_vals), len(C_vals)), np.nan)
    for ki, K in enumerate(K_vals):
        for ci, C in enumerate(C_vals):
            jv = get('Jhat_emp', K, C, pol, '2r_variavel')
            js = get('Jhat_emp', K, C, pol, '2r_static')
            if js > 0 and not np.isnan(jv):
                mat[ki, ci] = jv / js

    im = ax.imshow(mat, aspect='auto', cmap='RdYlGn_r',
                   vmin=0.5, vmax=1.5, origin='lower')
    ax.set_xticks(range(len(C_vals))); ax.set_xticklabels(C_vals)
    ax.set_yticks(range(len(K_vals))); ax.set_yticklabels(K_vals)
    ax.set_xlabel('C'); ax.set_ylabel('K')
    ax.set_title(LABELS[pol])

    for ki in range(len(K_vals)):
        for ci in range(len(C_vals)):
            v = mat[ki, ci]
            if not np.isnan(v):
                txt_color = 'white' if (v < 0.65 or v > 1.35) else 'black'
                ax.text(ci, ki, f'{v:.2f}', ha='center', va='center',
                        fontsize=8, color=txt_color, fontweight='bold')

    cb = fig_v4.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    cb.set_label('Jhat_var / Jhat_stat  (> 1 = pior)')
    cb.ax.axhline(1.0, color='k', linewidth=0.8)

fig_v4.tight_layout()
out = os.path.join(OUT_DIR, 'fig_v4_degradation_by_policy.png')
fig_v4.savefig(out, bbox_inches='tight')
print(f'Saved: {out}')

print('\nDone.')
