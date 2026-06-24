"""
plot_risk_analysis.py
Análise de risco (R_sys) do experimento capacity_debug.
Quatro figuras:
  fig_r1 — R_sys vs C (log, subplots por K)
  fig_r2 — Heatmap razão R_sys / PF
  fig_r3 — Scatter Jhat vs R_sys (surrogate x risco real)
  fig_r4 — Heatmap rho_emp (Jhat / J_LB) por política
"""
import csv, os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

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
MARKS  = {'round-robin-sw': 'o', 'pf-classic-sw': 's',
          'max-weight-sw':  '^', 'aoi-sw':        'D'}

def get(field, K, C, pol):
    for r in rows:
        if int(r['K']) == K and int(r['capacity']) == C and r['policy'] == pol:
            v = r[field]
            return float(v) if v not in ('', 'nan') else np.nan
    return np.nan

plt.rcParams.update({
    'font.family': 'sans-serif', 'font.size': 10,
    'axes.titlesize': 11, 'axes.labelsize': 10,
    'legend.fontsize': 9, 'figure.dpi': 150,
    'axes.spines.top': False, 'axes.spines.right': False,
})

# ══════════════════════════════════════════════════════════════════════
# Fig R1 — R_sys vs C  (subplots por K, log)
# ══════════════════════════════════════════════════════════════════════
fig_r1, axes = plt.subplots(2, 3, figsize=(13, 7), sharey=False)
fig_r1.suptitle('Risco do sistema R_sys  vs  Capacidade C  (RouteA SNR≈38dB + RouteB SNR≈7dB)',
                fontweight='bold')

for ax, K in zip(axes.flat, K_vals):
    for pol in POLS:
        y = [get('mean_r_sys', K, C, pol) for C in C_vals]
        ax.plot(C_vals, y, marker=MARKS[pol], color=COLORS[pol],
                label=LABELS[pol], linewidth=1.8, markersize=6)
    ax.set_title(f'K = {K}')
    ax.set_xlabel('Capacidade C')
    ax.set_ylabel('R_sys médio')
    ax.set_yscale('log')
    ax.set_xticks(C_vals)
    ax.yaxis.set_major_formatter(ticker.LogFormatterSciNotation())
    ax.grid(True, which='both', linestyle=':', alpha=0.4)

handles, labels_ = axes.flat[0].get_legend_handles_labels()
fig_r1.legend(handles, labels_, loc='upper center', ncol=4,
              framealpha=0.9, bbox_to_anchor=(0.5, 0.97))
fig_r1.tight_layout(rect=[0, 0, 1, 0.93])
out = os.path.join(OUT_DIR, 'fig_r1_rsys_vs_C.png')
fig_r1.savefig(out, bbox_inches='tight')
print(f'Saved: {out}')

# ══════════════════════════════════════════════════════════════════════
# Fig R2 — Heatmap razão R_sys / PF  (3 painéis: RR, MW, AoI)
# ══════════════════════════════════════════════════════════════════════
fig_r2, axes2 = plt.subplots(1, 3, figsize=(13, 4))
fig_r2.suptitle('Razão R_sys  (política / PF)  —  verde = menor risco que PF',
                fontweight='bold')

for ax, pol in zip(axes2, ['round-robin-sw', 'max-weight-sw', 'aoi-sw']):
    mat = np.full((len(K_vals), len(C_vals)), np.nan)
    for ki, K in enumerate(K_vals):
        for ci, C in enumerate(C_vals):
            jpf = get('mean_r_sys', K, C, 'pf-classic-sw')
            jp  = get('mean_r_sys', K, C, pol)
            if jpf > 0 and not np.isnan(jp):
                mat[ki, ci] = jp / jpf

    im = ax.imshow(mat, aspect='auto', cmap='RdYlGn_r',
                   vmin=0.4, vmax=1.4, origin='lower')
    ax.set_xticks(range(len(C_vals))); ax.set_xticklabels(C_vals)
    ax.set_yticks(range(len(K_vals))); ax.set_yticklabels(K_vals)
    ax.set_xlabel('Capacidade C'); ax.set_ylabel('K (slots/época)')
    ax.set_title(LABELS[pol])

    for ki in range(len(K_vals)):
        for ci in range(len(C_vals)):
            v = mat[ki, ci]
            if not np.isnan(v):
                color = 'white' if (v < 0.65 or v > 1.2) else 'black'
                ax.text(ci, ki, f'{v:.2f}', ha='center', va='center',
                        fontsize=8, color=color, fontweight='bold')

    cb = fig_r2.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    cb.set_label('R_sys / R_sys_PF')

fig_r2.tight_layout()
out = os.path.join(OUT_DIR, 'fig_r2_rsys_ratio_vs_pf.png')
fig_r2.savefig(out, bbox_inches='tight')
print(f'Saved: {out}')

# ══════════════════════════════════════════════════════════════════════
# Fig R3 — Scatter Jhat vs R_sys  (surrogate x risco real)
# ══════════════════════════════════════════════════════════════════════
fig_r3, axes3 = plt.subplots(1, 2, figsize=(12, 5))
fig_r3.suptitle('Surrogate Jhat  vs  Risco real R_sys  —  qualidade do surrogate',
                fontweight='bold')

# left: all points colored by policy
ax = axes3[0]
for pol in POLS:
    xv, yv = [], []
    for r in rows:
        if r['policy'] == pol:
            xv.append(float(r['Jhat_emp']))
            yv.append(float(r['mean_r_sys']))
    ax.scatter(xv, yv, color=COLORS[pol], label=LABELS[pol],
               marker=MARKS[pol], s=40, alpha=0.75, edgecolors='none')

# diagonal guide: if Jhat == Rsys
lim_min = min(float(r['Jhat_emp']) for r in rows)
lim_max = max(float(r['Jhat_emp']) for r in rows)
ax.plot([lim_min, lim_max], [lim_min, lim_max],
        'k--', linewidth=0.8, alpha=0.4, label='Jhat = R_sys')
ax.set_xscale('log'); ax.set_yscale('log')
ax.set_xlabel('$\\hat{J}$ (surrogate)'); ax.set_ylabel('R_sys (risco real)')
ax.set_title('Todos os pontos (todos K, C)')
ax.legend(markerscale=1.2)
ax.grid(True, which='both', linestyle=':', alpha=0.35)

# right: same but colored by C (tamanho ~ K), só MW e PF para clareza
ax2 = axes3[1]
cmap_c = plt.cm.plasma
c_norm = matplotlib.colors.LogNorm(vmin=min(C_vals), vmax=max(C_vals))

for pol, marker in [('pf-classic-sw', 's'), ('max-weight-sw', '^')]:
    for r in rows:
        if r['policy'] != pol: continue
        C = int(r['capacity'])
        ax2.scatter(float(r['Jhat_emp']), float(r['mean_r_sys']),
                    marker=marker, s=55,
                    color=cmap_c(c_norm(C)),
                    edgecolors=COLORS[pol], linewidths=0.8, alpha=0.85)

# legend for policies
from matplotlib.lines import Line2D
leg_pols = [Line2D([0],[0], marker='s', color='gray', linestyle='None',
                   markersize=8, label='PF'),
            Line2D([0],[0], marker='^', color='gray', linestyle='None',
                   markersize=8, label='MW')]
ax2.legend(handles=leg_pols, title='Política', loc='upper left')

sm = plt.cm.ScalarMappable(cmap=cmap_c, norm=c_norm)
sm.set_array([])
cb3 = fig_r3.colorbar(sm, ax=ax2, fraction=0.046, pad=0.04)
cb3.set_label('Capacidade C')
cb3.set_ticks(C_vals); cb3.ax.set_yticklabels(C_vals)

diag_min = min(float(r['Jhat_emp']) for r in rows if r['policy'] in ('pf-classic-sw','max-weight-sw'))
diag_max = max(float(r['Jhat_emp']) for r in rows if r['policy'] in ('pf-classic-sw','max-weight-sw'))
ax2.plot([diag_min, diag_max], [diag_min, diag_max],
         'k--', linewidth=0.8, alpha=0.4)
ax2.set_xscale('log'); ax2.set_yscale('log')
ax2.set_xlabel('$\\hat{J}$ (surrogate)'); ax2.set_ylabel('R_sys (risco real)')
ax2.set_title('PF vs MW — colorido por C')
ax2.grid(True, which='both', linestyle=':', alpha=0.35)

fig_r3.tight_layout()
out = os.path.join(OUT_DIR, 'fig_r3_jhat_vs_rsys_scatter.png')
fig_r3.savefig(out, bbox_inches='tight')
print(f'Saved: {out}')

# ══════════════════════════════════════════════════════════════════════
# Fig R4 — Heatmap rho_emp  por política  (K × C)
# ══════════════════════════════════════════════════════════════════════
fig_r4, axes4 = plt.subplots(1, 4, figsize=(16, 4))
fig_r4.suptitle('rho_emp = Jhat / J_LB  por política  —  1.0 = ótimo teórico',
                fontweight='bold')

for ax, pol in zip(axes4, POLS):
    mat = np.full((len(K_vals), len(C_vals)), np.nan)
    for ki, K in enumerate(K_vals):
        for ci, C in enumerate(C_vals):
            mat[ki, ci] = get('rho_emp', K, C, pol)

    # clip display at 5.0 for readability (some K=32 values blow up)
    mat_disp = np.clip(mat, 0, 5.0)
    im = ax.imshow(mat_disp, aspect='auto', cmap='YlOrRd',
                   vmin=0, vmax=5.0, origin='lower')
    ax.set_xticks(range(len(C_vals))); ax.set_xticklabels(C_vals)
    ax.set_yticks(range(len(K_vals))); ax.set_yticklabels(K_vals)
    ax.set_xlabel('Capacidade C'); ax.set_ylabel('K')
    ax.set_title(LABELS[pol])

    for ki in range(len(K_vals)):
        for ci in range(len(C_vals)):
            v = mat[ki, ci]
            if not np.isnan(v):
                disp = f'{v:.2f}' if v < 10 else f'{v:.0f}'
                color = 'white' if mat_disp[ki, ci] > 3.5 else 'black'
                ax.text(ci, ki, disp, ha='center', va='center',
                        fontsize=7.5, color=color, fontweight='bold')

    cb = fig_r4.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    cb.set_label('rho (clipado em 5)')

fig_r4.tight_layout()
out = os.path.join(OUT_DIR, 'fig_r4_rho_heatmap.png')
fig_r4.savefig(out, bbox_inches='tight')
print(f'Saved: {out}')

print('\nDone.')
