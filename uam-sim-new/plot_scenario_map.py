"""
plot_scenario_map.py
Mapa dos dois cenários SW: 2r_static e 2r_variavel
Coordenadas em [North, East] — eixos: East=X, North=Y
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.lines import Line2D

# ── Posições (East, North) ────────────────────────────────────────────────────
gnb = (5000, 0)          # gNB

# Rotas — (East_start, North_start, East_goal, North_goal)
routeA     = (1000, 1000, 9000, 1000)
routeB     = (12000, -500, 14000, -600)
routeBlong = (5500, 0, 14000, -700)

# Hotspots [North, East] → converter para (East, North)
hotspots_raw = [
  (-466.7,  11166.7),  (-616.2, 11154.4),  (-317.2, 11179.0),
  (-500.0,  11571.4),  (-649.5, 11559.1),  (-350.5, 11583.7),
  (-533.3,  11976.2),  (-682.8, 11963.9),  (-383.8, 11988.5),
  (-566.7,  12381.0),  (-716.2, 12368.6),  (-417.2, 12393.3),
  (-600.0,  12785.7),  (-749.5, 12773.4),  (-450.5, 12798.0),
  (-633.3,  13190.5),  (-782.8, 13178.2),  (-483.8, 13202.8),
  (-666.7,  13595.2),  (-816.2, 13582.9),  (-517.2, 13607.5),
  (-700.0,  14000.0),  (-849.5, 13987.7),  (-550.5, 14012.3),
]
hs_e = [h[1] for h in hotspots_raw]
hs_n = [h[0] for h in hotspots_raw]

# ── Figura: dois subplots lado a lado ────────────────────────────────────────
fig, axes = plt.subplots(1, 2, figsize=(16, 7), constrained_layout=True)
fig.suptitle('Cenários do Experimento SW Sweep', fontsize=14, fontweight='bold')

COLORS = {
    'routeA':     '#2196F3',   # azul
    'routeB':     '#FF5722',   # laranja-vermelho
    'routeBlong': '#E91E63',   # rosa/magenta
    'gnb':        '#4CAF50',   # verde
    'hotspot':    '#FF9800',   # âmbar
}

def draw_route(ax, r, color, label, lw=2.5):
    ax.annotate('', xy=(r[2], r[3]), xytext=(r[0], r[1]),
                arrowprops=dict(arrowstyle='->', color=color, lw=lw))
    ax.plot([r[0], r[2]], [r[1], r[3]], '-', color=color, lw=lw, zorder=3)
    # start/end markers
    ax.plot(r[0], r[1], 'o', color=color, ms=8, zorder=4)
    ax.plot(r[2], r[3], 's', color=color, ms=8, zorder=4)
    # label at midpoint
    mx, my = (r[0]+r[2])/2, (r[1]+r[3])/2
    ax.text(mx, my + 200, label, color=color, fontsize=9,
            ha='center', fontweight='bold', zorder=5)

def draw_gnb(ax):
    ax.plot(*gnb, '*', color=COLORS['gnb'], ms=18, zorder=6,
            markeredgecolor='black', markeredgewidth=0.5)
    ax.text(gnb[0], gnb[1] - 400, 'gNB\n(E=5000, N=0)',
            color=COLORS['gnb'], fontsize=8, ha='center', fontweight='bold')

def draw_hotspots(ax):
    ax.scatter(hs_e, hs_n, c=COLORS['hotspot'], s=40, marker='^',
               zorder=5, edgecolors='black', linewidths=0.4, label='Hotspots')

def add_snr_annotations(ax, scenario):
    """Marca zonas de SNR ao longo das rotas."""
    if scenario == 'static':
        # RouteA: SNR varia 17–33 dB (mais perto da gNB no meio)
        ax.annotate('SNR 17–33 dB', xy=(5000, 1000), fontsize=7.5,
                    ha='center', color=COLORS['routeA'],
                    xytext=(5000, 1600),
                    arrowprops=dict(arrowstyle='->', color=COLORS['routeA'], lw=1))
        # RouteB: SNR ~6–8 dB
        ax.annotate('SNR 6–8 dB', xy=(13000, -550), fontsize=7.5,
                    ha='center', color=COLORS['routeB'],
                    xytext=(11500, -1200),
                    arrowprops=dict(arrowstyle='->', color=COLORS['routeB'], lw=1))
    else:
        # RouteBlong: gradiente
        ax.annotate('SNR 30–35 dB', xy=(5500, 0), fontsize=7.5,
                    ha='left', color=COLORS['routeBlong'],
                    xytext=(6200, 600),
                    arrowprops=dict(arrowstyle='->', color=COLORS['routeBlong'], lw=1))
        ax.annotate('SNR 6–8 dB', xy=(14000, -700), fontsize=7.5,
                    ha='right', color=COLORS['routeBlong'],
                    xytext=(12500, -1500),
                    arrowprops=dict(arrowstyle='->', color=COLORS['routeBlong'], lw=1))

def format_ax(ax, title):
    ax.set_xlabel('East (m)', fontsize=11)
    ax.set_ylabel('North (m)', fontsize=11)
    ax.set_title(title, fontsize=12, fontweight='bold')
    ax.set_aspect('equal')
    ax.grid(True, ls='--', alpha=0.35)
    ax.axhline(0, color='gray', lw=0.5, ls=':')
    ax.axvline(0, color='gray', lw=0.5, ls=':')

# ── Cenário 1: 2r_static (RouteA + RouteB) ───────────────────────────────────
ax1 = axes[0]
draw_gnb(ax1)
draw_route(ax1, routeA, COLORS['routeA'],  'RouteA')
draw_route(ax1, routeB, COLORS['routeB'],  'RouteB')
add_snr_annotations(ax1, 'static')
format_ax(ax1, '2r_static  (RouteA + RouteB)')

legend_els = [
    Line2D([0],[0], color=COLORS['routeA'],  lw=2, label='RouteA  (SNR 17–33 dB)'),
    Line2D([0],[0], color=COLORS['routeB'],  lw=2, label='RouteB  (SNR 6–8 dB)'),
    Line2D([0],[0], marker='*', color=COLORS['gnb'], lw=0, ms=12,
           markeredgecolor='black', label='gNB'),
]
ax1.legend(handles=legend_els, loc='upper left', fontsize=8)

# ── Cenário 2: 2r_variavel (RouteA + RouteBlong) ─────────────────────────────
ax2 = axes[1]
draw_gnb(ax2)
draw_route(ax2, routeA,     COLORS['routeA'],     'RouteA')
draw_route(ax2, routeBlong, COLORS['routeBlong'],  'RouteBlong')
draw_hotspots(ax2)
add_snr_annotations(ax2, 'variavel')
format_ax(ax2, '2r_variavel  (RouteA + RouteBlong + hotspots)')

legend_els2 = [
    Line2D([0],[0], color=COLORS['routeA'],     lw=2, label='RouteA  (SNR 17–33 dB)'),
    Line2D([0],[0], color=COLORS['routeBlong'], lw=2, label='RouteBlong  (SNR 6–35 dB)'),
    Line2D([0],[0], marker='*', color=COLORS['gnb'], lw=0, ms=12,
           markeredgecolor='black', label='gNB'),
    Line2D([0],[0], marker='^', color=COLORS['hotspot'], lw=0, ms=8,
           markeredgecolor='black', label='Hotspots (24)'),
]
ax2.legend(handles=legend_els2, loc='upper left', fontsize=8)

# ── Salvar ────────────────────────────────────────────────────────────────────
out = 'scenario_map.png'
fig.savefig(out, dpi=150, bbox_inches='tight')
print(f'Saved: {out}')
plt.show()
