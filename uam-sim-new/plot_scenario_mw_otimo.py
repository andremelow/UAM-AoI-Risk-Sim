"""
plot_scenario_mw_otimo.py
Visualização dos cenários 2r_static e 2r_variavel:
  - Posição da gNB
  - Rotas
  - Distribuição de SNR (modelo UMa LOS, 3GPP TR 38.901)
  - Distribuição de risco (hotspots Gaussianos)
"""

import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.patheffects as pe
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.gridspec import GridSpec

# ── Parâmetros do sistema (build_scenario_variavel.m) ─────────────────────────
BS_EAST   = 5000.0    # m
BS_NORTH  = 0.0       # m
H_BS      = 30.0      # m (alt = -30 → 30 m acima do solo)
H_DRONE   = 60.0      # m (Down = -60 → 60 m)

FC_GHZ    = 3.5
P_TX_DBM  = 23.0
G_BS_DBI  = 15.0
NOISE_DBM = -174 + 10 * np.log10(20e6)   # thermal noise
NF_DB     = 7.0
SNR_LINK_BUDGET = P_TX_DBM + G_BS_DBI - NOISE_DBM - NF_DB  # = 132 dB

THRESH_SNR = 5.0   # dB (link threshold)

# ── Modelo UMa LOS (3GPP TR 38.901 Tabela 7.4.1-1, d_2D ≤ d_BP) ─────────────
def snr_at(north, east):
    """SNR [dB] para drone em (north, east) com modelo UMa LOS."""
    d2d = np.sqrt((east - BS_EAST)**2 + (north - BS_NORTH)**2)
    d3d = np.sqrt(d2d**2 + (H_BS - H_DRONE)**2)
    d3d = np.maximum(d3d, 1.0)
    PL  = 28.0 + 22.0 * np.log10(d3d) + 20.0 * np.log10(FC_GHZ)
    return SNR_LINK_BUDGET - PL


# ── Hotspots (manualHotspots de build_scenario_variavel.m) ───────────────────
HOTSPOTS = np.array([
    [-466.7,  11166.7, 0.75, 180],
    [-616.2,  11154.4, 0.75, 180],
    [-317.2,  11179.0, 0.75, 180],
    [-500.0,  11571.4, 0.75, 180],
    [-649.5,  11559.1, 0.75, 180],
    [-350.5,  11583.7, 0.75, 180],
    [-533.3,  11976.2, 0.75, 180],
    [-682.8,  11963.9, 0.75, 180],
    [-383.8,  11988.5, 0.75, 180],
    [-566.7,  12381.0, 0.75, 180],
    [-716.2,  12368.6, 0.75, 180],
    [-417.2,  12393.3, 0.75, 180],
    [-600.0,  12785.7, 0.75, 180],
    [-749.5,  12773.4, 0.75, 180],
    [-450.5,  12798.0, 0.75, 180],
    [-633.3,  13190.5, 0.75, 180],
    [-782.8,  13178.2, 0.75, 180],
    [-483.8,  13202.8, 0.75, 180],
    [-666.7,  13595.2, 0.75, 180],
    [-816.2,  13582.9, 0.75, 180],
    [-517.2,  13607.5, 0.75, 180],
    [-700.0,  14000.0, 0.75, 180],
    [-849.5,  13987.7, 0.75, 180],
    [-550.5,  14012.3, 0.75, 180],
])

def ground_risk(north_grid, east_grid):
    """Soma de Gaussianas dos hotspots (mesma lógica do simulador)."""
    rho = np.zeros_like(north_grid, dtype=float)
    for h in HOTSPOTS:
        hn, he, amp, sigma = h
        r2 = (north_grid - hn)**2 + (east_grid - he)**2
        rho += amp * np.exp(-r2 / (2 * sigma**2))
    return rho


# ── Definições de rotas e cenários ───────────────────────────────────────────
ROUTES = {
    'RouteA':    {'start': (1000,  1000), 'end': (1000,  9000),
                  'color': '#1565C0', 'label': 'RouteA'},
    'RouteB':    {'start': (-500, 12000), 'end': (-600, 14000),
                  'color': '#6A1B9A', 'label': 'RouteB'},
    'RouteBlong':{'start': (0,    5500),  'end': (-700, 14000),
                  'color': '#E65100', 'label': 'RouteBlong'},
}

SCENARIOS = {
    '2r_static':   ['RouteA', 'RouteB'],
    '2r_variavel': ['RouteA', 'RouteBlong'],
}

# ── Grade de cálculo ──────────────────────────────────────────────────────────
N_EAST  = np.linspace(-500, 15000, 400)
N_NORTH = np.linspace(-1200, 1800, 300)
EE, NN  = np.meshgrid(N_EAST, N_NORTH)

SNR_GRID  = snr_at(NN, EE)
RISK_GRID = ground_risk(NN, EE)

# ── Colormaps customizados ────────────────────────────────────────────────────
snr_cmap = LinearSegmentedColormap.from_list(
    'snr', ['#B71C1C', '#F44336', '#FF9800', '#FFEB3B', '#8BC34A', '#1B5E20'], N=256)
risk_cmap = LinearSegmentedColormap.from_list(
    'risk', ['#FFFFFF', '#FFF9C4', '#FFB300', '#E65100', '#B71C1C'], N=256)


def draw_route(ax, key, snr_coloring=False, lw=2.5):
    """Desenha uma rota como linha com setas de direção."""
    r  = ROUTES[key]
    n0, e0 = r['start']
    n1, e1 = r['end']
    n_pts = np.linspace(n0, n1, 200)
    e_pts = np.linspace(e0, e1, 200)

    if snr_coloring:
        # Colore a rota pelo SNR local
        from matplotlib.collections import LineCollection
        snr_pts = snr_at(n_pts, e_pts)
        points  = np.array([e_pts, n_pts]).T.reshape(-1, 1, 2)
        segs    = np.concatenate([points[:-1], points[1:]], axis=1)
        lc = LineCollection(segs, cmap='RdYlGn', linewidth=lw+0.5,
                            norm=plt.Normalize(0, 35), zorder=4)
        lc.set_array(snr_pts)
        ax.add_collection(lc)
        # Borda preta fina
        ax.plot(e_pts, n_pts, color='k', lw=lw+2.0, zorder=3, alpha=0.35)
        return lc
    else:
        ax.plot(e_pts, n_pts, color='k',     lw=lw+2.0, zorder=3, alpha=0.4)
        ax.plot(e_pts, n_pts, color=r['color'], lw=lw, zorder=4)

    # Setas de direção (mid e 3/4)
    for t in [0.45, 0.70]:
        idx = int(t * len(n_pts))
        dn  = n_pts[min(idx+5, len(n_pts)-1)] - n_pts[max(idx-5, 0)]
        de  = e_pts[min(idx+5, len(e_pts)-1)] - e_pts[max(idx-5, 0)]
        ax.annotate('', xy=(e_pts[idx]+de*50, n_pts[idx]+dn*50),
                    xytext=(e_pts[idx], n_pts[idx]),
                    arrowprops=dict(arrowstyle='->', color=r['color'],
                                   lw=1.8, mutation_scale=14),
                    zorder=5)


def annotate_snr(ax, key, positions=(0.25, 0.5, 0.75)):
    """Anota SNR em posições ao longo da rota."""
    r  = ROUTES[key]
    n0, e0 = r['start']
    n1, e1 = r['end']
    for t in positions:
        n = n0 + t * (n1 - n0)
        e = e0 + t * (e1 - e0)
        s = snr_at(n, e)
        ax.text(e, n - 130, f'{s:.0f} dB', fontsize=7.5, ha='center',
                color='white', fontweight='bold', zorder=7,
                path_effects=[pe.withStroke(linewidth=2.5, foreground='black')])


def draw_bs(ax):
    """Marca a gNB com estrela e label."""
    ax.plot(BS_EAST, BS_NORTH, marker='*', ms=18, color='#FFD600',
            mec='black', mew=1.0, zorder=8, label='gNB')
    ax.text(BS_EAST, BS_NORTH + 180, 'gNB', fontsize=9, ha='center',
            fontweight='bold', color='#FFD600', zorder=9,
            path_effects=[pe.withStroke(linewidth=2.5, foreground='black')])


def draw_waypoints(ax, key):
    """Marca início (círculo) e fim (quadrado) da rota."""
    r = ROUTES[key]
    n0, e0 = r['start']
    n1, e1 = r['end']
    ax.plot(e0, n0, 'o', ms=7, color=r['color'], mec='white', mew=1.2, zorder=6)
    ax.plot(e1, n1, 's', ms=7, color=r['color'], mec='white', mew=1.2, zorder=6)


def style_ax(ax, title):
    ax.set_xlim(-500, 15000)
    ax.set_ylim(-1200, 1800)
    ax.set_xlabel('East (m)', fontsize=9)
    ax.set_ylabel('North (m)', fontsize=9)
    ax.set_title(title, fontsize=10, fontweight='bold')
    ax.set_aspect('equal')
    ax.grid(True, ls=':', alpha=0.3, color='white')
    ax.tick_params(labelsize=8)
    # Distâncias de referência (círculos em torno da BS)
    for r_km in [2, 5, 9]:
        circle = plt.Circle((BS_EAST, BS_NORTH), r_km*1000,
                             color='white', fill=False, ls='--',
                             lw=0.7, alpha=0.4, zorder=2)
        ax.add_patch(circle)
        ax.text(BS_EAST + r_km*1000 * 0.71, BS_NORTH + r_km*1000 * 0.71,
                f'{r_km} km', fontsize=7, color='white', alpha=0.7, zorder=3)


# ── Figura principal (2 cenários × 2 métricas) ────────────────────────────────
fig = plt.figure(figsize=(18, 13), facecolor='#1a1a2e')
gs  = GridSpec(2, 3, figure=fig, width_ratios=[1, 1, 0.04],
               hspace=0.35, wspace=0.12,
               left=0.06, right=0.93, top=0.93, bottom=0.06)

for row, (sc_name, route_keys) in enumerate(SCENARIOS.items()):

    # ── Painel esquerdo: SNR ─────────────────────────────────────────────────
    ax_snr = fig.add_subplot(gs[row, 0])
    ax_snr.set_facecolor('#0d0d1a')

    im_snr = ax_snr.pcolormesh(EE, NN, np.clip(SNR_GRID, 0, 35),
                                cmap=snr_cmap, vmin=0, vmax=35,
                                shading='gouraud', zorder=1)

    # Contorno de threshold SNR
    ax_snr.contour(EE, NN, SNR_GRID, levels=[THRESH_SNR],
                   colors='white', linewidths=1.2, linestyles='--',
                   alpha=0.7, zorder=2)
    ax_snr.text(BS_EAST + 4200, -400, f'SNR={THRESH_SNR:.0f} dB', fontsize=7.5,
                color='white', alpha=0.8,
                path_effects=[pe.withStroke(linewidth=1.5, foreground='black')])

    for key in route_keys:
        lc = draw_route(ax_snr, key, snr_coloring=True)
        annotate_snr(ax_snr, key, positions=[0.15, 0.5, 0.85])
        draw_waypoints(ax_snr, key)
    draw_bs(ax_snr)

    style_ax(ax_snr, f'{sc_name}  —  SNR (UMa LOS, fc=3.5 GHz)')

    # Legenda de rotas
    handles = [mpatches.Patch(color=ROUTES[k]['color'],
               label=ROUTES[k]['label']) for k in route_keys]
    handles += [plt.Line2D([0],[0], marker='*', color='#FFD600',
                ms=12, ls='none', label='gNB')]
    ax_snr.legend(handles=handles, loc='upper left', fontsize=8,
                  facecolor='#0d0d1a', edgecolor='white',
                  labelcolor='white', framealpha=0.8)

    # ── Painel direito: Risco ────────────────────────────────────────────────
    ax_rsk = fig.add_subplot(gs[row, 1])
    ax_rsk.set_facecolor('#0d0d1a')

    im_rsk = ax_rsk.pcolormesh(EE, NN, RISK_GRID,
                                cmap=risk_cmap, vmin=0, vmax=0.6,
                                shading='gouraud', zorder=1)

    # Contornos de risco
    ax_rsk.contour(EE, NN, RISK_GRID, levels=[0.1, 0.3, 0.5],
                   colors=['#FFF176', '#FF8F00', '#B71C1C'],
                   linewidths=0.9, alpha=0.8, zorder=2)

    for key in route_keys:
        draw_route(ax_rsk, key, snr_coloring=False)
        draw_waypoints(ax_rsk, key)
    draw_bs(ax_rsk)

    style_ax(ax_rsk, f'{sc_name}  —  Risco terrestre (hotspots)')

    handles2 = [mpatches.Patch(color=ROUTES[k]['color'],
                label=ROUTES[k]['label']) for k in route_keys]
    handles2 += [plt.Line2D([0],[0], marker='*', color='#FFD600',
                  ms=12, ls='none', label='gNB')]
    ax_rsk.legend(handles=handles2, loc='upper left', fontsize=8,
                  facecolor='#0d0d1a', edgecolor='white',
                  labelcolor='white', framealpha=0.8)

    # ── Colorbars compartilhadas ─────────────────────────────────────────────
    ax_cb = fig.add_subplot(gs[row, 2])
    ax_cb.set_visible(False)

    # SNR colorbar (esquerda de ax_snr)
    cb_snr = fig.colorbar(im_snr, ax=ax_snr, fraction=0.035, pad=0.02,
                          label='SNR (dB)')
    cb_snr.ax.yaxis.label.set_color('white')
    cb_snr.ax.tick_params(colors='white', labelsize=8)
    cb_snr.outline.set_edgecolor('white')

    # Risk colorbar (direita de ax_rsk)
    cb_rsk = fig.colorbar(im_rsk, ax=ax_rsk, fraction=0.035, pad=0.02,
                          label='Intensidade de risco')
    cb_rsk.ax.yaxis.label.set_color('white')
    cb_rsk.ax.tick_params(colors='white', labelsize=8)
    cb_rsk.outline.set_edgecolor('white')
    cb_rsk.set_ticks([0, 0.1, 0.3, 0.5])

# ── Título geral ──────────────────────────────────────────────────────────────
fig.text(0.5, 0.97,
         'Cenários MW-Otimo — Geometria, SNR e Risco Terrestre',
         ha='center', va='top', fontsize=14, fontweight='bold', color='white')

# ── Legenda do painel SNR (linha da rota = SNR) ───────────────────────────────
fig.text(0.27, 0.01,
         '● início   ■ fim   — rota colorida pelo SNR local   ○ contorno SNR = 5 dB',
         ha='center', fontsize=8, color='#CCCCCC')

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       'csv_export_mw_otimo', 'plots')
os.makedirs(OUT_DIR, exist_ok=True)
out = os.path.join(OUT_DIR, 'scenario_map.png')
fig.savefig(out, dpi=180, bbox_inches='tight', facecolor=fig.get_facecolor())
print(f'Saved: {out}')
plt.close(fig)
