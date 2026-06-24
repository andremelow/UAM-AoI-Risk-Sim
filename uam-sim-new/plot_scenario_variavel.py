"""
plot_scenario_variavel.py
Visualização do cenário 2r_variavel com hotspots atualizados.

Painel esquerdo  — mapa 2D de densidade ρ(x) + rotas + BS + hotspots
Painel central   — perfil ρ(x) ao longo de cada rota vs distância
Painel direito   — perfil SNR estimado ao longo de RouteBlong (UMa LoS aprox.)
"""
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from scipy.interpolate import RegularGridInterpolator

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR    = os.path.join(SCRIPT_DIR, 'csv_export', 'capacity_debug')

# ── Parâmetros do cenário (espelham build_scenario_variavel.m) ─────────────
ROUTES = [
    dict(start=np.array([1000., 1000.]), goal=np.array([1000., 9000.]),
         label='RouteA',    color='#99CCFF'),
    dict(start=np.array([0.,    5500.]), goal=np.array([-700., 14000.]),
         label='RouteBlong', color='#FF9999'),
]
BS_EAST, BS_NORTH, BS_ALT = 5000., 0., 30.   # Alt = altura acima do solo [m]
DRONE_ALT = 80.                               # altitude típica dos drones [m]

# Parâmetros de rádio
FC_GHZ       = 3.5
P_TX_DBM     = 23.
GNB_GAIN_DBI = 15.
NOISE_FIG_DB = 7.
BW_HZ        = 20e6
THERMAL_DBM  = -174 + 10*np.log10(BW_HZ)    # ≈ -101 dBm
NOISE_FLOOR  = THERMAL_DBM + NOISE_FIG_DB    # ≈ -94 dBm
THRESH_SNR   = 5.                            # threshold para throughput

# Hotspots [North, East, amp, sigma_m]
HOTSPOTS = np.array([
    [-466.7, 11166.7, 0.75, 180],
    [-616.2, 11154.4, 0.75, 180],
    [-317.2, 11179.0, 0.75, 180],
    [-500.0, 11571.4, 0.75, 180],
    [-649.5, 11559.1, 0.75, 180],
    [-350.5, 11583.7, 0.75, 180],
    [-533.3, 11976.2, 0.75, 180],
    [-682.8, 11963.9, 0.75, 180],
    [-383.8, 11988.5, 0.75, 180],
    [-566.7, 12381.0, 0.75, 180],
    [-716.2, 12368.6, 0.75, 180],
    [-417.2, 12393.3, 0.75, 180],
    [-600.0, 12785.7, 0.75, 180],
    [-749.5, 12773.4, 0.75, 180],
    [-450.5, 12798.0, 0.75, 180],
    [-633.3, 13190.5, 0.75, 180],
    [-782.8, 13178.2, 0.75, 180],
    [-483.8, 13202.8, 0.75, 180],
    [-666.7, 13595.2, 0.75, 180],
    [-816.2, 13582.9, 0.75, 180],
    [-517.2, 13607.5, 0.75, 180],
    [-700.0, 14000.0, 0.75, 180],
    [-849.5, 13987.7, 0.75, 180],
    [-550.5, 14012.3, 0.75, 180],
])

# ── Grid (replica init_ground_risk.m) ─────────────────────────────────────
all_n = np.array([r['start'][0] for r in ROUTES] + [r['goal'][0] for r in ROUTES])
all_e = np.array([r['start'][1] for r in ROUTES] + [r['goal'][1] for r in ROUTES])
span      = max(np.ptp(all_n), np.ptp(all_e))
span      = max(span, 2002.)
margin    = max(200., span * 0.15)
cell_size = max(20., round(span / 60.))

x_min = min(all_n.min() - margin, BS_NORTH - margin)
x_max = max(all_n.max() + margin, BS_NORTH + margin)
y_min = min(all_e.min()  - margin, BS_EAST  - margin)
y_max = max(all_e.max()  + margin, BS_EAST  + margin)

xv = np.arange(x_min + cell_size/2, x_max, cell_size)   # North
yv = np.arange(y_min + cell_size/2, y_max, cell_size)   # East
XG, YG = np.meshgrid(xv, yv)   # XG[i,j]=North=xv[j], YG[i,j]=East=yv[i]

rhoMap = np.zeros_like(XG)
for hs in HOTSPOTS:
    hn, he, amp, sig = hs
    rhoMap += amp * np.exp(-((XG - hn)**2 + (YG - he)**2) / (2*sig**2))
rhoMap = np.clip(rhoMap, 0, 1)

# ── Interpolador para perfil ao longo das rotas ────────────────────────────
# rhoMap[i,j] = ρ(North=xv[j], East=yv[i])
# RegularGridInterpolator(points=(yv, xv), values=rhoMap) → query (East, North)
rho_interp = RegularGridInterpolator(
    (yv, xv), rhoMap, method='linear', bounds_error=False, fill_value=0.)

# ── SNR ao longo de RouteBlong (UMa LoS aproximado) ───────────────────────
def snr_uma_los(north, east, drone_alt=DRONE_ALT):
    """SNR em dB usando PL UMa LoS simplificado (3GPP TR 38.901)."""
    d2d  = np.sqrt((north - BS_NORTH)**2 + (east - BS_EAST)**2)
    d3d  = np.sqrt(d2d**2 + (drone_alt - BS_ALT)**2)
    d3d  = np.maximum(d3d, 1.)
    # UMa LoS: PL = 28 + 22*log10(d3d) + 20*log10(fc_GHz)
    PL   = 28 + 22*np.log10(d3d) + 20*np.log10(FC_GHZ)
    return (P_TX_DBM + GNB_GAIN_DBI - PL) - NOISE_FLOOR

N_SAMP = 600

# ── Figure ────────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(17, 8), facecolor='#0d0d0d')
gs  = fig.add_gridspec(1, 3, width_ratios=[2.4, 1, 1],
                        left=0.04, right=0.97, top=0.91, bottom=0.09,
                        wspace=0.32)

AX_KW = dict(facecolor='#111', tick_params={'colors': '#aaa'},
             xlabel_kw={'color': '#aaa'}, ylabel_kw={'color': '#aaa'})

def dark_ax(ax):
    ax.set_facecolor('#111')
    ax.tick_params(colors='#aaa')
    ax.xaxis.label.set_color('#aaa')
    ax.yaxis.label.set_color('#aaa')
    ax.title.set_color('white')
    for sp in ax.spines.values():
        sp.set_edgecolor('#444')
    ax.grid(color='#333', linestyle=':', alpha=0.6)

# ─── Painel 1: Mapa 2D ────────────────────────────────────────────────────
ax1 = fig.add_subplot(gs[0])
dark_ax(ax1)

# rhoMap.T corrige transposição: imshow row→Y(North), col→X(East)
im = ax1.imshow(rhoMap.T, origin='lower',
                extent=[y_min, y_max, x_min, x_max],
                cmap='hot', vmin=0, vmax=1, alpha=0.85, aspect='auto')
cb = fig.colorbar(im, ax=ax1, fraction=0.025, pad=0.02)
cb.set_label('Densidade ρ(x)', color='#aaa', fontsize=9)
cb.ax.yaxis.set_tick_params(color='#aaa')
plt.setp(cb.ax.yaxis.get_ticklabels(), color='#aaa')

# Rotas coloridas por SNR ao longo do trajeto
for rt in ROUTES:
    ns = np.linspace(rt['start'][0], rt['goal'][0], N_SAMP)
    es = np.linspace(rt['start'][1], rt['goal'][1], N_SAMP)
    snr = snr_uma_los(ns, es)
    norm_snr = mcolors.Normalize(vmin=0, vmax=40)
    cmap_snr  = plt.cm.RdYlGn

    for k in range(N_SAMP - 1):
        ax1.plot([es[k], es[k+1]], [ns[k], ns[k+1]],
                 color=cmap_snr(norm_snr(snr[k])), linewidth=2.5, solid_capstyle='round')

    # Marcadores de início/fim
    ax1.scatter([rt['start'][1]], [rt['start'][0]], s=70,
                color='white', zorder=10, marker='o')
    ax1.scatter([rt['goal'][1]],  [rt['goal'][0]],  s=70,
                color='white', zorder=10, marker='s')
    ax1.text(rt['goal'][1] + 100, rt['goal'][0], rt['label'],
             color=rt['color'], fontsize=8.5, va='center', fontweight='bold')

# Colorbar de SNR sobreposta (ghost axes)
sm = plt.cm.ScalarMappable(cmap=cmap_snr, norm=norm_snr)
sm.set_array([])
cb2 = fig.colorbar(sm, ax=ax1, fraction=0.018, pad=0.10, location='right')
cb2.set_label('SNR estimado (dB)', color='#aaa', fontsize=8)
cb2.ax.yaxis.set_tick_params(color='#aaa')
plt.setp(cb2.ax.yaxis.get_ticklabels(), color='#aaa')
cb2.ax.axhline(THRESH_SNR, color='white', linewidth=1.0, linestyle='--', alpha=0.7)
cb2.ax.text(2.8, THRESH_SNR + 0.5, f'thr={THRESH_SNR:.0f}dB',
            color='white', fontsize=7, transform=cb2.ax.transData)

# BS
ax1.scatter([BS_EAST], [BS_NORTH], marker='^', s=150,
            color='cyan', zorder=11, label='BS gNB')
ax1.annotate(f'BS\n(E={BS_EAST:.0f}m)', xy=(BS_EAST, BS_NORTH),
             xytext=(BS_EAST - 1200, BS_NORTH + 400),
             color='cyan', fontsize=8,
             arrowprops=dict(arrowstyle='->', color='cyan', lw=0.8))

# Hotspot markers
ax1.scatter(HOTSPOTS[:, 1], HOTSPOTS[:, 0], marker='x', s=35,
            color='#FF6600', linewidths=1.0, zorder=9, label='Hotspots')

# Linha do terço final (início)
t23_n = ROUTES[1]['start'][0] + (2/3)*(ROUTES[1]['goal'][0] - ROUTES[1]['start'][0])
t23_e = ROUTES[1]['start'][1] + (2/3)*(ROUTES[1]['goal'][1] - ROUTES[1]['start'][1])
ax1.axvline(t23_e, color='#FF6600', linewidth=0.8, linestyle='--', alpha=0.6)
ax1.text(t23_e + 80, x_min + 400, '← terço final', color='#FF6600',
         fontsize=7.5, rotation=90, va='bottom', alpha=0.8)

ax1.set_xlabel('East (m)')
ax1.set_ylabel('North (m)')
ax1.set_title('Cenário 2r_variavel — mapa ρ(x) + SNR ao longo das rotas', fontsize=10)
ax1.legend(loc='upper left', facecolor='#222', edgecolor='#555',
           labelcolor='#ccc', fontsize=8)
ax1.set_xlim(y_min, y_max)
ax1.set_ylim(x_min, x_max)

# ─── Painel 2: Perfil ρ(x) ao longo das rotas ────────────────────────────
ax2 = fig.add_subplot(gs[1])
dark_ax(ax2)

for rt in ROUTES:
    ns   = np.linspace(rt['start'][0], rt['goal'][0], N_SAMP)
    es   = np.linspace(rt['start'][1], rt['goal'][1], N_SAMP)
    rho  = rho_interp(np.column_stack([es, ns]))
    dx   = np.diff(ns); dy = np.diff(es)
    dist = np.concatenate([[0], np.cumsum(np.sqrt(dx**2 + dy**2))]) / 1000
    ax2.plot(rho, dist, color=rt['color'], linewidth=2.0, label=rt['label'])
    # marca início do terço final (RouteBlong)
    if 'long' in rt['label'].lower():
        t23_idx = np.argmin(np.abs(dist - dist[-1]*2/3))
        ax2.axhline(dist[t23_idx], color='#FF6600', linewidth=0.8,
                    linestyle='--', alpha=0.7)
        ax2.text(0.01, dist[t23_idx] + 0.1, '↑ terço final', color='#FF6600', fontsize=7.5)

ax2.set_xlabel('Densidade ρ(x)')
ax2.set_ylabel('Distância percorrida (km)')
ax2.set_title('Perfil ρ\nao longo da rota', fontsize=10)
ax2.set_xlim(-0.02, 1.05)
ax2.set_ylim(0, max(
    np.linalg.norm(ROUTES[1]['goal'] - ROUTES[1]['start'])/1000 * 1.05, 2))
ax2.legend(facecolor='#222', edgecolor='#555', labelcolor='#ccc', fontsize=8)

# ─── Painel 3: SNR ao longo de RouteBlong ─────────────────────────────────
ax3 = fig.add_subplot(gs[2])
dark_ax(ax3)

rt = ROUTES[1]
ns   = np.linspace(rt['start'][0], rt['goal'][0], N_SAMP)
es   = np.linspace(rt['start'][1], rt['goal'][1], N_SAMP)
snr  = snr_uma_los(ns, es)
dx   = np.diff(ns); dy = np.diff(es)
dist = np.concatenate([[0], np.cumsum(np.sqrt(dx**2 + dy**2))]) / 1000

# colore o perfil pela faixa de SNR
ax3.fill_betweenx(dist, THRESH_SNR, snr, where=(snr >= THRESH_SNR),
                  color='#3a7a3a', alpha=0.35, label='Acima do threshold')
ax3.fill_betweenx(dist, snr, THRESH_SNR, where=(snr < THRESH_SNR),
                  color='#7a2a2a', alpha=0.35, label='Abaixo do threshold')
ax3.plot(snr, dist, color='#FF9999', linewidth=2.0, label='RouteBlong SNR')
ax3.axvline(THRESH_SNR, color='white', linewidth=0.9, linestyle='--', alpha=0.6)
ax3.text(THRESH_SNR + 0.3, dist[-1]*0.05, f'thr={THRESH_SNR:.0f}dB',
         color='white', fontsize=8, va='bottom')

# marca terço final
t23_d = dist[-1] * 2/3
ax3.axhline(t23_d, color='#FF6600', linewidth=0.8, linestyle='--', alpha=0.7)
ax3.text(THRESH_SNR + 0.3, t23_d + 0.1, '↑ terço final', color='#FF6600', fontsize=7.5)

# zona de hotspots (E≈11167 para frente)
snr_at_t23 = float(np.interp(t23_d, dist, snr))
ax3.annotate(f'SNR≈{snr_at_t23:.1f}dB\nno início do\nterço final',
             xy=(snr_at_t23, t23_d), xytext=(snr_at_t23 + 4, t23_d + 0.4),
             color='#FF6600', fontsize=7.5,
             arrowprops=dict(arrowstyle='->', color='#FF6600', lw=0.8))

ax3.set_xlabel('SNR (dB) — UMa LoS aprox.')
ax3.set_ylabel('Distância percorrida (km)')
ax3.set_title('Perfil SNR\nRouteBlong', fontsize=10)
ax3.set_ylim(0, dist[-1] * 1.05)
ax3.legend(facecolor='#222', edgecolor='#555', labelcolor='#ccc', fontsize=8,
           loc='lower left')

fig.suptitle('Cenário 2r_variavel — hotspots no terço final de RouteBlong',
             color='white', fontsize=12, fontweight='bold')

out = os.path.join(OUT_DIR, 'fig_scenario_variavel.png')
fig.savefig(out, bbox_inches='tight', facecolor=fig.get_facecolor(), dpi=150)
print(f'Saved: {out}')
