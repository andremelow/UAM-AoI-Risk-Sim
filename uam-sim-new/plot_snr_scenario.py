"""
plot_snr_scenario.py
Perfis de sinal do cenário 2r_variavel usando dados reais da simulação.

Três painéis:
  Esq  — Mapa 2D de SNR sobre a área (modelo UMa LoS, drone_alt=80m)
  Centro — SNR vs distância percorrida por rota (dados reais + envelope min/max)
  Dir  — Throughput Shannon vs distância (BW=20 MHz)
"""
import csv, glob, os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE       = os.path.join(SCRIPT_DIR, 'csv_export', 'capacity_debug')
OUT_DIR    = BASE

# ── Parâmetros do cenário ─────────────────────────────────────────────────
ROUTES = [
    dict(start=np.array([200.,  4000.]), goal=np.array([300.,  6000.]),
         label='RouteA',    color='#99CCFF', drone_ids=list(range(1,11))),
    dict(start=np.array([0.,    5500.]), goal=np.array([-700., 14000.]),
         label='RouteBlong', color='#FF9999', drone_ids=list(range(11,16))),
]
BS_EAST, BS_NORTH, BS_ALT = 5000., 0., 30.
DRONE_ALT   = 80.
FC_GHZ      = 3.5
P_TX_DBM    = 23.
GNB_GAIN    = 15.
NOISE_FIG   = 7.
BW_MHZ      = 20.
THERMAL_DBM = -174 + 10*np.log10(BW_MHZ * 1e6)
NOISE_FLOOR = THERMAL_DBM + NOISE_FIG
THRESH_SNR  = 5.
SPEED       = 10.02   # m/s
UPD_RATE    = 7       # slots/s

# ── SNR analítico UMa LoS (para mapa 2D) ─────────────────────────────────
def snr_uma(north, east, alt=DRONE_ALT):
    d3d = np.sqrt((north - BS_NORTH)**2 + (east - BS_EAST)**2
                  + (alt - BS_ALT)**2)
    d3d = np.maximum(d3d, 1.)
    PL  = 28 + 22*np.log10(d3d) + 20*np.log10(FC_GHZ)
    return (P_TX_DBM + GNB_GAIN - PL) - NOISE_FLOOR

def throughput(snr_db):
    snr_lin = 10**(snr_db / 10.)
    cap     = np.where(snr_db >= THRESH_SNR,
                       BW_MHZ * np.log2(1 + snr_lin), 0.)
    return cap   # Mbps

# ── Dados reais: SNR vs distância por rota ────────────────────────────────
slot_files = sorted(glob.glob(os.path.join(BASE, 'K8', 'C08', '2r_variavel',
                                           '*_slot_series.csv')))
# usa qualquer política (SNR é determinístico dado a posição)
slot_rows  = list(csv.DictReader(open(slot_files[0])))

N_BINS = 80

snr_by_route = {}
for rt in ROUTES:
    L        = np.linalg.norm(rt['goal'] - rt['start'])
    all_d, all_snr = [], []
    for d in rt['drone_ids']:
        dr = sorted([r for r in slot_rows if int(r['drone_id']) == d],
                    key=lambda r: int(r['slot']))
        if not dr:
            continue
        t0 = int(dr[0]['slot'])
        for r in dr:
            dt   = (int(r['slot']) - t0) / UPD_RATE
            frac = min(dt * SPEED / L, 1.0)
            all_d.append(frac * L / 1000)
            all_snr.append(float(r['snr_db']))
    all_d   = np.array(all_d)
    all_snr = np.array(all_snr)
    bins    = np.linspace(0, L/1000, N_BINS + 1)
    mids    = 0.5*(bins[:-1] + bins[1:])
    idx     = np.digitize(all_d, bins)
    mn  = np.array([np.mean (all_snr[idx==i]) if np.any(idx==i) else np.nan
                    for i in range(1, len(bins))])
    lo  = np.array([np.percentile(all_snr[idx==i], 10) if np.any(idx==i) else np.nan
                    for i in range(1, len(bins))])
    hi  = np.array([np.percentile(all_snr[idx==i], 90) if np.any(idx==i) else np.nan
                    for i in range(1, len(bins))])
    snr_by_route[rt['label']] = dict(dist=mids, mean=mn, lo=lo, hi=hi,
                                     color=rt['color'], L_km=L/1000)

# ── Grid para mapa 2D ─────────────────────────────────────────────────────
all_n = np.array([r['start'][0] for r in ROUTES] + [r['goal'][0] for r in ROUTES])
all_e = np.array([r['start'][1] for r in ROUTES] + [r['goal'][1] for r in ROUTES])
span      = max(np.ptp(all_n), np.ptp(all_e), 2002.)
margin    = max(200., span * 0.15)
cell_size = max(50., round(span / 80.))   # resolução menor para ser rápido

x_min = min(all_n.min() - margin, BS_NORTH - margin)
x_max = max(all_n.max() + margin, BS_NORTH + margin)
y_min = min(all_e.min()  - margin, BS_EAST  - margin)
y_max = max(all_e.max()  + margin, BS_EAST  + margin)

xv = np.arange(x_min, x_max, cell_size)
yv = np.arange(y_min, y_max, cell_size)
XG, YG = np.meshgrid(xv, yv)
SNR_MAP = snr_uma(XG, YG)

# ── Figura ────────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(17, 6.5), facecolor='#0d0d0d')
gs  = fig.add_gridspec(1, 3, width_ratios=[2.2, 1.2, 1.2],
                        left=0.04, right=0.97, top=0.89, bottom=0.10,
                        wspace=0.30)

def dark_ax(ax):
    ax.set_facecolor('#111')
    ax.tick_params(colors='#aaa', labelsize=8)
    ax.xaxis.label.set_color('#aaa')
    ax.yaxis.label.set_color('#aaa')
    ax.title.set_color('white')
    for sp in ax.spines.values():
        sp.set_edgecolor('#444')
    ax.grid(color='#2a2a2a', linestyle=':', alpha=0.8)

# ─── Painel 1: Mapa 2D de SNR ─────────────────────────────────────────────
ax1 = fig.add_subplot(gs[0])
dark_ax(ax1)

SNR_CMAP = 'RdYlGn'
SNR_NORM = mcolors.Normalize(vmin=0, vmax=40)

im = ax1.imshow(SNR_MAP.T, origin='lower',
                extent=[y_min, y_max, x_min, x_max],
                cmap=SNR_CMAP, norm=SNR_NORM, alpha=0.80, aspect='auto')

cb = fig.colorbar(im, ax=ax1, fraction=0.025, pad=0.02)
cb.set_label('SNR (dB)', color='#aaa', fontsize=9)
cb.ax.yaxis.set_tick_params(color='#aaa', labelsize=8)
plt.setp(cb.ax.yaxis.get_ticklabels(), color='#aaa')
cb.ax.axhline(THRESH_SNR, color='white', linewidth=1.2, linestyle='--')
cb.ax.text(3.5, THRESH_SNR + 0.5, f'{THRESH_SNR:.0f} dB',
           color='white', fontsize=7.5, transform=cb.ax.transData)

# Contorno de threshold
cs = ax1.contour(YG, XG, SNR_MAP, levels=[THRESH_SNR],
                 colors=['white'], linewidths=0.9, linestyles='--', alpha=0.7)
ax1.clabel(cs, fmt=f'{THRESH_SNR:.0f} dB', colors='white', fontsize=7.5)

# Rotas coloridas por SNR
N_SEG = 500
cmap_snr = plt.cm.get_cmap(SNR_CMAP)
for rt in ROUTES:
    ns = np.linspace(rt['start'][0], rt['goal'][0], N_SEG)
    es = np.linspace(rt['start'][1], rt['goal'][1], N_SEG)
    sv = snr_uma(ns, es)
    for k in range(N_SEG - 1):
        ax1.plot([es[k], es[k+1]], [ns[k], ns[k+1]],
                 color=cmap_snr(SNR_NORM(sv[k])),
                 linewidth=3.0, solid_capstyle='round')
    ax1.scatter([rt['start'][1]], [rt['start'][0]], s=60,
                color='white', zorder=10, marker='o')
    ax1.scatter([rt['goal'][1]],  [rt['goal'][0]],  s=60,
                color='white', zorder=10, marker='s')
    ax1.text(rt['goal'][1] + 80, rt['goal'][0] + 50,
             rt['label'], color=rt['color'], fontsize=8.5, fontweight='bold')

# BS
ax1.scatter([BS_EAST], [BS_NORTH], marker='^', s=160, color='cyan', zorder=11)
ax1.annotate('BS', xy=(BS_EAST, BS_NORTH),
             xytext=(BS_EAST - 1400, BS_NORTH + 350),
             color='cyan', fontsize=8.5,
             arrowprops=dict(arrowstyle='->', color='cyan', lw=0.9))

# Raio de cobertura (SNR = threshold)
# R ≈ from calibration: at d~7km SNR≈threshold
from matplotlib.patches import Circle
r_cov = np.sqrt(((THRESH_SNR - 28 - 20*np.log10(FC_GHZ)
                   - P_TX_DBM - GNB_GAIN + NOISE_FLOOR) / -22)**10) * 1.0
# numeric: find d where snr=0 → iterative
d_test = np.logspace(2, 4.5, 1000)
snr_test = snr_uma(BS_NORTH + d_test, BS_EAST)
d_thresh = float(np.interp(THRESH_SNR, snr_test[::-1], d_test[::-1]))
circ = Circle((BS_EAST, BS_NORTH), d_thresh, fill=False,
              edgecolor='white', linewidth=0.6, linestyle=':', alpha=0.4)
ax1.add_patch(circ)
ax1.text(BS_EAST, BS_NORTH + d_thresh + 200,
         f'R≈{d_thresh/1000:.1f} km\n(SNR={THRESH_SNR:.0f} dB)',
         color='white', fontsize=7, ha='center', alpha=0.6)

ax1.set_xlabel('East (m)')
ax1.set_ylabel('North (m)')
ax1.set_title('Mapa de SNR — UMa LoS  (drone alt=80 m)', fontsize=10)
ax1.set_xlim(y_min, y_max)
ax1.set_ylim(x_min, x_max)

# ─── Painel 2: SNR vs distância (dados reais) ─────────────────────────────
ax2 = fig.add_subplot(gs[1])
dark_ax(ax2)

for label, d in snr_by_route.items():
    dist, mn, lo, hi = d['dist'], d['mean'], d['lo'], d['hi']
    ax2.fill_betweenx(dist, lo, hi, alpha=0.20, color=d['color'])
    ax2.plot(mn, dist, color=d['color'], linewidth=2.2, label=label)

    # marca zona abaixo do threshold
    below = mn < THRESH_SNR
    if np.any(below):
        d_cross = float(np.interp(THRESH_SNR, mn[::-1], dist[::-1]))
        ax2.axhline(d_cross, color=d['color'], linewidth=0.7,
                    linestyle=':', alpha=0.6)
        ax2.text(THRESH_SNR + 1, d_cross + 0.1,
                 f'{d_cross:.1f} km', color=d['color'], fontsize=7.5)

ax2.axvline(THRESH_SNR, color='white', linewidth=0.9, linestyle='--', alpha=0.7)
ax2.text(THRESH_SNR + 0.3, 0.15, f'{THRESH_SNR:.0f} dB\nthr',
         color='white', fontsize=7.5, va='bottom')

# marca terço final de RouteBlong
L_B = snr_by_route['RouteBlong']['L_km']
ax2.axhline(L_B * 2/3, color='#FF6600', linewidth=0.8, linestyle='--', alpha=0.7)
ax2.text(2, L_B*2/3 + 0.1, '↑ hotspots', color='#FF6600', fontsize=7.5)

ax2.set_xlabel('SNR (dB)\n[médio ± P10–P90 entre drones]')
ax2.set_ylabel('Distância percorrida (km)')
ax2.set_title('Perfil SNR\n(dados simulados, K=8 C=8)', fontsize=10)
ax2.set_ylim(0, L_B * 1.05)
ax2.set_xlim(0, 45)
ax2.legend(facecolor='#222', edgecolor='#555', labelcolor='#ccc', fontsize=8.5)

# ─── Painel 3: Throughput Shannon vs distância ────────────────────────────
ax3 = fig.add_subplot(gs[2])
dark_ax(ax3)

for label, d in snr_by_route.items():
    dist, mn, lo, hi = d['dist'], d['mean'], d['lo'], d['hi']
    cap_mn = throughput(mn)
    cap_lo = throughput(lo)
    cap_hi = throughput(hi)
    ax3.fill_betweenx(dist, cap_lo, cap_hi, alpha=0.20, color=d['color'])
    ax3.plot(cap_mn, dist, color=d['color'], linewidth=2.2, label=label)

# zona de corte (cap=0)
ax3.axvline(0, color='white', linewidth=0.6, linestyle=':', alpha=0.4)
ax3.axhline(L_B * 2/3, color='#FF6600', linewidth=0.8, linestyle='--', alpha=0.7)
ax3.text(1, L_B*2/3 + 0.1, '↑ hotspots', color='#FF6600', fontsize=7.5)

ax3.set_xlabel('Throughput Shannon\n(Mbps, BW=20 MHz)')
ax3.set_ylabel('Distância percorrida (km)')
ax3.set_title('Capacidade de canal\n(Shannon, BW=20 MHz)', fontsize=10)
ax3.set_ylim(0, L_B * 1.05)
ax3.set_xlim(-5, None)
ax3.legend(facecolor='#222', edgecolor='#555', labelcolor='#ccc', fontsize=8.5)

fig.suptitle('Cenário 2r_variavel — Perfis de sinal ao longo das rotas',
             color='white', fontsize=12, fontweight='bold')

out = os.path.join(OUT_DIR, 'fig_snr_scenario.png')
fig.savefig(out, bbox_inches='tight', facecolor=fig.get_facecolor(), dpi=150)
print(f'Saved: {out}')
