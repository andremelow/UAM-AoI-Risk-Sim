"""
plot_rgnd_hotspots.py
Dois painéis:
  fig_gnd_1 — Mapa espacial de rho(x): hotspots + rotas + BS
  fig_gnd_2 — R_gnd médio por slot por política (K=8, C=8, 2r_variavel)
              + idem para 2r_static (R_gnd=0 sempre → linha de referência)
"""
import csv, glob, os, collections
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import matplotlib.patches as mpatches

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE       = os.path.join(SCRIPT_DIR, 'csv_export', 'capacity_debug')
OUT_DIR    = BASE

POLS   = ['round-robin-sw', 'pf-classic-sw', 'max-weight-sw', 'aoi-sw']
LABELS = {'round-robin-sw': 'RR', 'pf-classic-sw': 'PF',
          'max-weight-sw':  'MW', 'aoi-sw':        'AoI'}
COLORS = {'round-robin-sw': '#4878CF', 'pf-classic-sw': '#6ACC65',
          'max-weight-sw':  '#D65F5F', 'aoi-sw':        '#B47CC7'}

# ── Parâmetros do cenário (espelham build_scenario_variavel.m) ─────────────
# Rotas [North, East]
route_A   = dict(start=np.array([200,  4000]), goal=np.array([300,  6000]),
                 label='RouteA',    color='royalblue', ndrones_steady=4)
route_B   = dict(start=np.array([0,    5500]), goal=np.array([-700, 14000]),
                 label='RouteBlong', color='firebrick',  ndrones_steady=4)
BS_EAST, BS_NORTH = 5000, 0

# Hotspots manuais: [North, East, amplitude, sigma_m]
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

plt.rcParams.update({
    'font.family': 'sans-serif', 'font.size': 10,
    'axes.titlesize': 11, 'axes.labelsize': 10,
    'legend.fontsize': 9,  'figure.dpi': 150,
    'axes.spines.top': False, 'axes.spines.right': False,
})

# ══════════════════════════════════════════════════════════════════════════════
# Fig GND-1 — Mapa espacial de rho(x)
# ══════════════════════════════════════════════════════════════════════════════

# Replicar lógica de init_ground_risk para calcular extents do grid
all_north = np.array([route_A['start'][0], route_A['goal'][0],
                      route_B['start'][0], route_B['goal'][0]])
all_east  = np.array([route_A['start'][1], route_A['goal'][1],
                      route_B['start'][1], route_B['goal'][1]])
span      = max(np.ptp(all_north), np.ptp(all_east))
span      = max(span, 2002)            # corridorLength ref
margin    = max(200, span * 0.15)      # ≈ 1500 m
cell_size = max(20, round(span / 60))  # ≈ 167 m

x_min = min(all_north.min() - margin, BS_NORTH - margin)
x_max = max(all_north.max() + margin, BS_NORTH + margin)
y_min = min(all_east.min()  - margin, BS_EAST  - margin)
y_max = max(all_east.max()  + margin, BS_EAST  + margin)

xv = np.arange(x_min + cell_size/2, x_max, cell_size)
yv = np.arange(y_min + cell_size/2, y_max, cell_size)
XG, YG = np.meshgrid(xv, yv)      # shape (ny, nx); XG = North axis

rhoMap = np.zeros_like(XG)
for hs in HOTSPOTS:
    hn, he, amp, sig = hs
    rhoMap += amp * np.exp(-((XG - hn)**2 + (YG - he)**2) / (2 * sig**2))
rhoMap = np.clip(rhoMap, 0, 1)

fig1, ax = plt.subplots(figsize=(7, 10))
# imshow: extent = [ymin, ymax, xmin, xmax] (East on X, North on Y)
im = ax.imshow(rhoMap.T, origin='lower',
               extent=[y_min, y_max, x_min, x_max],
               cmap='YlOrRd', vmin=0, vmax=1, alpha=0.85, aspect='auto')
cb = fig1.colorbar(im, ax=ax, fraction=0.025, pad=0.03)
cb.set_label('Densidade populacional ρ(x)  [normalizada 0–1]')

# Rotas
for rt in [route_A, route_B]:
    sn, se = rt['start']
    gn, ge = rt['goal']
    ax.annotate('', xy=(ge, gn), xytext=(se, sn),
                arrowprops=dict(arrowstyle='->', color=rt['color'],
                                lw=2.0, mutation_scale=15))
    ax.plot([se, ge], [sn, gn], color=rt['color'], lw=2.0,
            label=rt['label'])
    ax.scatter([se, ge], [sn, gn], s=50, color=rt['color'], zorder=5)

# BS
ax.scatter([BS_EAST], [BS_NORTH], marker='^', s=140, color='black',
           zorder=6, label='BS gNB')
ax.annotate('BS', xy=(BS_EAST, BS_NORTH), xytext=(BS_EAST+300, BS_NORTH+200),
            fontsize=9, color='black')

# Hotspot centróides
ax.scatter(HOTSPOTS[:, 1], HOTSPOTS[:, 0], marker='x', s=40,
           color='darkred', linewidths=1.2, label='Hotspots', zorder=7)

ax.set_xlabel('East [m]')
ax.set_ylabel('North [m]')
ax.set_title('Mapa de densidade ρ(x) — hotspots e rotas\n'
             '(RouteA: canal bom, RouteBlong: canal degradante + hotspots no fim)')
ax.legend(loc='upper left', framealpha=0.85)
ax.grid(True, linestyle=':', alpha=0.3)

out1 = os.path.join(OUT_DIR, 'fig_gnd1_hotspot_map.png')
fig1.savefig(out1, bbox_inches='tight')
print(f'Saved: {out1}')

# ══════════════════════════════════════════════════════════════════════════════
# Fig GND-2 — R_gnd médio por slot  (K=8, C=8, 2r_variavel)
#             + referência 2r_static (R_gnd ≡ 0)
# ══════════════════════════════════════════════════════════════════════════════

def load_rgnd_per_slot(slot_csv):
    """Retorna dict {slot: r_map_mean_over_active_drones}."""
    per_slot = collections.defaultdict(list)
    with open(slot_csv) as f:
        for row in csv.DictReader(f):
            per_slot[int(row['slot'])].append(float(row['r_map']))
    return {s: np.mean(v) for s, v in per_slot.items()}

fig2, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=False)

for ax, sc, sc_label in zip(axes,
                             ['2r_variavel', '2r_static'],
                             ['Gradiente (RouteBlong até E=14km)',
                              'Estático (RouteB até E=8km, sem hotspots)']):
    files = glob.glob(os.path.join(BASE, 'K8', 'C08', sc, '*_slot_series.csv'))
    if not files:
        ax.set_title(f'{sc_label} — sem dados')
        continue

    for f in sorted(files):
        pol = os.path.basename(f).split('_N')[0]
        if pol not in POLS:
            continue
        per_slot = load_rgnd_per_slot(f)
        slots = sorted(per_slot)
        vals  = [per_slot[s] for s in slots]

        # suavização: média móvel de 50 slots
        w = 50
        vals_sm = np.convolve(vals, np.ones(w)/w, mode='same')

        ax.plot(slots, vals_sm, color=COLORS[pol], linewidth=1.6,
                label=LABELS[pol], alpha=0.9)
        # linha fina original
        ax.plot(slots, vals, color=COLORS[pol], linewidth=0.4, alpha=0.2)

    ax.set_ylabel('R_gnd médio\n(média sobre drones ativos)')
    ax.set_title(sc_label)
    ax.legend(loc='upper left', framealpha=0.85)
    ax.grid(True, linestyle=':', alpha=0.35)
    ax.yaxis.set_major_formatter(ticker.FormatStrFormatter('%.0f'))

    # marcação: zona de hotspots (slots ≈ 5000–11500 para os drones 11–15)
    if sc == '2r_variavel':
        ax.axvspan(5000, 11500, alpha=0.07, color='red', label='_zona hotspot')
        yhi = ax.get_ylim()[1]
        ax.annotate('Zona\nhotspot', xy=(5100, yhi * 0.75),
                    fontsize=8, color='darkred', alpha=0.8)

axes[1].set_xlabel('Slot')
fig2.suptitle('R_gnd médio por slot — K=8, C=8\n'
              'Suavizado (média móvel 50 slots) + original (transparente)',
              fontweight='bold')
fig2.tight_layout()
out2 = os.path.join(OUT_DIR, 'fig_gnd2_rgnd_per_slot.png')
fig2.savefig(out2, bbox_inches='tight')
print(f'Saved: {out2}')

print('\nDone.')
