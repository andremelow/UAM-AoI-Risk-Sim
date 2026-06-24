"""
plot_rmap_fraction.py
Fracao do risco devida ao mapa de solo (r_map/r_tot) ao longo das rotas.

Mostra onde o termo ESPACIAL de risco (que o surrogate Jhat = n_s*h1+n_v*z
ignora) domina o risco real R_tot.  Usa os dados reais por slot da
simulacao; cada drone e mapeado a sua rota pela duracao de voo e a posicao
ao longo da rota por frac=(slot-entry)/(exit-entry).

Painel 1 — fracao r_map/r_tot (%) vs East  (media + banda P25-P75)
Painel 2 — r_map e r_tot absolutos (medios) vs East

Usage:
  python plot_rmap_fraction.py --dir csv_export_mw_otimo_a23 --K 8 --C 32 \
      --policy mw-sw-otimo
"""
import os, glob, csv, argparse
import numpy as np
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser()
parser.add_argument('--dir',    default='csv_export_mw_otimo_a23')
parser.add_argument('--K',      type=int, default=8)
parser.add_argument('--C',      type=int, default=32)
parser.add_argument('--policy', default='mw-sw-otimo')
parser.add_argument('--scenario', default='2r_a23')
args = parser.parse_args()

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR   = os.path.join(SCRIPT_DIR, args.dir)
RUN_DIR    = os.path.join(DATA_DIR, f'K{args.K}', f'C{args.C:02d}', args.scenario)
OUT_DIR    = os.path.join(DATA_DIR, 'plots')
os.makedirs(OUT_DIR, exist_ok=True)

def find_one(pattern):
    hits = glob.glob(os.path.join(RUN_DIR, pattern))
    if not hits:
        raise SystemExit(f'Nenhum arquivo {pattern} em {RUN_DIR}')
    return sorted(hits)[0]

slot_csv  = find_one(f'{args.policy}_*_slot_series.csv')
drone_csv = find_one(f'{args.policy}_*_drone_summary.csv')

# ── Rotas do cenario 2r_a23 (East: start -> goal) ─────────────────────────
ROUTES = {
    'RouteA':     dict(startE=1000., goalE= 9000., L=8000.,  color='#2196F3'),
    'RouteBhalf': dict(startE=9750., goalE=14000., L=4264.,  color='#E53935'),
}
BS_EAST = 5000.
HOTSPOT_E = (11167., 14000.)   # terco final (zona de hotspots no mapa de risco)

# ── Mapeia drone -> rota pela duracao de voo (RouteA ~1.9x mais longa) ─────
drones = {}
for r in csv.DictReader(open(drone_csv)):
    d = int(r['drone_id'])
    entry, exit_ = float(r['entry_slot']), float(r['exit_slot'])
    drones[d] = dict(entry=entry, exit=exit_, dur=exit_ - entry)
durs = np.array([v['dur'] for v in drones.values()])
thr_dur = 0.5 * (durs.min() + durs.max())     # 2 grupos claros (RouteA/RouteBhalf)
for d, v in drones.items():
    v['route'] = 'RouteA' if v['dur'] > thr_dur else 'RouteBhalf'

# ── Acumula componentes de risco por posicao East, por rota ───────────────
# r_unc,r_map,r_vid sao BRUTOS (pre-peso); r_tot e a soma ponderada (w=1/3
# cada).  Fracao do mapa = r_map/(r_unc+r_map+r_vid) — bounded [0,1] e, com
# pesos iguais, identica a (w_map*r_map)/r_tot.
acc = {rt: dict(E=[], rmap=[], rvid=[], rsum=[], rtot=[]) for rt in ROUTES}
for r in csv.DictReader(open(slot_csv)):
    d = int(r['drone_id'])
    if d not in drones:
        continue
    runc = float(r['r_unc']); rmap = float(r['r_map']); rvid = float(r['r_vid'])
    rsum = runc + rmap + rvid
    if rsum <= 0:
        continue
    v   = drones[d]
    dur = v['dur'] if v['dur'] > 0 else 1.0
    frac = min(max((float(r['slot']) - v['entry']) / dur, 0.0), 1.0)
    rt  = v['route']
    E   = ROUTES[rt]['startE'] + frac * (ROUTES[rt]['goalE'] - ROUTES[rt]['startE'])
    acc[rt]['E'].append(E)
    acc[rt]['rmap'].append(rmap)
    acc[rt]['rvid'].append(rvid)
    acc[rt]['rsum'].append(rsum)
    acc[rt]['rtot'].append(float(r['r_tot']))

# ── Binning por East, por rota ────────────────────────────────────────────
NB = 40
def binned(rt):
    E    = np.array(acc[rt]['E'])
    rmap = np.array(acc[rt]['rmap'])
    rvid = np.array(acc[rt]['rvid'])
    rsum = np.array(acc[rt]['rsum'])
    ratio = 100.0 * rmap / rsum            # fracao do mapa, em [0,100]%
    edges = np.linspace(ROUTES[rt]['startE'], ROUTES[rt]['goalE'], NB + 1)
    mids  = 0.5 * (edges[:-1] + edges[1:])
    idx   = np.digitize(E, edges)
    fr_mean = np.array([np.mean(ratio[idx == i])           if np.any(idx == i) else np.nan for i in range(1, NB+1)])
    fr_lo   = np.array([np.percentile(ratio[idx == i], 25) if np.any(idx == i) else np.nan for i in range(1, NB+1)])
    fr_hi   = np.array([np.percentile(ratio[idx == i], 75) if np.any(idx == i) else np.nan for i in range(1, NB+1)])
    rm_mean = np.array([np.mean(rmap[idx == i])            if np.any(idx == i) else np.nan for i in range(1, NB+1)])
    rv_mean = np.array([np.mean(rvid[idx == i])            if np.any(idx == i) else np.nan for i in range(1, NB+1)])
    return mids, fr_mean, fr_lo, fr_hi, rm_mean, rv_mean

# ── Figura ────────────────────────────────────────────────────────────────
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 5.5), constrained_layout=True)
fig.suptitle(f'Fracao espacial do risco (r_map/r_tot) ao longo das rotas — '
             f'{args.scenario}, {args.policy}, K={args.K} C={args.C}',
             fontsize=13, fontweight='bold')

for rt, p in ROUTES.items():
    mids, frm, flo, fhi, rmm, rvm = binned(rt)
    ax1.fill_between(mids, flo, fhi, color=p['color'], alpha=0.18)
    ax1.plot(mids, frm, color=p['color'], lw=2.4, marker='o', ms=4, label=rt)
    ax2.plot(mids, rmm, color=p['color'], lw=2.4, label=f'{rt} — r_map (solo)')
    ax2.plot(mids, rvm, color=p['color'], lw=1.6, ls='--', label=f'{rt} — r_vid (AoI)')

for ax in (ax1, ax2):
    ax.axvline(BS_EAST, color='cyan', lw=1.4, ls=':', label='gNB (E=5000)')
    ax.axvspan(*HOTSPOT_E, color='orange', alpha=0.10)
    ax.set_xlabel('East (m)  — posicao ao longo da rota')
    ax.grid(True, ls='--', alpha=0.4)

ax1.text(np.mean(HOTSPOT_E), ax1.get_ylim()[1]*0.05, 'zona de hotspots',
         color='darkorange', fontsize=8.5, ha='center')
ax1.set_ylabel('r_map / r_tot  (%)')
ax1.set_title('Fracao do risco devida ao MAPA DE SOLO\n(o que o surrogate Jhat ignora)',
              fontsize=11)
ax1.set_ylim(0, 100)
ax1.legend(fontsize=9, loc='lower right')

ax2.set_ylabel('Risco bruto (unidades do modelo)')
ax2.set_yscale('log')
ax2.set_title('Componentes brutos: r_map solo (cheia) vs r_vid AoI (tracejada)', fontsize=11)
ax2.legend(fontsize=8, loc='upper left', ncol=1)

out = os.path.join(OUT_DIR, f'rmap_fraction_{args.scenario}_{args.policy}_K{args.K}_C{args.C}.png')
fig.savefig(out, dpi=150, bbox_inches='tight')
print(f'Saved: {out}')
