import pandas as pd, numpy as np, matplotlib.pyplot as plt, os
df=pd.read_csv('csv_export_nv_sweep/nv_sweep.csv')
M=pd.read_csv('csv_export_mw_otimo_a23_gop_mw3/sweep_summary_all.csv')
PF={}
for C in [16,48]:
  for K in [4,16]:
    PF[(C,K)]=M[(M.capacity==C)&(M.K==K)&(M.policy=='pf-classic-sw')].mean_r_sys.values[0]
fig,axes=plt.subplots(1,4,figsize=(20,4.6),constrained_layout=True)
for ax,(C,K) in zip(axes,[(16,4),(16,16),(48,4),(48,16)]):
  d=df[(df.C==C)&(df.K==K)].sort_values('n_v_scale')
  ax.plot(d.n_v_scale,d.R_sys,'-o',color='#E53935',lw=2,ms=7,label='MW (mw3-sn)')
  ax.axhline(PF[(C,K)],color='#2196F3',ls='--',lw=2,label=f'PF = {PF[(C,K)]:.0f}')
  imin=d.R_sys.idxmin(); s_opt=d.loc[imin,'n_v_scale']; r_opt=d.loc[imin,'R_sys']
  ax.plot(s_opt,r_opt,'*',color='k',ms=16,zorder=5,label=f'ótimo: scale={int(s_opt)}, R={r_opt:.0f}')
  ax.set_xscale('log'); ax.set_xticks(d.n_v_scale); ax.get_xaxis().set_major_formatter(plt.matplotlib.ticker.ScalarFormatter())
  ax.minorticks_off()
  ax.set_title(f'C={C}, K={K}',fontweight='bold'); ax.set_xlabel('n_v_scale (boost de n_v)')
  ax.set_ylabel('R_sys'); ax.grid(True,ls='--',alpha=0.4); ax.legend(fontsize=8)
fig.suptitle('Max-Weight: risco vs rebalanceamento de n_v (vs PF) — 2r_a23',fontsize=13)
os.makedirs('csv_export_nv_sweep/plots',exist_ok=True)
fig.savefig('csv_export_nv_sweep/plots/nv_sweep.png',dpi=150,bbox_inches='tight')
print('saved')
