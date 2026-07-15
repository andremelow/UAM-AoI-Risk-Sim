function run_nvratio_test()
%% RUN_NVRATIO_TEST
%  Testa a normalização por densidade: n_v = k*n_s (k=0.25), na política
%  Max-Weight (mw3-sn), na grade completa K{1,2,4,16} x C{8,16,32,48}, 2r_a23/
%  GOP. Responde: com UM único k acoplado a n_s, o Max-Weight bate o PF em
%  todo C? Saída: csv_export_nvratio/nvratio.csv
setup_paths();
SIM_ROOT=fileparts(mfilename('fullpath'));
OUT=fullfile(SIM_ROOT,'csv_export_nvratio'); if ~exist(OUT,'dir'), mkdir(OUT); end
T=900; K_RATIO=0.25;

cfg0=build_scenario_variavel();
cfg0.w_unc=1/3; cfg0.w_map=1/3; cfg0.w_vid=1/3;
cfg0.arrivalModel='capacity_drain'; cfg0.corridorCapacity=1;
cfg0.steady_duration=T; cfg0.minStartDelay=2;
cfg0.videoModel='gop'; cfg0.fps=30; cfg0.GOP=15; cfg0.L_I=30; cfg0.L_P=3;
cfg0.L_vid=5; cfg0.L_c2=1; cfg0.w_coh=5; cfg0.L_nom=0.01;
cfg0.p_c2=0.80; cfg0.p_vid=0.80; cfg0.rho_0=0.01;
cfg0.csvExport=false; cfg0.headless=true;

rA.numDrones=1; rA.start=[1000,1000]; rA.goal=[1000,9000];
rBh.numDrones=1; rBh.start=[-350,9750]; rBh.goal=[-700,14000];
routes0=[rA,rBh]; frac=[2/3,1/3];

Ks=[1 2 4 16]; Cs=[8 16 32 48];
combos={}; for K=Ks, for C=Cs, combos{end+1}=struct('K',K,'C',C); end, end %#ok<AGROW>
n=numel(combos);
pool=gcp('nocreate'); if isempty(pool), parpool('local',6); end

rows=cell(n,1);
parfor i=1:n
  c=combos{i};
  cfg=cfg0; routes=routes0;
  cp=max(1,round(frac/sum(frac)*c.C)); cp(end)=max(1,c.C-sum(cp(1:end-1)));
  ft=arrayfun(@(r) norm(r.goal-r.start)/cfg.speedVal, routes);
  for ri=1:2, sp=ft(ri)/max(cp(ri),1); routes(ri).numDrones=cp(ri)+floor(T/sp); end
  cfg.routes=routes; cfg.corridorLength=norm(routes(1).goal-routes(1).start);
  cfg.flightTime=ft(1); cfg.corridorCapacity=c.C; cfg.dronesPerSlot=c.K;
  cfg.routing.mode='static'; cfg.arrivalModel='capacity_drain_routes';
  le=arrayfun(@(ri) cfg.minStartDelay+(routes(ri).numDrones-1)*ft(ri)/max(cp(ri),1),1:2);
  cfg.maxStartDelay=max(le)+10; cfg.H_max=[]; cfg.omega=[]; cfg.beta_s=[]; cfg.beta_v=[];
  cfg.n_v_ratio=K_RATIO; cfg.schedulingPolicy='mw3-sn';
  try
    r=run_sim(cfg);
    rows{i}=sprintf('%d,%d,%.3f,%.3f,%.3f,%.3f,%.5f,%.6e',c.K,c.C,r.cfg.n_s,r.cfg.n_v, ...
      mean(r.h1MeanV),mean(r.h2MeanV),mean(r.q_emp_v),r.Rsys_emp);
    fprintf('[NVR] K=%2d C=%2d | n_s=%.1f n_v=%.1f h2=%.0f q_v=%.4f Rsys=%.0f\n', ...
      c.K,c.C,r.cfg.n_s,r.cfg.n_v,mean(r.h2MeanV),mean(r.q_emp_v),r.Rsys_emp);
  catch ME
    rows{i}=sprintf('%d,%d,NaN,NaN,NaN,NaN,NaN,NaN',c.K,c.C);
  end
end
fid=fopen(fullfile(OUT,'nvratio.csv'),'w');
fprintf(fid,'K,C,n_s,n_v,mean_h1,mean_h2,q_v,R_sys\n');
for i=1:n, fprintf(fid,'%s\n',rows{i}); end
fclose(fid);
fprintf('[NVR] CSV -> %s (k=%.2f)\n', fullfile(OUT,'nvratio.csv'), K_RATIO);
end
