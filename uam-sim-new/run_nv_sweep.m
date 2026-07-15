function run_nv_sweep()
%% RUN_NV_SWEEP
%  Varre o coeficiente surrogado de vídeo n_v (via n_v_scale) na política
%  Max-Weight (mw3-sn), no cenário 2r_a23/GOP, para achar onde ela deixa de
%  starvar o vídeo e passa a bater o PF. Pontos (C,K) onde o PF vencia.
%
%  Saída: csv_export_nv_sweep/nv_sweep.csv
%   colunas: n_v_scale, C, K, n_s, n_v, mean_h1, mean_h2, q_v, R_sys

setup_paths();
SIM_ROOT = fileparts(mfilename('fullpath'));
OUTDIR = fullfile(SIM_ROOT, 'csv_export_nv_sweep');
if ~exist(OUTDIR,'dir'), mkdir(OUTDIR); end
T_STEADY = 900;

scales = [1 2 5 10 20 50 100];
Cs     = [16 48];
Ks     = [4 16];

cfg_base = build_scenario_variavel();
cfg_base.w_unc=1/3; cfg_base.w_map=1/3; cfg_base.w_vid=1/3;
cfg_base.arrivalModel='capacity_drain'; cfg_base.corridorCapacity=1;
cfg_base.steady_duration=T_STEADY; cfg_base.minStartDelay=2;
cfg_base.videoModel='gop'; cfg_base.fps=30; cfg_base.GOP=15; cfg_base.L_I=30; cfg_base.L_P=3;
cfg_base.L_vid=5; cfg_base.L_c2=1; cfg_base.w_coh=5; cfg_base.L_nom=0.01;
cfg_base.p_c2=0.80; cfg_base.p_vid=0.80; cfg_base.rho_0=0.01;
cfg_base.csvExport=false; cfg_base.headless=true;

% cenário 2r_a23
rA.numDrones=1; rA.start=[1000,1000]; rA.goal=[1000,9000];
rBh.numDrones=1; rBh.start=[-350,9750]; rBh.goal=[-700,14000];
routes0=[rA,rBh]; frac=[2/3,1/3];

combos = {};
for s=scales, for C=Cs, for K=Ks
    combos{end+1}=struct('s',s,'C',C,'K',K); %#ok<AGROW>
end, end, end
n=numel(combos);

pool=gcp('nocreate'); if isempty(pool), parpool('local',6); end

rows=cell(n,1);
parfor i=1:n
    c=combos{i};
    cfg=cfg_base; routes=routes0; nR=2;
    cp=max(1,round(frac/sum(frac)*c.C)); cp(end)=max(1,c.C-sum(cp(1:end-1)));
    ft=arrayfun(@(r) norm(r.goal-r.start)/cfg.speedVal, routes);
    for ri=1:nR, sp=ft(ri)/max(cp(ri),1); routes(ri).numDrones=cp(ri)+floor(T_STEADY/sp); end
    cfg.routes=routes; cfg.corridorLength=norm(routes(1).goal-routes(1).start);
    cfg.flightTime=ft(1); cfg.corridorCapacity=c.C; cfg.dronesPerSlot=c.K;
    cfg.routing.mode='static'; cfg.arrivalModel='capacity_drain_routes';
    le=arrayfun(@(ri) cfg.minStartDelay+(routes(ri).numDrones-1)*ft(ri)/max(cp(ri),1),1:nR);
    cfg.maxStartDelay=max(le)+10; cfg.H_max=[]; cfg.omega=[]; cfg.beta_s=[]; cfg.beta_v=[];
    cfg.n_v_scale=c.s;
    cfg.schedulingPolicy='mw3-sn';
    try
        r=run_sim(cfg);
        rows{i}=sprintf('%d,%d,%d,%.4f,%.4f,%.3f,%.3f,%.5f,%.6e', ...
            c.s, c.C, c.K, r.cfg.n_s, r.cfg.n_v, mean(r.h1MeanV), mean(r.h2MeanV), ...
            mean(r.q_emp_v), r.Rsys_emp);
        fprintf('[NV] scale=%3d C=%2d K=%2d | n_v=%.3f h2=%.0f q_v=%.4f Rsys=%.0f\n', ...
            c.s, c.C, c.K, r.cfg.n_v, mean(r.h2MeanV), mean(r.q_emp_v), r.Rsys_emp);
    catch ME
        rows{i}=sprintf('%d,%d,%d,NaN,NaN,NaN,NaN,NaN,NaN', c.s,c.C,c.K);
        fprintf('[NV] scale=%d C=%d K=%d FAILED: %s\n', c.s,c.C,c.K,ME.message);
    end
end

fid=fopen(fullfile(OUTDIR,'nv_sweep.csv'),'w');
fprintf(fid,'n_v_scale,C,K,n_s,n_v,mean_h1,mean_h2,q_v,R_sys\n');
for i=1:n, fprintf(fid,'%s\n',rows{i}); end
fclose(fid);
fprintf('[NV] CSV -> %s\n', fullfile(OUTDIR,'nv_sweep.csv'));
end
