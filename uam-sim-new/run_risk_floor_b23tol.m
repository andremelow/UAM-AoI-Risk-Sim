function run_risk_floor_b23tol()
%% RUN_RISK_FLOOR_B23TOL
%  Piso de risco teórico (abs + realizável, média e pico) para o cenário
%  2r_b23 (1/3 RouteA + 2/3 RouteBhalf) COM takeoff/landing, por C.
%  Pisos fase-aware (usam w_tol nas rampas). Saída:
%  csv_export_mw_otimo_a23_gop_b23topt_ch/risk_floor.csv
setup_paths();
OUT = fullfile(fileparts(mfilename('fullpath')), ...
               'csv_export_mw_otimo_a23_gop_b23topt_ch', 'risk_floor.csv');
T = 900; Cs = [8 16 32 48 64];

cfg0 = build_scenario_variavel();
cfg0.w_unc=1/3; cfg0.w_map=1/3; cfg0.w_vid=1/3;
cfg0.arrivalModel='capacity_drain'; cfg0.corridorCapacity=1;
cfg0.steady_duration=T; cfg0.minStartDelay=2;
cfg0.videoModel='gop'; cfg0.fps=30; cfg0.GOP=15; cfg0.L_I=30; cfg0.L_P=3;
cfg0.L_vid=5; cfg0.L_c2=1; cfg0.w_coh=5; cfg0.L_nom=0.01;
cfg0.p_c2=0.80; cfg0.p_vid=0.80; cfg0.rho_0=0.01;
cfg0.csvExport=false; cfg0.headless=true;
cfg0.altProfile='takeoff_landing'; cfg0.takeoffDist=500; cfg0.cruiseAlt=60;
cfg0.w_tol_vid=0.80; cfg0.w_tol_unc=0.10; cfg0.w_tol_map=0.10;
cfg0.computeRiskFloor=true;

rA.numDrones=1; rA.start=[1000,1000]; rA.goal=[1000,9000];
rBh.numDrones=1; rBh.start=[-350,9750]; rBh.goal=[-700,14000];
routes0=[rA,rBh]; frac=[1/3,2/3];

pool=gcp('nocreate'); if isempty(pool), parpool('local',5); end
rows=cell(numel(Cs),1);
parfor k=1:numel(Cs)
    C=Cs(k); cfg=cfg0; routes=routes0;
    cp=max(1,round(frac/sum(frac)*C)); cp(end)=max(1,C-sum(cp(1:end-1)));
    ft=arrayfun(@(r) norm(r.goal-r.start)/cfg.speedVal, routes);
    for ri=1:2, sp=ft(ri)/max(cp(ri),1); routes(ri).numDrones=cp(ri)+floor(T/sp); end
    cfg.routes=routes; cfg.corridorLength=norm(routes(1).goal-routes(1).start);
    cfg.flightTime=ft(1); cfg.corridorCapacity=C; cfg.dronesPerSlot=4;
    cfg.routing.mode='static'; cfg.arrivalModel='capacity_drain_routes';
    le=arrayfun(@(ri) cfg.minStartDelay+(routes(ri).numDrones-1)*ft(ri)/max(cp(ri),1),1:2);
    cfg.maxStartDelay=max(le)+10; cfg.H_max=[]; cfg.omega=[]; cfg.beta_s=[]; cfg.beta_v=[];
    cfg.schedulingPolicy='pf-classic-sw';
    r=run_sim(cfg);
    fa=r.rSysFloorAbs; fr=r.rSysFloorReal; wf=max(1,round(0.2*numel(fa)));
    rows{k}=sprintf('2r_b23,%d,%.6e,%.6e,%.6e,%.6e,%.6e', C, ...
        r.Rsys_floor_abs, r.Rsys_floor_real, max(fa(wf:end)), max(fr(wf:end)), r.Rsys_emp);
    fprintf('[FLOOR] C=%2d abs=%.0f real=%.0f abs_pk=%.0f\n', C, ...
        r.Rsys_floor_abs, r.Rsys_floor_real, max(fa(wf:end)));
end
fid=fopen(OUT,'w');
fprintf(fid,'scenario,capacity,Rsys_floor_abs,Rsys_floor_real,Rsys_floor_abs_peak,Rsys_floor_real_peak,Rsys_emp_ref\n');
for k=1:numel(Cs), fprintf(fid,'%s\n',rows{k}); end
fclose(fid);
fprintf('[FLOOR] -> %s\n', OUT);
end
