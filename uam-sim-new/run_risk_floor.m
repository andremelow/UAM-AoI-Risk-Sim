function run_risk_floor()
%% RUN_RISK_FLOOR
%  Computa o RISCO MÍNIMO TEÓRICO do cenário (piso de risco) por (cenário, C),
%  avaliando o modelo de risco com a AoI no piso físico sobre as trajetórias
%  reais. Independe de política/K (posições fixadas pelo roteamento), então
%  roda 1x por (cenário, C) com round-robin-sw, K=8, computeRiskFloor=true.
%
%  Dois pisos (ambos lower bounds de R_sys):
%    abs  = updates perfeitos/instantâneos (h->0 => r=r_min ; z->0 => R_vid=0)
%    real = limitado por canal/quadro (h_s=1/p_s ; z=L => R_vid=max(L/H_nom-1,0))
%
%  Saída: csv_export_mw_otimo_a23_gop_final/risk_floor.csv

setup_paths();
SIM_ROOT = fileparts(mfilename('fullpath'));
OUT = fullfile(SIM_ROOT, 'csv_export_mw_otimo_a23_gop_final', 'risk_floor.csv');
T_STEADY = 900;

cfg_base = build_scenario_variavel();
cfg_base.w_unc=1/3; cfg_base.w_map=1/3; cfg_base.w_vid=1/3;
cfg_base.arrivalModel='capacity_drain'; cfg_base.corridorCapacity=1;
cfg_base.steady_duration=T_STEADY; cfg_base.minStartDelay=2;
cfg_base.videoModel='gop'; cfg_base.fps=30; cfg_base.GOP=15; cfg_base.L_I=30; cfg_base.L_P=3;
cfg_base.L_vid=5; cfg_base.L_c2=1; cfg_base.w_coh=5; cfg_base.L_nom=0.01;
cfg_base.p_c2=0.80; cfg_base.p_vid=0.80; cfg_base.rho_0=0.01;
cfg_base.csvExport=false; cfg_base.headless=true;
cfg_base.computeRiskFloor=true;          % <-- liga o cálculo do piso

scenarios = define_sc();
caps = [2 4 8 16 32 48 64];
combos = {};
for si=1:numel(scenarios)
    for C=caps
        combos{end+1} = struct('sc',scenarios(si),'C',C); %#ok<AGROW>
    end
end
n = numel(combos);

pool = gcp('nocreate');
if isempty(pool), parpool('local', 6); end

rows = cell(n,1);
parfor k=1:n
    c = combos{k};
    cfg = build_cfg(cfg_base, c.sc, c.C, T_STEADY);
    cfg.schedulingPolicy = 'round-robin-sw';
    cfg.dronesPerSlot = 8;
    try
        r = run_sim(cfg);
        fa = r.rSysFloorAbs;  fr = r.rSysFloorReal;
        wf = max(1, round(0.2*numel(fa)));
        abs_pk  = max(fa(wf:end));   real_pk = max(fr(wf:end));
        rows{k} = sprintf('%s,%d,%.6e,%.6e,%.6e,%.6e,%.6e', c.sc.label, c.C, ...
            r.Rsys_floor_abs, r.Rsys_floor_real, abs_pk, real_pk, r.Rsys_emp);
        fprintf('[FLOOR] %-10s C=%2d  abs=%.1f real=%.1f abs_pk=%.1f (Rsys=%.1f)\n', ...
            c.sc.label, c.C, r.Rsys_floor_abs, r.Rsys_floor_real, abs_pk, r.Rsys_emp);
    catch ME
        rows{k} = sprintf('%s,%d,NaN,NaN,NaN,NaN,NaN', c.sc.label, c.C);
        fprintf('[FLOOR] %-10s C=%2d FAILED: %s\n', c.sc.label, c.C, ME.message);
    end
end

fid = fopen(OUT,'w');
fprintf(fid,'scenario,capacity,Rsys_floor_abs,Rsys_floor_real,Rsys_floor_abs_peak,Rsys_floor_real_peak,Rsys_emp_ref\n');
for k=1:n, fprintf(fid,'%s\n',rows{k}); end
fclose(fid);
fprintf('[FLOOR] CSV -> %s\n', OUT);
end


function scenarios = define_sc()
rA.numDrones=1; rA.start=[1000,1000]; rA.goal=[1000,9000]; rA.label='RouteA';
rB.numDrones=1; rB.start=[-500,12000]; rB.goal=[-600,14000]; rB.label='RouteB';
rBh.numDrones=1; rBh.start=[-350,9750]; rBh.goal=[-700,14000]; rBh.label='RouteBhalf';
s1.label='2r_static'; s1.routing_mode='static'; s1.routes=[rA,rB];  s1.route_frac=[1/2,1/2];
s2.label='2r_a23';    s2.routing_mode='static'; s2.routes=[rA,rBh]; s2.route_frac=[2/3,1/3];
scenarios=[s1,s2];
end


function cfg = build_cfg(cfg_base, sc, C, T_steady)
cfg=cfg_base; routes=sc.routes; nR=numel(routes);
frac=sc.route_frac(:)'/sum(sc.route_frac);
cp=max(1,round(frac*C)); cp(end)=max(1,C-sum(cp(1:end-1)));
ft=arrayfun(@(r) norm(r.goal(:)'-r.start(:)')/cfg.speedVal, routes);
for ri=1:nR, sp=ft(ri)/max(cp(ri),1); routes(ri).numDrones=cp(ri)+floor(T_steady/sp); end
cfg.routes=routes; cfg.corridorLength=norm(routes(1).goal(:)'-routes(1).start(:)');
cfg.flightTime=ft(1); cfg.corridorCapacity=C; cfg.steady_duration=T_steady;
cfg.routing.mode=sc.routing_mode;
if max(ft)/min(ft)>1.5, cfg.arrivalModel='capacity_drain_routes'; end
le=arrayfun(@(ri) cfg.minStartDelay+(routes(ri).numDrones-1)*ft(ri)/max(cp(ri),1),1:nR);
cfg.maxStartDelay=max(le)+10; cfg.H_max=[]; cfg.omega=[]; cfg.beta_s=[]; cfg.beta_v=[];
end
