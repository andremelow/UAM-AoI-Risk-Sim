%% RERUN_RRSW  Re-runs only round-robin-sw across the full (K,C) grid.
%  Use after fixing policy_round_robin_sw.m to regenerate affected CSVs.
%
%  Usage (headless):
%    matlab -nodisplay -r "cd /mnt/stg/wrk/interlagos/simulator/v3/uam-sim-new; rerun_mwsw; exit"

setup_paths();

K_values        = [1 2 4 8 16 32];
capacity_values = [2 4 8 16 32 64];
T_STEADY        = 900;
POLICY          = 'max-weight-sw';

SIM_ROOT = fileparts(mfilename('fullpath'));
OUT_ROOT = fullfile(SIM_ROOT, 'csv_export', 'capacity_15min_sw_nsw_assimetrico');

ts      = datestr(now, 'yyyymmdd_HHMMSS'); %#ok<TNOW1,DATST>
logFile = fullfile(OUT_ROOT, sprintf('rerun_mwsw_%s.log', ts));
diary(logFile); diary on;

fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  RERUN round-robin-sw  (fix: chosen < n_elig guard)\n');
fprintf('  K: %s\n', num2str(K_values));
fprintf('  C: %s\n', num2str(capacity_values));
fprintf('  Started: %s\n', datestr(now)); %#ok<TNOW1,DATST>
fprintf('═══════════════════════════════════════════════════════════════\n\n');

cfg_base           = build_base_config_assimetrico_local();
cfg_base.headless  = true;

scenarios = define_scenarios_local();
n_sc      = numel(scenarios);

nK = numel(K_values);
nC = numel(capacity_values);
n_total = nK * nC * n_sc;

combos(n_total) = struct('K',0,'C',0,'sc',[]);
idx = 0;
for ki = 1:nK
    for ci = 1:nC
        for si = 1:n_sc
            idx = idx + 1;
            combos(idx).K  = K_values(ki);
            combos(idx).C  = capacity_values(ci);
            combos(idx).sc = scenarios(si);
        end
    end
end

nPhysCores = feature('numcores');
nTarget    = max(1, floor(nPhysCores / 4));
cluster    = parcluster('local');
if cluster.NumWorkers ~= nTarget
    cluster.NumWorkers = nTarget;
    saveProfile(cluster);
end
nWorkers = min(n_total, cluster.NumWorkers);
pool = gcp('nocreate');
if isempty(pool)
    parpool('local', nWorkers);
end
fprintf('[PARALLEL] %d workers\n', nWorkers);

all_rows_cell = cell(n_total, 1);
sw_t0 = tic;

parfor idx = 1:n_total
    c       = combos(idx);
    outDir  = fullfile(OUT_ROOT, sprintf('K%d', c.K), ...
                       sprintf('C%02d', c.C), c.sc.label);
    system(sprintf('mkdir -p "%s"', outDir));

    cfg = build_capacity_run_cfg_local(cfg_base, c.C, c.K, T_STEADY, c.sc, outDir);
    cfg.schedulingPolicy = POLICY;

    fprintf('[K=%2d C=%2d %s] iniciando...\n', c.K, c.C, c.sc.label);
    t0 = tic;
    try
        results = run_sim(cfg);
        dt = toc(t0);
        fprintf('[K=%2d C=%2d %s] OK  %.0f s\n', c.K, c.C, c.sc.label, dt);
        all_rows_cell{idx} = build_row_local(results, c.C, c.K, c.sc, POLICY);
    catch ME
        dt = toc(t0);
        fprintf('[K=%2d C=%2d %s] FAIL %.0f s: %s\n', c.K, c.C, c.sc.label, dt, ME.message);
        all_rows_cell{idx} = struct('capacity',c.C,'K',c.K,'scenario',c.sc.label, ...
            'routing_mode',c.sc.routing_mode,'num_routes',numel(c.sc.routes), ...
            'policy',POLICY,'error',ME.message,'mean_h1',NaN,'mean_h2',NaN, ...
            'mean_r_sys',NaN,'Jhat_emp',NaN);
    end
end

fprintf('\n[RERUN_RRSW] Concluído em %.2f h\n', toc(sw_t0)/3600);

% Rebuild global summary by merging with existing non-rrsw rows
existing_all = fullfile(OUT_ROOT, 'sweep_capacity_summary_all.csv');
new_all      = fullfile(OUT_ROOT, 'sweep_capacity_summary_all.csv');

T_old = readtable(existing_all, 'TextType','string');
T_old = T_old(T_old.policy ~= "round-robin-sw", :);

valid = ~cellfun(@isempty, all_rows_cell);
new_rows = all_rows_cell(valid);
T_new = rows_to_table_local(new_rows);

T_merged = [T_old; T_new];
writetable(T_merged, new_all);
fprintf('[RERUN_RRSW] Global summary updated → %s  (%d rows)\n', new_all, height(T_merged));

diary off;


%% ── Local helpers (mirror experiment_capacity_15min_sw_nsw_assimetrico) ──

function cfg = build_base_config_assimetrico_local()
cfg = build_scenario_assimetrico();
cfg.w_unc = 1/3; cfg.w_map = 1/3; cfg.w_vid = 1/3;
cfg.arrivalModel     = 'capacity_drain';
cfg.corridorCapacity = 1;
cfg.steady_duration  = 1200;
cfg.minStartDelay    = 2;
cfg.videoModel = 'atomic'; cfg.fps = 30; cfg.L_vid = 100; cfg.L_c2 = 1;
cfg.w_coh = 5; cfg.L_nom = 0.01; cfg.p_c2 = 0.80; cfg.p_vid = 0.80;
cfg.rho_0 = 0.01;
cfg.csvExport = true;
cfg.csvDir    = 'csv_export/capacity_15min_sw_nsw_assimetrico';
end

function scenarios = define_scenarios_local()
r1.numDrones=1; r1.start=[200,-1500]; r1.goal=[500,1500];  r1.label='Route2';
r2.numDrones=1; r2.start=[-500,1500]; r2.goal=[-600,-1500]; r2.label='Route3';
s1.label='2r_static'; s1.routing_mode='static'; s1.routes=[r1,r2];
scenarios = s1;
end

function cfg = build_capacity_run_cfg_local(cfg_base, C, K, T_steady, sc, outDir)
cfg = cfg_base;
nR = numel(sc.routes); routes = sc.routes;
C_per = floor(C/nR); C_rem = C - C_per*nR;
flightTime = norm(routes(1).goal - routes(1).start) / cfg.speedVal;
for ri = 1:nR
    C_r = C_per + (ri <= C_rem);
    spacing = flightTime / max(C_r,1);
    routes(ri).numDrones = C_r + floor(T_steady/spacing);
end
cfg.routes = routes;
cfg.corridorLength   = norm(routes(1).goal - routes(1).start);
cfg.flightTime       = flightTime;
cfg.corridorCapacity = C;
cfg.steady_duration  = T_steady;
cfg.dronesPerSlot    = K;
cfg.csvDir           = outDir;
cfg.routing.mode     = sc.routing_mode;
N_total = sum([routes.numDrones]);
spacing_min = flightTime / max(C,1);
cfg.maxStartDelay = cfg.minStartDelay + (N_total-1)*spacing_min + 10;
cfg.H_max=[]; cfg.omega=[]; cfg.beta_s=[]; cfg.beta_v=[];
end

function row = build_row_local(results, C, K, sc, pol)
cfg = results.cfg; N = results.numDrones;
p_emp = zeros(2*N, 1);
for u = 1:N
    tot_c2  = results.slotsC2(u)  + results.slotsC2Fail(u);
    tot_vid = results.slotsVid(u) + results.slotsVidFail(u);
    p_emp(2*(u-1)+1) = results.slotsC2(u)  / max(tot_c2,1);
    p_emp(2*(u-1)+2) = results.slotsVid(u) / max(tot_vid,1);
end
LB_legacy = compute_lower_bound(cfg, p_emp);
xs_final = arrayfun(@(u) ifempty_v(results.xsVal{u},0), 1:N);
xv_final = arrayfun(@(u) ifempty_v(results.xvVal{u},0), 1:N);
row.capacity=C; row.K=K; row.scenario=sc.label; row.routing_mode=sc.routing_mode;
row.num_routes=numel(sc.routes); row.num_drones_total=N; row.policy=pol;
row.mean_h1=mean(results.h1MeanV); row.mean_h2=mean(results.h2MeanV);
row.mean_r_sys=results.Rsys_emp; row.peak_r_sys=max(results.rSysData);
row.Jhat_emp=results.Jhat_emp; row.J_LB_doc=results.J_LB; row.rho_emp=results.rho_emp;
row.LB_legacy=LB_legacy;
row.qbar_s_mean=mean(cfg.qbar_s(1:min(N,end)));
row.qbar_v_mean=mean(cfg.qbar_v(1:min(N,end)));
row.q_emp_s_mean=mean(results.q_emp_s); row.q_emp_v_mean=mean(results.q_emp_v);
row.xs_final_mean=mean(xs_final); row.xv_final_mean=mean(xv_final);
row.n_s=cfg.n_s; row.n_v=cfg.n_v; row.error='';
end

function T = rows_to_table_local(rows)
fields = {'capacity','K','scenario','routing_mode','num_routes','num_drones_total','policy', ...
          'mean_h1','mean_h2','mean_r_sys','peak_r_sys','Jhat_emp','J_LB_doc','rho_emp', ...
          'LB_legacy','qbar_s_mean','qbar_v_mean','q_emp_s_mean','q_emp_v_mean', ...
          'xs_final_mean','xv_final_mean','n_s','n_v'};
n = numel(rows);
data = cell(n, numel(fields));
for ri = 1:n
    r = rows{ri};
    for fi = 1:numel(fields)
        data{ri,fi} = r.(fields{fi});
    end
end
T = cell2table(data, 'VariableNames', fields);
end

function v = ifempty_v(x, d)
if isempty(x), v = d; else, v = x(end); end
end
