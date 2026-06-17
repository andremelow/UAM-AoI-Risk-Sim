function experiment_capacity_debug(varargin)
%% EXPERIMENT_CAPACITY_DEBUG
%  Cenário de debug: duas rotas curtas (~2 km) em zonas SNR fixas.
%    Route A — próxima à BS  [N=200,E=4000]→[N=300,E=6000]   SNR ≈ 35-40 dB
%    Route B — longe + hotspots [N=-500,E=12000]→[N=-600,E=14000] SNR ≈ 6-8 dB
%  gNB: [East=5000, North=0, Alt=30 m].  Path loss: UMa.
%
%  Objetivo: PF deve favorecer Route A (melhor SNR) e negligenciar Route B,
%  acumulando risco. Max-weight deve detectar h_s crescendo em Route B e
%  realocar slots, dominando PF e RR.
%
%  Sweep: K ∈ {1,2,4,8,16,32}  ×  C ∈ {2,4,8,16,32,64}  ×  1 cenário  ×  4 políticas
%  Total: 144 runs   T_steady = 300 s
%
%  Usage:
%    experiment_capacity_debug()
%    experiment_capacity_debug('--yes')
%    experiment_capacity_debug('skip-smoke')
%    experiment_capacity_debug('--C','2,4,8')
%    experiment_capacity_debug('--K','4,8,32')

setup_paths();

%% ── Argument parsing ─────────────────────────────────────────────────
skip_smoke   = false;
auto_confirm = false;
smoke_only   = false;
cap_override = [];
k_override   = [];

ai = 1;
while ai <= numel(varargin)
    switch varargin{ai}
        case 'skip-smoke',   skip_smoke   = true;
        case '--smoke-only', smoke_only   = true;
        case '--yes',        auto_confirm = true;
        case '--C'
            ai = ai + 1;
            if ai <= numel(varargin)
                cap_override = str2double(strsplit(varargin{ai}, ','));
            end
        case '--K'
            ai = ai + 1;
            if ai <= numel(varargin)
                k_override = str2double(strsplit(varargin{ai}, ','));
            end
    end
    ai = ai + 1;
end

%% ── Sweep axes ───────────────────────────────────────────────────────
if isempty(cap_override)
    capacity_values = [2, 4, 8, 16, 32, 64];
else
    capacity_values = cap_override;
end
if isempty(k_override)
    K_values = [1, 2, 4, 8, 16, 32];
else
    K_values = k_override;
end
policies  = {'round-robin-sw', 'pf-classic-sw', 'max-weight-sw', 'aoi-sw'};
T_STEADY  = 300;

SIM_ROOT = fileparts(mfilename('fullpath'));
OUT_ROOT = fullfile(SIM_ROOT, 'csv_export', 'capacity_debug');

logFile = fullfile(SIM_ROOT, sprintf('experiment_debug_%s.log', datestr(now,'yyyymmdd_HHMMSS'))); %#ok<TNOW1,DATST>
diary(logFile);

scenarios  = define_capacity_scenarios_debug();
n_sc       = numel(scenarios);
total_runs = numel(K_values) * numel(capacity_values) * n_sc * numel(policies);

fprintf('\n═══════════════════════════════════════════════════════════════\n');
fprintf('  EXPERIMENT_CAPACITY_DEBUG\n');
fprintf('  %s\n', datestr(now)); %#ok<TNOW1,DATST>
fprintf('═══════════════════════════════════════════════════════════════\n\n');
fprintf('Planned: %dK × %dC × %d cenários × %d políticas = %d runs\n', ...
        numel(K_values), numel(capacity_values), n_sc, numel(policies), total_runs);
fprintf('K values   : %s\n', num2str(K_values));
fprintf('Capacidades: %s\n', num2str(capacity_values));
fprintf('Políticas  : %s\n', strjoin(policies, ' | '));
fprintf('T_steady   : %d s\n', T_STEADY);

if ~auto_confirm && ~smoke_only
    fprintf('\nPress ENTER to continue, or Ctrl-C to abort.\n');
    pause;
end

cfg_base          = build_scenario_debug();
cfg_base.w_unc    = 1/3;
cfg_base.w_map    = 1/3;
cfg_base.w_vid    = 1/3;
cfg_base.arrivalModel     = 'capacity_drain';
cfg_base.corridorCapacity = 1;
cfg_base.steady_duration  = T_STEADY;
cfg_base.minStartDelay    = 2;
cfg_base.videoModel = 'atomic';
cfg_base.fps        = 30;
cfg_base.L_vid      = 100;
cfg_base.L_c2       = 1;
cfg_base.w_coh      = 5;
cfg_base.L_nom      = 0.01;
cfg_base.p_c2       = 0.80;
cfg_base.p_vid      = 0.80;
cfg_base.rho_0      = 0.01;
cfg_base.csvExport  = true;
cfg_base.csvDir     = 'csv_export/capacity_debug';
cfg_base.headless   = true;

%% ── Smoke test ───────────────────────────────────────────────────────
if ~skip_smoke || smoke_only
    fprintf('\n──────────────────────────────────────────────────────────\n');
    fprintf('  SMOKE TEST (SCALE=0.1, C=2, K=1, T=30s)\n');
    fprintf('──────────────────────────────────────────────────────────\n\n');
    smoke_t0 = tic;
    smoke_ok = run_smoke_debug(OUT_ROOT, cfg_base);
    fprintf('[SMOKE] Completed in %.1f s, status: %s\n', ...
            toc(smoke_t0), ternary(smoke_ok, 'PASS', 'FAIL'));
    if smoke_only, diary off; return; end
    if ~smoke_ok
        fprintf('[SMOKE] FAILED — aborting.\n');
        diary off; return;
    end
    if ~auto_confirm
        fprintf('\nSmoke OK. Press ENTER to launch the full sweep.\n');
        pause;
    end
end

%% ── Parallel pool ────────────────────────────────────────────────────
combos   = build_combos_debug(K_values, capacity_values, scenarios, policies);
n_combos = numel(combos);

nTarget = 8;
cluster = parcluster('local');
if cluster.NumWorkers ~= nTarget
    cluster.NumWorkers = nTarget;
    saveProfile(cluster);
end
nWorkers = min(n_combos, cluster.NumWorkers);

pool = gcp('nocreate');
if isempty(pool)
    parpool('local', nWorkers);
    fprintf('[PARALLEL] Pool started: %d workers.\n', nWorkers);
else
    if pool.NumWorkers < nWorkers
        delete(pool);
        parpool('local', nWorkers);
        fprintf('[PARALLEL] Pool restarted: %d workers.\n', nWorkers);
    else
        fprintf('[PARALLEL] Using existing pool: %d workers.\n', pool.NumWorkers);
    end
end

%% ── Sweep ────────────────────────────────────────────────────────────
fprintf('\n──────────────────────────────────────────────────────────\n');
fprintf('  DEBUG SWEEP  (%d runs, %d workers)\n', n_combos, nWorkers);
fprintf('──────────────────────────────────────────────────────────\n\n');

sw_t0 = tic;
all_rows_cell = cell(n_combos, 1);

parfor idx = 1:n_combos
    c        = combos(idx);
    outDir_r = fullfile(OUT_ROOT, sprintf('K%d', c.K), ...
                        sprintf('C%02d', c.C), c.sc.label);
    system(sprintf('mkdir -p "%s"', outDir_r));

    cfg = build_capacity_run_cfg_debug(cfg_base, c.C, c.K, T_STEADY, c.sc, outDir_r);
    cfg.schedulingPolicy = c.policy;

    fprintf('[K=%2d C=%2d %-12s] %-22s  iniciando...\n', ...
            c.K, c.C, c.sc.label, c.policy);
    t_run = tic;
    try
        results = run_sim(cfg);
        dt = toc(t_run);
        fprintf('[K=%2d C=%2d %-12s] %-22s  %.0f s OK\n', ...
                c.K, c.C, c.sc.label, c.policy, dt);
        all_rows_cell{idx} = build_summary_row_debug(results, c.C, c.K, c.sc, c.policy);
    catch ME
        dt = toc(t_run);
        fprintf('[K=%2d C=%2d %-12s] %-22s  FAILED (%.0f s): %s\n', ...
                c.K, c.C, c.sc.label, c.policy, dt, ME.message);
        all_rows_cell{idx} = struct( ...
            'capacity', c.C, 'K', c.K, ...
            'scenario', c.sc.label, 'routing_mode', c.sc.routing_mode, ...
            'num_routes', numel(c.sc.routes), 'policy', c.policy, ...
            'error', ME.message, 'mean_h1', NaN, 'mean_h2', NaN, ...
            'mean_r_sys', NaN, 'Jhat_emp', NaN);
    end
end

fprintf('\n[DEBUG] Sweep concluído em %.2f h\n', toc(sw_t0)/3600);

%% ── Summary CSVs ─────────────────────────────────────────────────────
for ki = 1:numel(K_values)
    for ci = 1:numel(capacity_values)
        for si = 1:n_sc
            K = K_values(ki); C = capacity_values(ci); sc = scenarios(si);
            mask = cellfun(@(r) isfield(r,'K') && r.K==K && ...
                                isfield(r,'capacity') && r.capacity==C && ...
                                isfield(r,'scenario') && strcmp(r.scenario, sc.label), ...
                           all_rows_cell);
            write_summary_debug(all_rows_cell(mask), ...
                fullfile(OUT_ROOT, sprintf('K%d',K), sprintf('C%02d',C), ...
                         sc.label, 'sweep_capacity_summary.csv'));
        end
    end
end

valid   = ~cellfun(@isempty, all_rows_cell);
all_csv = fullfile(OUT_ROOT, 'sweep_capacity_summary_all.csv');
write_summary_debug(all_rows_cell(valid), all_csv);

fprintf('\n═══════════════════════════════════════════════════════════════\n');
fprintf('  EXPERIMENT_CAPACITY_DEBUG COMPLETED\n');
fprintf('  Finished: %s\n', datestr(now)); %#ok<TNOW1,DATST>
fprintf('  %d runs total.  Summary: %s\n', total_runs, all_csv);
fprintf('═══════════════════════════════════════════════════════════════\n\n');
diary off;
end


%% ═══════════════════════════════════════════════════════════════════
%%  Cenários
%% ═══════════════════════════════════════════════════════════════════
function scenarios = define_capacity_scenarios_debug()
% Route A — curta, próxima à BS (SNR ≈ 35-40 dB)
rA.numDrones = 1;
rA.start     = [200,  4000];   % [North, East]
rA.goal      = [300,  6000];
rA.label     = 'RouteA';

% Route B — curta, longe da BS + hotspots (SNR ≈ 6-8 dB)
rB.numDrones = 1;
rB.start     = [-500, 12000];  % [North, East]
rB.goal      = [-600, 14000];
rB.label     = 'RouteB';

s1.label        = '2r_static';
s1.routing_mode = 'static';
s1.routes       = [rA, rB];

scenarios = [s1];
end


%% ═══════════════════════════════════════════════════════════════════
%%  Build combos
%% ═══════════════════════════════════════════════════════════════════
function combos = build_combos_debug(K_values, capacity_values, scenarios, policies)
n = numel(K_values) * numel(capacity_values) * numel(scenarios) * numel(policies);
combos(n) = struct('K', 0, 'C', 0, 'sc', [], 'policy', '');
idx = 0;
for ki = 1:numel(K_values)
    for ci = 1:numel(capacity_values)
        for si = 1:numel(scenarios)
            for pi = 1:numel(policies)
                idx = idx + 1;
                combos(idx).K      = K_values(ki);
                combos(idx).C      = capacity_values(ci);
                combos(idx).sc     = scenarios(si);
                combos(idx).policy = policies{pi};
            end
        end
    end
end
end


%% ═══════════════════════════════════════════════════════════════════
%%  Build run config
%% ═══════════════════════════════════════════════════════════════════
function cfg = build_capacity_run_cfg_debug(cfg_base, C, K, T_steady, sc, outDir)
cfg    = cfg_base;
nR     = numel(sc.routes);
routes = sc.routes;

C_per = floor(C / nR);
C_rem = C - C_per * nR;

flightTime = norm(routes(1).goal - routes(1).start) / cfg.speedVal;
for ri = 1:nR
    C_r  = C_per + (ri <= C_rem);
    spacing = flightTime / max(C_r, 1);
    routes(ri).numDrones = C_r + floor(T_steady / spacing);
end

cfg.routes           = routes;
cfg.corridorLength   = norm(routes(1).goal - routes(1).start);
cfg.flightTime       = flightTime;
cfg.corridorCapacity = C;
cfg.steady_duration  = T_steady;
cfg.dronesPerSlot    = K;
cfg.csvDir           = outDir;
cfg.routing.mode     = sc.routing_mode;

N_total           = sum([routes.numDrones]);
spacing_min       = flightTime / max(C, 1);
cfg.maxStartDelay = cfg.minStartDelay + (N_total - 1) * spacing_min + 10;

cfg.H_max  = [];
cfg.omega  = [];
cfg.beta_s = [];
cfg.beta_v = [];
end


%% ═══════════════════════════════════════════════════════════════════
%%  Build summary row
%% ═══════════════════════════════════════════════════════════════════
function row = build_summary_row_debug(results, C, K, sc, pol)
cfg = results.cfg;
N   = results.numDrones;

p_emp = zeros(2*N, 1);
for u = 1:N
    tot_c2  = results.slotsC2(u)  + results.slotsC2Fail(u);
    tot_vid = results.slotsVid(u) + results.slotsVidFail(u);
    p_emp(2*(u-1)+1) = results.slotsC2(u)  / max(tot_c2,  1);
    p_emp(2*(u-1)+2) = results.slotsVid(u) / max(tot_vid, 1);
end
LB_legacy = compute_lower_bound(cfg, p_emp);

xs_final = arrayfun(@(u) ifempty(results.xsVal{u}, 0), 1:N);
xv_final = arrayfun(@(u) ifempty(results.xvVal{u}, 0), 1:N);

row.capacity         = C;
row.K                = K;
row.scenario         = sc.label;
row.routing_mode     = sc.routing_mode;
row.num_routes       = numel(sc.routes);
row.num_drones_total = N;
row.policy           = pol;
row.mean_h1          = mean(results.h1MeanV);
row.mean_h2          = mean(results.h2MeanV);
row.mean_r_sys       = results.Rsys_emp;
row.peak_r_sys       = max(results.rSysData);
row.Jhat_emp         = results.Jhat_emp;
row.J_LB_doc         = results.J_LB;
row.rho_emp          = results.rho_emp;
row.LB_legacy        = LB_legacy;
row.qbar_s_mean      = mean(cfg.qbar_s(1:min(N,end)));
row.qbar_v_mean      = mean(cfg.qbar_v(1:min(N,end)));
row.q_emp_s_mean     = mean(results.q_emp_s);
row.q_emp_v_mean     = mean(results.q_emp_v);
row.xs_final_mean    = mean(xs_final);
row.xv_final_mean    = mean(xv_final);
row.n_s              = cfg.n_s;
row.n_v              = cfg.n_v;
row.error            = '';
end


%% ═══════════════════════════════════════════════════════════════════
%%  Write CSV
%% ═══════════════════════════════════════════════════════════════════
function write_summary_debug(rows, filepath)
if isempty(rows), return; end
fid = fopen(filepath, 'w');
if fid == -1
    mkdir(fileparts(filepath));
    fid = fopen(filepath, 'w');
end
fprintf(fid, ['capacity,K,scenario,routing_mode,num_routes,num_drones_total,policy,' ...
              'mean_h1,mean_h2,mean_r_sys,peak_r_sys,' ...
              'Jhat_emp,J_LB_doc,rho_emp,LB_legacy,' ...
              'qbar_s_mean,qbar_v_mean,q_emp_s_mean,q_emp_v_mean,' ...
              'xs_final_mean,xv_final_mean,n_s,n_v\n']);
for ri = 1:numel(rows)
    r = rows{ri};
    if ~isfield(r, 'mean_h1') || isnan(r.mean_h1), continue; end
    fprintf(fid, ['%d,%d,%s,%s,%d,%d,%s,' ...
                  '%.4f,%.4f,%.6e,%.6e,' ...
                  '%.6e,%.6e,%.4f,%.6e,' ...
                  '%.4f,%.4f,%.4f,%.4f,' ...
                  '%.2f,%.2f,%.6e,%.6e\n'], ...
        r.capacity, r.K, r.scenario, r.routing_mode, r.num_routes, ...
        r.num_drones_total, r.policy, ...
        r.mean_h1, r.mean_h2, r.mean_r_sys, r.peak_r_sys, ...
        r.Jhat_emp, r.J_LB_doc, r.rho_emp, r.LB_legacy, ...
        r.qbar_s_mean, r.qbar_v_mean, r.q_emp_s_mean, r.q_emp_v_mean, ...
        r.xs_final_mean, r.xv_final_mean, r.n_s, r.n_v);
end
fclose(fid);
fprintf('[DEBUG] Summary → %s\n', filepath);
end


%% ═══════════════════════════════════════════════════════════════════
%%  Smoke test
%% ═══════════════════════════════════════════════════════════════════
function ok = run_smoke_debug(outRoot, cfg_base)
ok     = true;
n_fail = 0;
SCALE  = 0.1;
C_smoke = 2; K_smoke = 1; T_smoke = 30;
pols = {'round-robin-sw', 'pf-classic-sw', 'max-weight-sw', 'aoi-sw'};
scs  = define_capacity_scenarios_debug();

cfg_s = cfg_base;
cfg_s.routes(1).start   = cfg_base.routes(1).start * SCALE;
cfg_s.routes(1).goal    = cfg_base.routes(1).goal  * SCALE;
cfg_s.corridorLength    = norm(cfg_s.routes(1).goal - cfg_s.routes(1).start);
cfg_s.flightTime        = cfg_s.corridorLength / cfg_base.speedVal;
cfg_s.microBSPos(:,1:2) = cfg_base.microBSPos(:,1:2) * SCALE;
cfg_s.manualHotspots    = [];
cfg_s.numHotspots       = 0;
cfg_s.csvExport         = false;

for si = 1:numel(scs)
    sc = scs(si);
    sc_smoke = sc;
    r = sc.routes(1);
    r.start = r.start * SCALE;
    r.goal  = r.goal  * SCALE;
    sc_smoke.routes = r;

    for pi = 1:numel(pols)
        pol   = pols{pi};
        cfg_r = build_capacity_run_cfg_debug(cfg_s, C_smoke, K_smoke, T_smoke, ...
                                             sc_smoke, fullfile(outRoot, 'smoke'));
        cfg_r.schedulingPolicy = pol;
        try
            results = run_sim(cfg_r);
            h1_mean = mean(results.h1MeanV(~isnan(results.h1MeanV)));
            if isnan(h1_mean) || h1_mean <= 0
                fprintf('[SMOKE] %-12s %-22s  FAIL: h1=%.3f\n', sc.label, pol, h1_mean);
                n_fail = n_fail + 1;
            else
                fprintf('[SMOKE] %-12s %-22s  h1=%.1f  h2=%.1f  OK\n', ...
                        sc.label, pol, h1_mean, mean(results.h2MeanV(~isnan(results.h2MeanV))));
            end
        catch ME
            fprintf('[SMOKE] %-12s %-22s  EXCEPTION: %s\n', sc.label, pol, ME.message);
            n_fail = n_fail + 1;
        end
    end
end
ok = (n_fail == 0);
fprintf('[SMOKE] %s\n', ternary(ok, 'Todos os checks passaram.', ...
        sprintf('%d check(s) falharam.', n_fail)));
end
