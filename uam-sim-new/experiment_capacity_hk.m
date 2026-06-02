function experiment_capacity_hk(varargin)
%% EXPERIMENT_CAPACITY_HK  Complementary sweep — high-K slots {16, 32}.
%
%  Parâmetros idênticos ao experiment_capacity, exceto:
%    K        — dronesPerSlot ∈ [16 32]
%    Output   — csv_export/capacity_20min_hk/
%
%  Modelo capacity_drain:
%    Fase fill:   drones entram com spacing = flightTime/C_route até C simultâneos.
%    Fase steady: C drones ativos por steady_duration = 1200 s  (20 min).
%    Fase drain:  sem novos drones; os activos terminam o voo.
%
%  Sweep axes:
%    K        — dronesPerSlot ∈ [16 32]
%    C        — corridorCapacity ∈ [10 20 30 40 50 60 70 80 90 100]
%    Routing  — static | risk_astar  ×  1 route | 2 routes  (4 cenários)
%    Policy   — round-robin | aoi-pure | aoi-normalized | risk-aware | max-weight-drift | pf-classic
%
%  Total: 2K × 10C × 4 routing × 6 policies = 480 runs.
%  Corredor: 2 km, dois lanes paralelos (North ±50 m), speed = 5 m/s.
%
%  Output: csv_export/capacity_20min_hk/K{K}/C{CCC}/{routing_label}/
%
%  Usage:
%    experiment_capacity_hk()
%    experiment_capacity_hk('--smoke-only')
%    experiment_capacity_hk('skip-smoke')
%    experiment_capacity_hk('--yes')
%    experiment_capacity_hk('--headless')
%    experiment_capacity_hk('--C', '10,20,30')   % subset de capacidades

setup_paths();

% ── Argument parsing ──────────────────────────────────────────────────
skip_smoke   = false;
auto_confirm = false;
headless     = false;
smoke_only   = false;
cap_override = [];

ai = 1;
while ai <= numel(varargin)
    switch varargin{ai}
        case 'skip-smoke',   skip_smoke   = true;
        case '--smoke-only', smoke_only   = true;
        case '--yes',        auto_confirm = true;
        case '--headless',   headless     = true;
        case '--C'
            ai = ai + 1;
            if ai <= numel(varargin)
                cap_override = str2double(strsplit(varargin{ai}, ','));
            end
    end
    ai = ai + 1;
end

% ── Sweep axes ────────────────────────────────────────────────────────
if isempty(cap_override)
    capacity_values = 10:10:100;
else
    capacity_values = cap_override;
end
K_values  = [16, 32];
policies  = {'round-robin', 'aoi-pure', 'aoi-normalized', 'risk-aware', ...
             'max-weight-drift', 'pf-classic'};
T_STEADY  = 1200;  % segundos em regime estável (20 min)

% ── Output root ───────────────────────────────────────────────────────
SIM_ROOT = fileparts(mfilename('fullpath'));
OUT_ROOT = fullfile(SIM_ROOT, 'csv_export', 'capacity_20min_hk');
system(sprintf('mkdir -p "%s"', OUT_ROOT));

% ── Logfile ───────────────────────────────────────────────────────────
ts      = datestr(now, 'yyyymmdd_HHMMSS'); %#ok<TNOW1,DATST>
logFile = fullfile(OUT_ROOT, sprintf('experiment_capacity_hk_%s.log', ts));
diary(logFile); diary on;

fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  EXPERIMENT_CAPACITY_HK — high-K sweep {16, 32}\n');
fprintf('  Corredor: 2 km East, 2 lanes paralelos (N±50 m)  speed=5 m/s\n');
fprintf('  Modelo: capacity_drain  steady=%ds  (~%d s por run)\n', ...
        T_STEADY, T_STEADY + 2*400);
fprintf('  Started: %s\n', datestr(now)); %#ok<TNOW1,DATST>
fprintf('  Log: %s\n', logFile);
fprintf('═══════════════════════════════════════════════════════════════\n\n');

scenarios  = define_capacity_scenarios();
n_sc       = numel(scenarios);
total_runs = numel(K_values) * numel(capacity_values) * n_sc * numel(policies);
fprintf('Planned: %dK × %dC × %d routing × %d policies = %d runs\n', ...
        numel(K_values), numel(capacity_values), n_sc, numel(policies), total_runs);
fprintf('K values: %s\n', num2str(K_values));
fprintf('Capacidades: %s\n', num2str(capacity_values));
fprintf('Cenários: %s\n', strjoin({scenarios.label}, ' | '));

if ~auto_confirm && ~smoke_only
    fprintf('\nPress ENTER to continue, or Ctrl-C to abort.\n');
    pause;
end

% ── Base config ───────────────────────────────────────────────────────
cfg_base          = base_config_capacity();
cfg_base.headless = true;

% ── Smoke test ────────────────────────────────────────────────────────
if ~skip_smoke || smoke_only
    fprintf('\n──────────────────────────────────────────────────────────\n');
    fprintf('  SMOKE TEST (corredor 200m, C=3, T_steady=40s, K=16, 2 políticas × 2 cenários)\n');
    fprintf('──────────────────────────────────────────────────────────\n\n');
    smoke_t0 = tic;
    smoke_ok = run_smoke_capacity(OUT_ROOT, cfg_base);
    fprintf('[SMOKE] Completed in %.1f s, status: %s\n', ...
            toc(smoke_t0), ternary(smoke_ok, 'PASS', 'FAIL'));
    if smoke_only
        diary off; return;
    end
    if ~smoke_ok
        fprintf('[SMOKE] FAILED — aborting.\n');
        diary off; return;
    end
    if ~auto_confirm
        fprintf('\nSmoke OK. Press ENTER to launch the full sweep, or Ctrl-C to abort.\n');
        pause;
    end
end

% ── Parallel pool ─────────────────────────────────────────────────────
combos   = build_combos(K_values, capacity_values, scenarios, policies);
n_combos = numel(combos);

nPhysCores = feature('numcores');
nTarget    = max(1, floor(nPhysCores / 4));
cluster    = parcluster('local');
if cluster.NumWorkers ~= nTarget
    cluster.NumWorkers = nTarget;
    saveProfile(cluster);
end
nWorkers = min(n_combos, cluster.NumWorkers);

pool = gcp('nocreate');
if isempty(pool)
    parpool('local', nWorkers);
    fprintf('[PARALLEL] Pool started: %d workers (cores físicos=%d).\n', nWorkers, nPhysCores);
else
    if pool.NumWorkers < nWorkers
        delete(pool);
        parpool('local', nWorkers);
        fprintf('[PARALLEL] Pool restarted: %d workers.\n', nWorkers);
    else
        fprintf('[PARALLEL] Using existing pool: %d workers.\n', pool.NumWorkers);
    end
end

% ── Sweep ─────────────────────────────────────────────────────────────
fprintf('\n──────────────────────────────────────────────────────────\n');
fprintf('  CAPACITY_HK SWEEP  (%d runs, %d workers)\n', n_combos, nWorkers);
fprintf('──────────────────────────────────────────────────────────\n\n');

sw_t0 = tic;
all_rows_cell = cell(n_combos, 1);

parfor idx = 1:n_combos
    c        = combos(idx);
    outDir_r = fullfile(OUT_ROOT, sprintf('K%d', c.K), ...
                        sprintf('C%03d', c.C), c.sc.label);
    system(sprintf('mkdir -p "%s"', outDir_r));

    cfg = build_capacity_run_cfg(cfg_base, c.C, c.K, T_STEADY, c.sc, outDir_r);
    cfg.schedulingPolicy = c.policy;

    fprintf('[K=%d C=%3d %-12s] %-22s  iniciando...\n', ...
            c.K, c.C, c.sc.label, c.policy);
    t_run = tic;
    try
        results = run_sim(cfg);
        dt = toc(t_run);
        fprintf('[K=%d C=%3d %-12s] %-22s  %.0f s OK\n', ...
                c.K, c.C, c.sc.label, c.policy, dt);
        all_rows_cell{idx} = build_summary_row(results, c.C, c.K, c.sc, c.policy);
    catch ME
        dt = toc(t_run);
        fprintf('[K=%d C=%3d %-12s] %-22s  FAILED (%.0f s): %s\n', ...
                c.K, c.C, c.sc.label, c.policy, dt, ME.message);
        all_rows_cell{idx} = struct( ...
            'capacity', c.C, 'K', c.K, ...
            'scenario', c.sc.label, 'routing_mode', c.sc.routing_mode, ...
            'num_routes', numel(c.sc.routes), 'policy', c.policy, ...
            'error', ME.message, 'mean_h1', NaN, 'mean_h2', NaN, ...
            'mean_r_sys', NaN, 'Jhat_emp', NaN);
    end
end

fprintf('\n[CAPACITY_HK] Sweep concluído em %.2f h\n', toc(sw_t0)/3600);

% ── Summary CSVs por (K, C, scenario) ────────────────────────────────
for ki = 1:numel(K_values)
    for ci = 1:numel(capacity_values)
        for si = 1:n_sc
            K = K_values(ki); C = capacity_values(ci); sc = scenarios(si);
            mask = cellfun(@(r) isfield(r,'K') && r.K==K && ...
                                isfield(r,'capacity') && r.capacity==C && ...
                                isfield(r,'scenario') && strcmp(r.scenario, sc.label), ...
                           all_rows_cell);
            write_capacity_summary(all_rows_cell(mask), ...
                fullfile(OUT_ROOT, sprintf('K%d',K), sprintf('C%03d',C), ...
                         sc.label, 'sweep_capacity_summary.csv'));
        end
    end
end

% ── Summary global ────────────────────────────────────────────────────
valid   = ~cellfun(@isempty, all_rows_cell);
all_csv = fullfile(OUT_ROOT, 'sweep_capacity_summary_all.csv');
write_capacity_summary(all_rows_cell(valid), all_csv);
fprintf('[CAPACITY_HK] Aggregated → %s\n', all_csv);

fprintf('\n═══════════════════════════════════════════════════════════════\n');
fprintf('  EXPERIMENT_CAPACITY_HK COMPLETED\n');
fprintf('  Finished: %s\n', datestr(now)); %#ok<TNOW1,DATST>
fprintf('  %d runs total.\n', total_runs);
fprintf('  Summary: %s\n', all_csv);
fprintf('═══════════════════════════════════════════════════════════════\n\n');
diary off;
end


%% ═══════════════════════════════════════════════════════════════════
%%  Routing scenarios — 4 combinações estáticas/A* × 1/2 rotas
%% ═══════════════════════════════════════════════════════════════════
function scenarios = define_capacity_scenarios()
r1.numDrones = 1;
r1.start     = [ 50, -1000];
r1.goal      = [ 50,  1000];
r1.label     = 'lane-N';

r2.numDrones = 1;
r2.start     = [-50, -1000];
r2.goal      = [-50,  1000];
r2.label     = 'lane-S';

s1.label        = '1r_static';
s1.routing_mode = 'static';
s1.routes       = r1;

s2.label        = '1r_astar';
s2.routing_mode = 'risk_astar';
s2.routes       = r1;

s3.label        = '2r_static';
s3.routing_mode = 'static';
s3.routes       = [r1, r2];

s4.label        = '2r_astar';
s4.routing_mode = 'risk_astar';
s4.routes       = [r1, r2];

scenarios = [s1, s2, s3, s4];
end


%% ═══════════════════════════════════════════════════════════════════
%%  Build flat combo list for parfor
%% ═══════════════════════════════════════════════════════════════════
function combos = build_combos(K_values, capacity_values, scenarios, policies)
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
%%  Build run config para capacidade C, K slots e cenário de routing
%% ═══════════════════════════════════════════════════════════════════
function cfg = build_capacity_run_cfg(cfg_base, C, K, T_steady, sc, outDir)
cfg = cfg_base;

nR     = numel(sc.routes);
routes = sc.routes;

C_per  = floor(C / nR);
C_rem  = C - C_per * nR;

flightTime = norm(routes(1).goal - routes(1).start) / cfg.speedVal;
for ri = 1:nR
    C_r              = C_per + (ri <= C_rem);
    spacing          = flightTime / max(C_r, 1);
    N_r              = C_r + floor(T_steady / spacing);
    routes(ri).numDrones = N_r;
end

cfg.routes           = routes;
cfg.corridorLength   = norm(routes(1).goal - routes(1).start);
cfg.flightTime       = flightTime;
cfg.corridorCapacity = C;
cfg.steady_duration  = T_steady;
cfg.dronesPerSlot    = K;
cfg.csvDir           = outDir;
cfg.routing.mode     = sc.routing_mode;

N_total          = sum([routes.numDrones]);
spacing_min      = flightTime / max(C, 1);
cfg.maxStartDelay = cfg.minStartDelay + (N_total - 1) * spacing_min + 10;

cfg.H_max  = [];
cfg.omega  = [];
cfg.beta_s = [];
cfg.beta_v = [];
end


%% ═══════════════════════════════════════════════════════════════════
%%  Build summary row
%% ═══════════════════════════════════════════════════════════════════
function row = build_summary_row(results, C, K, sc, pol)
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
%%  Write summary CSV
%% ═══════════════════════════════════════════════════════════════════
function write_capacity_summary(rows, filepath)
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
fprintf('[CAPACITY_HK] Summary → %s\n', filepath);
end


%% ═══════════════════════════════════════════════════════════════════
%%  Smoke test — 200 m, C=3, K=16, T=40 s, 2 políticas × 2 cenários
%% ═══════════════════════════════════════════════════════════════════
function ok = run_smoke_capacity(outRoot, cfg_base)
ok     = true;
n_fail = 0;
SCALE   = 0.1;
C_smoke = 3;
T_smoke = 40;
K_smoke = 16;
pols    = {'round-robin', 'max-weight-drift'};
scs     = define_capacity_scenarios();
scs     = scs(1:2);

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
    sc       = scs(si);
    sc_smoke = sc;
    r        = sc.routes(1);
    r.start  = r.start * SCALE;
    r.goal   = r.goal  * SCALE;
    sc_smoke.routes = r;

    for pi = 1:numel(pols)
        pol   = pols{pi};
        cfg_r = build_capacity_run_cfg(cfg_s, C_smoke, K_smoke, T_smoke, ...
                                       sc_smoke, fullfile(outRoot, 'smoke'));
        cfg_r.schedulingPolicy = pol;

        flightTime_s = norm(sc_smoke.routes(1).goal - sc_smoke.routes(1).start) / cfg_s.speedVal;
        spacing_exp  = flightTime_s / C_smoke;
        N_exp        = C_smoke + floor(T_smoke / spacing_exp);

        try
            results = run_sim(cfg_r);
            h1_mean = mean(results.h1MeanV(~isnan(results.h1MeanV)));
            h2_mean = mean(results.h2MeanV(~isnan(results.h2MeanV)));
            fail = {};
            if results.numDrones ~= N_exp
                fail{end+1} = sprintf('N=%d (exp %d)', results.numDrones, N_exp); %#ok<AGROW>
            end
            if isnan(h1_mean) || h1_mean <= 0
                fail{end+1} = sprintf('h1=%.3f', h1_mean); %#ok<AGROW>
            end
            if isempty(fail)
                fprintf('[SMOKE] %-12s %-22s  N=%d  h1=%.1f  h2=%.1f  OK\n', ...
                        sc.label, pol, results.numDrones, h1_mean, h2_mean);
            else
                for fi = 1:numel(fail)
                    fprintf('[SMOKE] %-12s %-22s  FAIL: %s\n', sc.label, pol, fail{fi});
                end
                n_fail = n_fail + numel(fail);
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


%% ═══════════════════════════════════════════════════════════════════
%%  Helpers
%% ═══════════════════════════════════════════════════════════════════
function r = ternary(cond, a, b)
if cond, r = a; else, r = b; end
end

function v = ifempty(x, default)
if isempty(x), v = default; else, v = x(end); end
end
