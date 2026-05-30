function experiment_capacity(varargin)
%% EXPERIMENT_CAPACITY  Capacity-based sweep — parfor over K, serial over C × policy.
%
%  Modelo capacity_drain:
%    Fase fill:   drones entram com spacing = flightTime/C até atingir C simultâneos.
%    Fase steady: C drones ativos por steady_duration = 1200 s  (20 min).
%    Fase drain:  sem novos drones; os activos terminam o voo.
%    N_total = C + floor(1200/spacing)  [20 a 320 drones por run].
%
%  Sweep axes:
%    K  — dronesPerSlot ∈ [1 2 4 8]              (paralelizado via parfor)
%    C  — corridorCapacity ∈ [5 10 20 40 60 80]
%    Policy — round-robin | aoi-pure | risk-aware | max-weight-drift | pf-classic
%
%  Total: 4K × 6C × 5 policies = 120 runs.
%  Corredor: 2 km East (−1000 a +1000 m), speed = 5 m/s.
%
%  Usage:
%    experiment_capacity()
%    experiment_capacity('--smoke-only')         % só smoke test
%    experiment_capacity('skip-smoke')           % pula smoke, roda sweep
%    experiment_capacity('--yes')               % sem confirmação
%    experiment_capacity('--headless')          % sem dashboard
%    experiment_capacity('--C', '5,10,20')      % subset de capacidades

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
    capacity_values = [5, 10, 20, 40, 60, 80];
else
    capacity_values = cap_override;
end
K_values  = [1, 2, 4, 8];
policies  = {'round-robin', 'aoi-pure', 'risk-aware', 'max-weight-drift', 'pf-classic'};
T_STEADY  = 1200;  % segundos em regime estável (20 min)

% ── Output root ───────────────────────────────────────────────────────
SIM_ROOT = fileparts(mfilename('fullpath'));
OUT_ROOT = fullfile(SIM_ROOT, 'csv_export', 'capacity_20min');
system(sprintf('mkdir -p "%s"', OUT_ROOT));

% ── Logfile ───────────────────────────────────────────────────────────
ts      = datestr(now, 'yyyymmdd_HHMMSS'); %#ok<TNOW1,DATST>
logFile = fullfile(OUT_ROOT, sprintf('experiment_capacity_%s.log', ts));
diary(logFile); diary on;

fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  EXPERIMENT_CAPACITY — capacity × K × policy sweep\n');
fprintf('  Corridor: 2 km East (−1000 to +1000 m)  speed=5 m/s\n');
fprintf('  Modelo: capacity_drain  steady=%ds  sim_duration ≈ %d s por run\n', ...
        T_STEADY, T_STEADY + 2*400);
fprintf('  Started: %s\n', datestr(now)); %#ok<TNOW1,DATST>
fprintf('  Log: %s\n', logFile);
fprintf('═══════════════════════════════════════════════════════════════\n\n');

total_runs = numel(capacity_values) * numel(K_values) * numel(policies);
fprintf('Planned: %dC × %dK × %d policies = %d runs\n', ...
        numel(capacity_values), numel(K_values), numel(policies), total_runs);
fprintf('Modelo: capacity_drain  steady=%ds  corredor=2km  speed=5m/s\n', T_STEADY);

if ~auto_confirm && ~smoke_only
    fprintf('\nPress ENTER to continue, or Ctrl-C to abort.\n');
    pause;
end

% ── Base config ───────────────────────────────────────────────────────
cfg_base          = base_config_capacity();
cfg_base.headless = true;   % sempre headless no parfor

% ── Smoke test ────────────────────────────────────────────────────────
if ~skip_smoke || smoke_only
    fprintf('\n──────────────────────────────────────────────────────────\n');
    fprintf('  SMOKE TEST (corredor 200m, C=3, T_steady=40s, 2 políticas)\n');
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
end

% ── Parallel pool — um worker por run (K × C × policy) ───────────────
combos   = build_combos(K_values, capacity_values, policies);
n_combos = numel(combos);

% Usa todos os cores físicos disponíveis, respeitando o número de runs.
% feature('numcores') retorna cores físicos (ignora hyperthreading).
nPhysCores = feature('numcores');
nTarget    = max(1, floor(nPhysCores / 4));   % um quarto dos cores físicos
cluster    = parcluster('local');
if cluster.NumWorkers ~= nTarget
    cluster.NumWorkers = nTarget;
    saveProfile(cluster);
    fprintf('[PARALLEL] Perfil local atualizado: NumWorkers=%d (metade de %d cores físicos).\n', ...
            nTarget, nPhysCores);
end
nWorkers = min(n_combos, cluster.NumWorkers);

pool = gcp('nocreate');
if isempty(pool)
    parpool('local', nWorkers);
    fprintf('[PARALLEL] Pool started: %d workers (cores físicos=%d).\n', ...
            nWorkers, nPhysCores);
else
    if pool.NumWorkers < nWorkers
        delete(pool);
        parpool('local', nWorkers);
        fprintf('[PARALLEL] Pool restarted: %d workers.\n', nWorkers);
    else
        fprintf('[PARALLEL] Using existing pool: %d workers.\n', pool.NumWorkers);
    end
end

% ── Sweep (parfor sobre todos os runs) ───────────────────────────────
fprintf('\n──────────────────────────────────────────────────────────\n');
fprintf('  CAPACITY SWEEP  (%d runs, %d workers)\n', n_combos, nWorkers);
fprintf('──────────────────────────────────────────────────────────\n\n');

sw_t0 = tic;

all_rows_cell = cell(n_combos, 1);

parfor idx = 1:n_combos
    c        = combos(idx);
    outDir_r = fullfile(OUT_ROOT, sprintf('K%d', c.K), sprintf('C%03d', c.C));
    system(sprintf('mkdir -p "%s"', outDir_r));   % atômico, seguro em paralelo

    cfg = build_capacity_run_cfg(cfg_base, c.C, c.K, T_STEADY, outDir_r);
    cfg.schedulingPolicy = c.policy;

    fprintf('[K=%d C=%2d] %-22s  iniciando...\n', c.K, c.C, c.policy);
    t_run = tic;
    try
        results = run_sim(cfg);
        dt = toc(t_run);
        fprintf('[K=%d C=%2d] %-22s  %.0f s OK\n', c.K, c.C, c.policy, dt);
        all_rows_cell{idx} = build_summary_row(results, c.C, c.K, c.policy);
    catch ME
        dt = toc(t_run);
        fprintf('[K=%d C=%2d] %-22s  FAILED (%.0f s): %s\n', ...
                c.K, c.C, c.policy, dt, ME.message);
        all_rows_cell{idx} = struct( ...
            'capacity', c.C, 'K', c.K, 'policy', c.policy, ...
            'error',    ME.message, 'mean_h1', NaN, 'mean_h2', NaN, ...
            'mean_r_sys', NaN, 'Jhat_emp', NaN);
    end
end

fprintf('\n[CAPACITY] Sweep concluído em %.2f h\n', toc(sw_t0)/3600);

% ── Per-(K,C) summary CSVs ────────────────────────────────────────────
for ki = 1:numel(K_values)
    for ci = 1:numel(capacity_values)
        K = K_values(ki); C = capacity_values(ci);
        mask = cellfun(@(r) isfield(r,'K') && r.K==K && isfield(r,'capacity') && r.capacity==C, ...
                       all_rows_cell);
        write_capacity_summary(all_rows_cell(mask), ...
            fullfile(OUT_ROOT, sprintf('K%d',K), sprintf('C%03d',C), ...
                     'sweep_capacity_summary.csv'));
    end
end

% ── Consolidar summary global ─────────────────────────────────────────
valid = ~cellfun(@isempty, all_rows_cell);
all_rows = all_rows_cell(valid);
all_csv  = fullfile(OUT_ROOT, 'sweep_capacity_summary_all.csv');
write_capacity_summary(all_rows, all_csv);
fprintf('[CAPACITY] Aggregated → %s\n', all_csv);

% ── Final ─────────────────────────────────────────────────────────────
fprintf('\n═══════════════════════════════════════════════════════════════\n');
fprintf('  EXPERIMENT_CAPACITY COMPLETED\n');
fprintf('  Finished: %s\n', datestr(now)); %#ok<TNOW1,DATST>
fprintf('  %d runs total.\n', total_runs);
fprintf('  Summary: %s\n', all_csv);
fprintf('═══════════════════════════════════════════════════════════════\n\n');
diary off;
end


%% ═══════════════════════════════════════════════════════════════════
%%  Build flat list of all (K, C, policy) combinations for parfor
%% ═══════════════════════════════════════════════════════════════════
function combos = build_combos(K_values, capacity_values, policies)
n = numel(K_values) * numel(capacity_values) * numel(policies);
combos(n) = struct('K', 0, 'C', 0, 'policy', '');
idx = 0;
for ki = 1:numel(K_values)
    for ci = 1:numel(capacity_values)
        for pi = 1:numel(policies)
            idx = idx + 1;
            combos(idx).K      = K_values(ki);
            combos(idx).C      = capacity_values(ci);
            combos(idx).policy = policies{pi};
        end
    end
end
end


%% ═══════════════════════════════════════════════════════════════════
%%  Build a run-ready config for capacity C, dronesPerSlot K
%% ═══════════════════════════════════════════════════════════════════
function cfg = build_capacity_run_cfg(cfg_base, C, K, T_steady, outDir)
cfg = cfg_base;

spacing = cfg.flightTime / max(C, 1);
N_total = C + floor(T_steady / spacing);

cfg.routes(1).numDrones = N_total;
cfg.corridorCapacity    = C;
cfg.steady_duration     = T_steady;
cfg.dronesPerSlot       = K;
cfg.csvDir              = outDir;
cfg.maxStartDelay       = cfg.minStartDelay + (N_total - 1) * spacing + 10;

cfg.H_max  = [];
cfg.omega  = [];
cfg.beta_s = [];
cfg.beta_v = [];
end


%% ═══════════════════════════════════════════════════════════════════
%%  Build one summary row — usa results.cfg (validado dentro do run_sim)
%% ═══════════════════════════════════════════════════════════════════
function row = build_summary_row(results, C, K, pol)
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
row.qbar_s_mean      = mean(cfg.qbar_s(1:C));
row.qbar_v_mean      = mean(cfg.qbar_v(1:C));
row.q_emp_s_mean     = mean(results.q_emp_s);
row.q_emp_v_mean     = mean(results.q_emp_v);
row.xs_final_mean    = mean(xs_final);
row.xv_final_mean    = mean(xv_final);
row.n_s              = cfg.n_s;
row.n_v              = cfg.n_v;
row.error            = '';
end


%% ═══════════════════════════════════════════════════════════════════
%%  Write a cell-array of row structs to a CSV file
%% ═══════════════════════════════════════════════════════════════════
function write_capacity_summary(rows, filepath)
if isempty(rows), return; end
fid = fopen(filepath, 'w');
if fid == -1
    mkdir(fileparts(filepath));
    fid = fopen(filepath, 'w');
end
fprintf(fid, ['capacity,K,num_drones_total,policy,' ...
              'mean_h1,mean_h2,mean_r_sys,peak_r_sys,' ...
              'Jhat_emp,J_LB_doc,rho_emp,LB_legacy,' ...
              'qbar_s_mean,qbar_v_mean,q_emp_s_mean,q_emp_v_mean,' ...
              'xs_final_mean,xv_final_mean,n_s,n_v\n']);
for ri = 1:numel(rows)
    r = rows{ri};
    if ~isfield(r, 'mean_h1') || isnan(r.mean_h1), continue; end
    fprintf(fid, '%d,%d,%d,%s,%.4f,%.4f,%.6e,%.6e,%.6e,%.6e,%.4f,%.6e,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.6e,%.6e\n', ...
        r.capacity, r.K, r.num_drones_total, r.policy, ...
        r.mean_h1, r.mean_h2, r.mean_r_sys, r.peak_r_sys, ...
        r.Jhat_emp, r.J_LB_doc, r.rho_emp, r.LB_legacy, ...
        r.qbar_s_mean, r.qbar_v_mean, r.q_emp_s_mean, r.q_emp_v_mean, ...
        r.xs_final_mean, r.xv_final_mean, r.n_s, r.n_v);
end
fclose(fid);
fprintf('[CAPACITY] Summary → %s\n', filepath);
end


%% ═══════════════════════════════════════════════════════════════════
%%  Smoke test — corredor 200m, C=3, T_steady=40s, 2 políticas
%% ═══════════════════════════════════════════════════════════════════
function ok = run_smoke_capacity(outRoot, cfg_base)
ok     = true;
n_fail = 0;
SCALE    = 0.1;
C_smoke  = 3;
T_smoke  = 40;
K_smoke  = 1;
pols     = {'round-robin', 'max-weight-drift'};

cfg_s = cfg_base;
cfg_s.routes(1).start    = cfg_base.routes(1).start * SCALE;
cfg_s.routes(1).goal     = cfg_base.routes(1).goal  * SCALE;
cfg_s.corridorLength     = norm(cfg_s.routes(1).goal - cfg_s.routes(1).start);
cfg_s.flightTime         = cfg_s.corridorLength / cfg_base.speedVal;
cfg_s.microBSPos(:,1:2)  = cfg_base.microBSPos(:,1:2) * SCALE;
cfg_s.manualHotspots     = [];
cfg_s.numHotspots        = 0;
cfg_s.csvExport          = false;

spacing_exp = cfg_s.flightTime / C_smoke;
N_exp       = C_smoke + floor(T_smoke / spacing_exp);
t_full_exp  = cfg_s.minStartDelay + (C_smoke - 1) * spacing_exp;
t_mid_exp   = t_full_exp + T_smoke / 2;

fprintf('[SMOKE] Corredor=%.0fm  C=%d  T_steady=%ds  %d políticas\n', ...
        cfg_s.corridorLength, C_smoke, T_smoke, numel(pols));
fprintf('[SMOKE] spacing=%.1fs  N_total_exp=%d  t_full=%.1fs  t_mid=%.1fs\n', ...
        spacing_exp, N_exp, t_full_exp, t_mid_exp);

for pi = 1:numel(pols)
    pol   = pols{pi};
    cfg_r = build_capacity_run_cfg(cfg_s, C_smoke, K_smoke, T_smoke, ...
                                   fullfile(outRoot, 'smoke'));
    cfg_r.schedulingPolicy = pol;
    fails_this = {};

    try
        results = run_sim(cfg_r);

        if results.numDrones ~= N_exp
            fails_this{end+1} = sprintf('N_total=%d (esperado %d)', ...
                                        results.numDrones, N_exp); %#ok<AGROW>
        end

        n_active = sum(results.startTimes <= t_mid_exp & ...
                       results.endTimes   >= t_mid_exp);
        if n_active ~= C_smoke
            fails_this{end+1} = sprintf('capacidade: %d activos em t=%.1fs (esperado %d)', ...
                                        n_active, t_mid_exp, C_smoke); %#ok<AGROW>
        end

        h1_mean = mean(results.h1MeanV(~isnan(results.h1MeanV)));
        h2_mean = mean(results.h2MeanV(~isnan(results.h2MeanV)));
        if isnan(h1_mean) || h1_mean <= 0
            fails_this{end+1} = sprintf('h1=%.3f inválido', h1_mean); %#ok<AGROW>
        end
        if isnan(h2_mean) || h2_mean <= 0
            fails_this{end+1} = sprintf('h2=%.3f inválido', h2_mean); %#ok<AGROW>
        end

        if isempty(fails_this)
            fprintf('[SMOKE] %-22s  N=%d  active@mid=%d/%d  h1=%.1f  h2=%.1f  OK\n', ...
                    pol, results.numDrones, n_active, C_smoke, h1_mean, h2_mean);
        else
            for fi = 1:numel(fails_this)
                fprintf('[SMOKE] %-22s  FAIL: %s\n', pol, fails_this{fi});
            end
            n_fail = n_fail + numel(fails_this);
        end

    catch ME
        fprintf('[SMOKE] %-22s  EXCEPTION: %s\n', pol, ME.message);
        if ~isempty(ME.stack)
            fprintf('[SMOKE]   at %s:%d\n', ME.stack(1).name, ME.stack(1).line);
        end
        n_fail = n_fail + 1;
    end
end

ok = (n_fail == 0);
if ok
    fprintf('[SMOKE] Todos os checks passaram.\n');
else
    fprintf('[SMOKE] %d check(s) falharam.\n', n_fail);
end
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
