function experiment_routing_all_policies(varargin)
%% EXPERIMENT_ROUTING_ALL_POLICIES  Full sweep — all six policies × routing × load × K.
%
%  Extends experiment_routing with pf-classic and max-weight-drift added to
%  the policy set.  Structure and sweep axes are identical to experiment_routing
%  so results are directly comparable.
%
%  Sweeps four dimensions:
%    K        — dronesPerSlot ∈ [1 2 4 8]
%    Load     — N ∈ [2 4 8 16 32 64] (doubling)
%    Routing  — static | Risk-A*
%    Routes   — 1 route | 2 routes   (→ 4 routing scenarios)
%    Policy   — round-robin, round-robin-aware, aoi-pure,
%               max-weight-drift, pf-classic, risk-aware
%
%  Total: 4K × 6N × 4 scenarios × 6 policies = 576 runs.
%
%  Output per (K,N): csv_export/routing_all/sweep/K{K}/N{N}/sweep_routing_summary.csv
%  Aggregated:       csv_export/routing_all/sweep/sweep_routing_summary_all.csv
%
%  Usage:
%    experiment_routing_all_policies()               % interactive, dashboard visible
%    experiment_routing_all_policies('skip-smoke')   % skip smoke test
%    experiment_routing_all_policies('--yes')        % skip confirmation prompt
%    experiment_routing_all_policies('--headless')   % batch / server

setup_paths();

% --- Argument parsing ---
skip_smoke   = false;
auto_confirm = false;
headless     = false;
ai = 1;
while ai <= numel(varargin)
    switch varargin{ai}
        case 'skip-smoke', skip_smoke   = true;
        case '--yes',      auto_confirm = true;
        case '--headless', headless     = true;
    end
    ai = ai + 1;
end

% --- Sweep axes (identical to experiment_routing) ---
K_values     = [1, 2, 4, 8];
drone_counts = [2, 4, 8, 16, 32, 64];

policies = { ...
    'round-robin', ...
    'round-robin-aware', ...
    'aoi-pure', ...
    'max-weight-drift', ...
    'pf-classic', ...
    'risk-aware' };

% --- Output root (absolute, anchored to this script's directory) ---
SIM_ROOT = fileparts(mfilename('fullpath'));
OUT_ROOT = fullfile(SIM_ROOT, 'csv_export', 'routing_all');
system(sprintf('mkdir -p "%s"', OUT_ROOT));

% --- Log file ---
ts      = datestr(now, 'yyyymmdd_HHMMSS'); %#ok<TNOW1,DATST>
logFile = fullfile(OUT_ROOT, sprintf('experiment_routing_all_%s.log', ts));
diary(logFile); diary on;

fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  EXPERIMENT_ROUTING_ALL_POLICIES — K × load × routing × policy sweep\n');
fprintf('  Policies (%d): %s\n', numel(policies), strjoin(policies, ', '));
fprintf('  Started: %s\n', datestr(now)); %#ok<TNOW1,DATST>
fprintf('  Log: %s\n', logFile);
fprintf('═══════════════════════════════════════════════════════════════\n\n');

% --- Confirmation ---
if ~auto_confirm
    n_runs = numel(K_values) * numel(drone_counts) * 4 * numel(policies);
    fprintf('This will run:\n');
    fprintf('  1. Smoke test       — ~5 min  (skip with ''skip-smoke'')\n');
    fprintf('  2. Routing sweep    — 4K × 6N × 4 scenarios × %d policies = %d runs\n\n', ...
            numel(policies), n_runs);
    fprintf('Press ENTER to continue, or Ctrl-C to abort.\n');
    pause;
end

% --- Common base config ---
cfg_base          = base_config_routing();
cfg_base.headless = headless;

% Baseline drone spacing derived from the reference config (same as experiment_routing)
base_raw    = build_scenario_config();
N_ref       = sum([base_raw.routes.numDrones]);
det_spacing = (base_raw.maxStartDelay - base_raw.minStartDelay) / max(N_ref - 1, 1);

% =====================================================================
% SMOKE TEST (~5 min) — verifies pipeline before committing to full run
% =====================================================================
if ~skip_smoke
    fprintf('\n──────────────────────────────────────────────────────────\n');
    fprintf('  STEP 1/2: SMOKE TEST\n');
    fprintf('──────────────────────────────────────────────────────────\n\n');
    smoke_t0 = tic;
    smoke_ok = run_smoke(OUT_ROOT, cfg_base);
    smoke_dt = toc(smoke_t0);
    fprintf('\n[SMOKE] Completed in %.1f min, status: %s\n', ...
            smoke_dt/60, ternary(smoke_ok, 'PASS', 'FAIL'));
    if ~smoke_ok
        fprintf('[SMOKE] FAILED — aborting. Inspect output above.\n');
        diary off;
        return;
    end
end

% =====================================================================
% ROUTING SWEEP — 4K × 6N × 4 scenarios × 6 policies  (parfor over K)
% =====================================================================
fprintf('\n──────────────────────────────────────────────────────────\n');
fprintf('  STEP %s/2: ROUTING SWEEP\n', ternary(skip_smoke, '1', '2'));
fprintf('──────────────────────────────────────────────────────────\n\n');

% Start parallel pool — one worker per K value (max 4).
nWorkers = min(numel(K_values), feature('numcores'));
pool = gcp('nocreate');
if isempty(pool)
    parpool('local', nWorkers);
    fprintf('[PARALLEL] Pool started: %d workers.\n', nWorkers);
else
    fprintf('[PARALLEL] Using existing pool: %d workers.\n', pool.NumWorkers);
end

sw_t0 = tic;

% Pre-allocate run_meta as cell matrix (K × N) — parfor requires indexed writes.
nK = numel(K_values);
nN = numel(drone_counts);
run_meta_cell = cell(nK, nN);

parfor ki = 1:nK
    K     = K_values(ki);
    local = struct('dir', {}, 'K', {}, 'N', {});

    for ni = 1:nN
        N = drone_counts(ni);
        fprintf('[K=%d N=%d] starting\n', K, N);

        cfg_n               = cfg_base;
        cfg_n.dronesPerSlot = K;
        cfg_n.maxStartDelay = cfg_n.minStartDelay + det_spacing * max(N - 1, 1);

        outDir_n = fullfile(OUT_ROOT, 'sweep', ...
                            sprintf('K%d', K), sprintf('N%03d', N));

        % Build sweep cfg via helper — avoids parfor struct-classification issue.
        run_routing_sweep(make_sweep_cfg(policies, N, cfg_n), outDir_n);

        local(end+1) = struct('dir', outDir_n, 'K', K, 'N', N); %#ok<AGROW>
        fprintf('[K=%d N=%d] done\n', K, N);
    end

    run_meta_cell{ki} = local;
end

% Flatten cell matrix into linear struct array for aggregation.
run_meta = [run_meta_cell{:}];

fprintf('\n[ROUTING] Completed in %.2f h\n', toc(sw_t0)/3600);

% ── Aggregate all per-(K,N) CSVs into one master summary ─────────────
aggregate_summaries(run_meta, fullfile(OUT_ROOT, 'sweep'));

% --- Final ---
fprintf('\n═══════════════════════════════════════════════════════════════\n');
fprintf('  EXPERIMENT_ROUTING_ALL_POLICIES COMPLETED\n');
fprintf('  Finished: %s\n', datestr(now)); %#ok<TNOW1,DATST>
fprintf('  Output: %s/sweep/sweep_routing_summary_all.csv\n', OUT_ROOT);
fprintf('  Log:    %s\n', logFile);
fprintf('═══════════════════════════════════════════════════════════════\n\n');
diary off;
end


%% ═══════════════════════════════════════════════════════════════════
%%  Routing scenario definitions — identical to experiment_routing
%% ═══════════════════════════════════════════════════════════════════
function scenarios = define_routing_scenarios(N_tot)
base = build_scenario_config();
nR   = numel(base.routes);

% ── 1-route ──────────────────────────────────────────────────────────
r1           = base.routes(1);
r1.numDrones = N_tot;

s1.label        = '1-route static';
s1.routing_mode = 'static';
s1.routes       = r1;

s2.label        = '1-route A*';
s2.routing_mode = 'risk_astar';
s2.routes       = r1;

% ── N-route: distribute N_tot evenly across all routes ───────────────
routes_n  = base.routes;
per_route = floor(N_tot / nR);
remainder = N_tot - per_route * nR;
for k = 1:nR
    routes_n(k).numDrones = per_route + (k <= remainder);
end

lbl_n = sprintf('%d-route', nR);

s3.label        = [lbl_n ' static'];
s3.routing_mode = 'static';
s3.routes       = routes_n;

s4.label        = [lbl_n ' A*'];
s4.routing_mode = 'risk_astar';
s4.routes       = routes_n;

scenarios = [s1, s2, s3, s4];
end


%% ═══════════════════════════════════════════════════════════════════
%%  Base config — inherits scenario, adds all doc-aligned params
%% ═══════════════════════════════════════════════════════════════════
function cfg = base_config_routing()
cfg = build_scenario_config();
cfg.csvExport         = true;
cfg.use_normalization = false;
% Max-weight-doc params (Sec. 4)
cfg.V_s               = 1.0;
cfg.V_v               = 1.0;
cfg.kappa             = 0.0;
% Max-weight-drift params (Sec. 2); beta_s/beta_v will default to n_s/n_v
% in validate_uam_config if left unset, but we set V explicitly here.
cfg.V                 = 1.0;
end


%% ═══════════════════════════════════════════════════════════════════
%%  Smoke test — 2 policies × 1 scenario (1-route static), 1:10 scale
%% ═══════════════════════════════════════════════════════════════════
function ok = run_smoke(outRoot, cfg_base)
ok = true;

cfg_smoke = cfg_base;
scale = 0.1;
r1    = cfg_base.routes(1);

rSmoke.label     = r1.label;
rSmoke.start     = r1.start * scale;
rSmoke.goal      = r1.goal  * scale;
rSmoke.numDrones = 3;

cfg_smoke.routes             = rSmoke;
cfg_smoke.corridorLength     = norm(rSmoke.goal - rSmoke.start);
cfg_smoke.flightTime         = cfg_smoke.corridorLength / cfg_base.speedVal;
cfg_smoke.maxStartDelay      = 30;
cfg_smoke.manualHotspots     = cfg_base.manualHotspots * scale;
cfg_smoke.microBSPos(:, 1:2) = cfg_base.microBSPos(:, 1:2) * scale;

sc.policies  = {'max-weight-drift', 'pf-classic'};
sc.scenarios = struct('label', 'smoke-static', 'routing_mode', 'static', 'routes', rSmoke);
sc.base_cfg  = cfg_smoke;
sc.headless  = cfg_base.headless;

try
    res = run_routing_sweep(sc, fullfile(outRoot, 'smoke'));
    for k = 1:numel(res)
        if isfield(res(k), 'error') && ~isempty(res(k).error)
            fprintf('[SMOKE] Run failed: policy=%s, error=%s\n', ...
                    res(k).policy, res(k).error);
            ok = false;
        elseif isfield(res(k), 'rho_emp') && ~isempty(res(k).rho_emp)
            % Check only that the simulation produced finite, positive output.
            % rho < 1 is expected in a short scaled scenario (transient Jhat_emp
            % can fall below the steady-state lower bound); it does not indicate
            % a crash or a broken pipeline.
            if ~isfinite(res(k).rho_emp) || res(k).rho_emp <= 0
                fprintf('[SMOKE] Invalid rho_emp=%.3f for policy=%s\n', ...
                        res(k).rho_emp, res(k).policy);
                ok = false;
            else
                fprintf('[SMOKE] %s: rho_emp=%.3f (OK)\n', ...
                        res(k).policy, res(k).rho_emp);
            end
        end
    end
catch ME
    fprintf('[SMOKE] EXCEPTION: %s\n', ME.message);
    if ~isempty(ME.stack)
        fprintf('[SMOKE] Stack: %s:%d\n', ME.stack(1).name, ME.stack(1).line);
    end
    ok = false;
end
end


%% ═══════════════════════════════════════════════════════════════════
%%  Aggregate per-(K,N) CSVs into a single master summary.
%%  Prepends a 'K' column to identify the dronesPerSlot dimension.
%% ═══════════════════════════════════════════════════════════════════
function aggregate_summaries(run_meta, masterDir)
masterFile = fullfile(masterDir, 'sweep_routing_summary_all.csv');
fid = fopen(masterFile, 'w');
header_written = false;

for d = 1:numel(run_meta)
    src = fullfile(run_meta(d).dir, 'sweep_routing_summary.csv');
    if ~exist(src, 'file'), continue; end

    K_val   = run_meta(d).K;
    fid_src = fopen(src, 'r');
    header  = fgetl(fid_src);

    if ~header_written
        fprintf(fid, 'K,%s\n', header);
        header_written = true;
    end

    while true
        line = fgetl(fid_src);
        if ~ischar(line), break; end
        if ~isempty(strtrim(line))
            fprintf(fid, '%d,%s\n', K_val, line);
        end
    end
    fclose(fid_src);
end
fclose(fid);
fprintf('\n[ROUTING] Aggregated summary → %s\n', masterFile);
end


%% ═══════════════════════════════════════════════════════════════════
%%  Build sweep_cfg struct — isolated so parfor can classify all vars
%% ═══════════════════════════════════════════════════════════════════
function sc = make_sweep_cfg(policies, N, cfg_n)
sc.policies  = policies;
sc.scenarios = define_routing_scenarios(N);
sc.base_cfg  = cfg_n;
sc.headless  = true;
end


%% ═══════════════════════════════════════════════════════════════════
%%  Helper
%% ═══════════════════════════════════════════════════════════════════
function r = ternary(cond, a, b)
if cond, r = a; else, r = b; end
end
