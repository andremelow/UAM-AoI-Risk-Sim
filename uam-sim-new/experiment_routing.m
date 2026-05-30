function experiment_routing(varargin)
%% EXPERIMENT_ROUTING  Full sweep — policy × routing × load × K.
%
%  Sweeps four dimensions:
%    K        — dronesPerSlot ∈ [1 2 4 8]
%    Load     — N ∈ [2 4 8 16 32 64] (doubling)
%    Routing  — static | Risk-A*
%    Routes   — 1 route | 2 routes   (→ 4 routing scenarios)
%    Policy   — round-robin, round-robin-aware, aoi-pure, risk-aware, max-weight
%
%  Total: 4K × 6N × 4 scenarios × 5 policies = 480 runs.
%
%  Output per (K,N): csv_export/routing/sweep_routing/K{K}/N{N}/sweep_routing_summary.csv
%  Aggregated:       csv_export/routing/sweep_routing/sweep_routing_summary_all.csv
%
%  Uso:
%    experiment_routing()                % roda tudo, dashboard visível
%    experiment_routing('skip-smoke')    % pula smoke test (já validado)
%    experiment_routing('--yes')         % sem confirmação (background)
%    experiment_routing('--headless')    % sem dashboard (batch / servidor)

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

% --- Sweep axes ---
K_values     = [1, 2, 4, 8];
drone_counts = [2, 4, 8, 16, 32, 64];

% --- Output root (absolute, anchored to this script's directory) ---
SIM_ROOT = fileparts(mfilename('fullpath'));
OUT_ROOT = fullfile(SIM_ROOT, 'csv_export', 'routing');
system(sprintf('mkdir -p "%s"', OUT_ROOT));

% --- Logfile ---
ts      = datestr(now, 'yyyymmdd_HHMMSS'); %#ok<TNOW1,DATST>
logFile = fullfile(OUT_ROOT, sprintf('experiment_routing_%s.log', ts));
diary(logFile); diary on;

fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  EXPERIMENT_ROUTING — K × load × routing × policy sweep\n');
fprintf('  Started: %s\n', datestr(now)); %#ok<TNOW1,DATST>
fprintf('  Log: %s\n', logFile);
fprintf('═══════════════════════════════════════════════════════════════\n\n');

% --- Confirmation ---
if ~auto_confirm
    fprintf('This will run:\n');
    fprintf('  1. Smoke test      — ~5 min  (skip with ''skip-smoke'')\n');
    fprintf('  2. Routing sweep   — 4K × 6N × 4 scenarios × 5 policies = 480 runs\n\n');
    fprintf('Press ENTER to continue, or Ctrl-C to abort.\n');
    pause;
end

% --- Common base config ---
cfg_base          = base_config_routing();
cfg_base.headless = headless;

% Baseline drone spacing (s/drone) derived from the reference config
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
% ROUTING SWEEP — 4K × 6N × 4 scenarios × 5 policies
% =====================================================================
fprintf('\n──────────────────────────────────────────────────────────\n');
fprintf('  STEP %s/2: ROUTING SWEEP\n', ternary(skip_smoke,'1','2'));
fprintf('──────────────────────────────────────────────────────────\n\n');

sw_t0    = tic;
policies = {'round-robin','round-robin-aware','aoi-pure','risk-aware','max-weight'};
run_meta = struct('dir',{},'K',{},'N',{});   % track dirs for aggregation

total_blocks = numel(K_values) * numel(drone_counts);
block_idx    = 0;

for ki = 1:numel(K_values)
    K = K_values(ki);
    fprintf('\n╔══ K = %d simultaneous slots ══╗\n', K);

    for ni = 1:numel(drone_counts)
        N = drone_counts(ni);
        block_idx = block_idx + 1;
        fprintf('\n  ── N = %d drones  [block %d/%d] ──\n', N, block_idx, total_blocks);

        % Per-block config: override K and scale start delays
        cfg_n = cfg_base;
        cfg_n.dronesPerSlot = K;
        cfg_n.maxStartDelay = cfg_n.minStartDelay + det_spacing * max(N - 1, 1);

        sc.policies  = policies;
        sc.scenarios = define_routing_scenarios(N);
        sc.base_cfg  = cfg_n;
        sc.headless  = headless;

        outDir_n = fullfile(OUT_ROOT, 'sweep_routing', ...
                            sprintf('K%d', K), sprintf('N%03d', N));
        run_routing_sweep(sc, outDir_n);

        run_meta(end+1) = struct('dir', outDir_n, 'K', K, 'N', N); %#ok<AGROW>
    end
end

fprintf('\n[ROUTING] Completed in %.2f h\n', toc(sw_t0)/3600);

% ── Aggregate all per-(K,N) CSVs into one master summary ─────────────
aggregate_summaries(run_meta, fullfile(OUT_ROOT, 'sweep_routing'));

% --- Final ---
fprintf('\n═══════════════════════════════════════════════════════════════\n');
fprintf('  EXPERIMENT_ROUTING COMPLETED\n');
fprintf('  Finished: %s\n', datestr(now)); %#ok<TNOW1,DATST>
fprintf('  Output: %s/sweep_routing/sweep_routing_summary_all.csv\n', OUT_ROOT);
fprintf('  Log:    %s\n', logFile);
fprintf('═══════════════════════════════════════════════════════════════\n\n');
diary off;
end


%% ═══════════════════════════════════════════════════════════════════
%%  Routing scenario definitions — parameterised by total drone count
%% ═══════════════════════════════════════════════════════════════════
function scenarios = define_routing_scenarios(N_tot)
% Routes derived from build_scenario_config; N_tot overrides numDrones.
%
% 1-route: all N_tot drones on route 1.
% N-route: drones distributed evenly across all routes (ceil/floor split).

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
%%  Base config (inherits background scenario, adds doc-aligned params)
%% ═══════════════════════════════════════════════════════════════════
function cfg = base_config_routing()
cfg = build_scenario_config();
cfg.csvExport         = true;
cfg.use_normalization = false;
cfg.V_s               = 1.0;
cfg.V_v               = 1.0;
cfg.kappa             = 0.0;
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

sc.policies  = {'round-robin', 'max-weight'};
sc.scenarios = struct('label','smoke-static','routing_mode','static','routes',rSmoke);
sc.base_cfg  = cfg_smoke;
sc.headless  = cfg_base.headless;

try
    res = run_routing_sweep(sc, fullfile(outRoot, 'smoke'));
    for k = 1:numel(res)
        if isfield(res(k),'error') && ~isempty(res(k).error)
            fprintf('[SMOKE] Run failed: policy=%s, error=%s\n', ...
                    res(k).policy, res(k).error);
            ok = false;
        elseif isfield(res(k),'rho_emp') && ~isempty(res(k).rho_emp)
            if isnan(res(k).rho_emp) || res(k).rho_emp < 0.99
                fprintf('[SMOKE] Suspicious rho_emp=%.3f for policy=%s\n', ...
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
%%  Aggregate per-(K,N) CSVs into a single master summary
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
%%  Helper
%% ═══════════════════════════════════════════════════════════════════
function r = ternary(cond, a, b)
if cond, r = a; else, r = b; end
end
