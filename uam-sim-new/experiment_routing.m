function experiment_routing(varargin)
%% EXPERIMENT_ROUTING  Routing-dimension sweep — all policies × 4 routing scenarios.
%
%  Mirrors sweep_paper.m structure; uses run_routing_sweep as the workhorse
%  (analog of run_sweep) and produces the same rich metrics / CSV format.
%
%  Scenarios (5 drones total each):
%    s1 — 1 route,  static routing
%    s2 — 1 route,  Risk-A* routing
%    s3 — 2 routes, static routing   (3 + 2 drones)
%    s4 — 2 routes, Risk-A* routing  (3 + 2 drones)
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

% --- Output root ---
OUT_ROOT = fullfile('csv_export', 'routing');
if ~exist(OUT_ROOT,'dir'), mkdir(OUT_ROOT); end

% --- Logfile ---
ts      = datestr(now, 'yyyymmdd_HHMMSS'); %#ok<TNOW1,DATST>
logFile = fullfile(OUT_ROOT, sprintf('experiment_routing_%s.log', ts));
diary(logFile); diary on;

fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  EXPERIMENT_ROUTING — all policies × routing scenarios\n');
fprintf('  Started: %s\n', datestr(now)); %#ok<TNOW1,DATST>
fprintf('  Log: %s\n', logFile);
fprintf('═══════════════════════════════════════════════════════════════\n\n');

% --- Confirmation ---
if ~auto_confirm
    fprintf('This will run:\n');
    fprintf('  1. Smoke test      — ~5 min  (skip with ''skip-smoke'')\n');
    fprintf('  2. Routing sweep   — 4 scenarios × 5 policies = 20 runs\n\n');
    fprintf('Press ENTER to continue, or Ctrl-C to abort.\n');
    pause;
end

% --- Common base config ---
cfg_base          = base_config_routing();
cfg_base.headless = headless;

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
% ROUTING SWEEP — all 4 scenarios × all 5 policies
% =====================================================================
fprintf('\n──────────────────────────────────────────────────────────\n');
fprintf('  STEP %s/2: ROUTING SWEEP\n', ternary(skip_smoke,'1','2'));
fprintf('──────────────────────────────────────────────────────────\n\n');

sw_t0 = tic;
sc.policies  = {'round-robin','round-robin-aware','aoi-pure','risk-aware','max-weight'};
sc.scenarios = define_routing_scenarios();
sc.base_cfg  = cfg_base;
sc.headless  = headless;
run_routing_sweep(sc, fullfile(OUT_ROOT, 'sweep_routing'));
fprintf('[ROUTING] Completed in %.2f h\n', toc(sw_t0)/3600);

% --- Final ---
fprintf('\n═══════════════════════════════════════════════════════════════\n');
fprintf('  EXPERIMENT_ROUTING COMPLETED\n');
fprintf('  Finished: %s\n', datestr(now)); %#ok<TNOW1,DATST>
fprintf('  Output: %s/sweep_routing/sweep_routing_summary.csv\n', OUT_ROOT);
fprintf('  Log:    %s\n', logFile);
fprintf('═══════════════════════════════════════════════════════════════\n\n');
diary off;
end


%% ═══════════════════════════════════════════════════════════════════
%%  Routing scenario definitions
%% ═══════════════════════════════════════════════════════════════════
function scenarios = define_routing_scenarios()
% Route geometry from build_scenario_config (Interlagos scenario):
%   Route 1: East-West corridor  [0,-3000] → [0,+3000]  (6 km)
%   Route 2: Angled return      [500,+3000] → [200,-3000]
% 10 drones total in all scenarios (matches base config).

rA.label = 'Route 1';  rA.start = [   0, -3000];  rA.goal = [   0,  3000];
rB.label = 'Route 2';  rB.start = [ 500,  3000];  rB.goal = [ 200, -3000];

s1.label = '1-route static';   s1.routing_mode = 'static';
r = rA; r.numDrones = 10;      s1.routes = r;

s2.label = '1-route A*';       s2.routing_mode = 'risk_astar';
r = rA; r.numDrones = 10;      s2.routes = r;

s3.label = '2-route static';   s3.routing_mode = 'static';
ra = rA; ra.numDrones = 5;
rb = rB; rb.numDrones = 5;     s3.routes = [ra, rb];

s4.label = '2-route A*';       s4.routing_mode = 'risk_astar';
ra = rA; ra.numDrones = 5;
rb = rB; rb.numDrones = 5;     s4.routes = [ra, rb];

scenarios = [s1, s2, s3, s4];
end


%% ═══════════════════════════════════════════════════════════════════
%%  Base config (inherits A2 geometry, adds doc-aligned params)
%% ═══════════════════════════════════════════════════════════════════
function cfg = base_config_routing()
cfg = build_scenario_config();   % herda cenário A2 completo
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

% Smoke test: 1:10 scale of Route 1, 3 drones, no manual hotspots
cfg_smoke = cfg_base;
scale = 0.1;
cfg_smoke.corridorLength  = cfg_base.corridorLength * scale;
cfg_smoke.flightTime      = cfg_smoke.corridorLength / cfg_base.speedVal;
cfg_smoke.maxStartDelay   = 30;
cfg_smoke.manualHotspots  = cfg_base.manualHotspots * scale;
cfg_smoke.microBSPos(:, 1:2) = cfg_base.microBSPos(:, 1:2) * scale;

rSmoke.label     = 'Route 1';
rSmoke.start     = [0, -cfg_smoke.corridorLength/2];
rSmoke.goal      = [0,  cfg_smoke.corridorLength/2];
rSmoke.numDrones = 3;
cfg_smoke.routes = rSmoke;

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
%%  Helper
%% ═══════════════════════════════════════════════════════════════════
function r = ternary(cond, a, b)
if cond, r = a; else, r = b; end
end
