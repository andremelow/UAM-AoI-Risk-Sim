function experiment_routing_pf(varargin)
%% EXPERIMENT_ROUTING_PF  Complementary sweep — pf-classic only.
%
%  Runs pf-classic across the same K × N × routing dimensions as
%  experiment_routing, writing output to the SAME directory tree:
%
%    csv_export/routing/sweep_routing/K{K}/N{N}/
%
%  This allows the R aggregator (aggregate_sim_data.R) to merge all
%  policies in a single pass and the dashboard to show everything together.
%
%  Total: 4K × 6N × 4 scenarios × 1 policy = 96 runs.
%
%  Uso:
%    experiment_routing_pf()            % roda tudo, dashboard visível
%    experiment_routing_pf('--yes')     % sem confirmação
%    experiment_routing_pf('--headless')

setup_paths();

% --- Argument parsing ---
auto_confirm = false;
headless     = false;
ai = 1;
while ai <= numel(varargin)
    switch varargin{ai}
        case '--yes',      auto_confirm = true;
        case '--headless', headless     = true;
    end
    ai = ai + 1;
end

% --- Sweep axes (must match experiment_routing) ---
K_values     = [1, 2, 4, 8];
drone_counts = [2, 4, 8, 16, 32, 64];

% --- Output root (same as experiment_routing) ---
OUT_ROOT = fullfile('csv_export', 'routing');
system(sprintf('mkdir -p "%s"', OUT_ROOT));

% --- Logfile ---
ts      = datestr(now, 'yyyymmdd_HHMMSS'); %#ok<TNOW1,DATST>
logFile = fullfile(OUT_ROOT, sprintf('experiment_routing_pf_%s.log', ts));
diary(logFile); diary on;

fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  EXPERIMENT_ROUTING_PF — pf-classic complement\n');
fprintf('  Started: %s\n', datestr(now)); %#ok<TNOW1,DATST>
fprintf('  Log: %s\n', logFile);
fprintf('═══════════════════════════════════════════════════════════════\n\n');

if ~auto_confirm
    fprintf('This will run:\n');
    fprintf('  pf-classic: 4K × 6N × 4 scenarios = 96 runs\n');
    fprintf('  Output merges into existing sweep_routing/ tree.\n\n');
    fprintf('Press ENTER to continue, or Ctrl-C to abort.\n');
    pause;
end

% --- Base config ---
cfg_base          = base_config_routing();
cfg_base.headless = headless;

base_raw    = build_scenario_config();
N_ref       = sum([base_raw.routes.numDrones]);
det_spacing = (base_raw.maxStartDelay - base_raw.minStartDelay) / max(N_ref - 1, 1);

% =====================================================================
% SWEEP
% =====================================================================
fprintf('\n──────────────────────────────────────────────────────────\n');
fprintf('  pf-classic sweep\n');
fprintf('──────────────────────────────────────────────────────────\n\n');

sw_t0       = tic;
total_blocks = numel(K_values) * numel(drone_counts);
block_idx    = 0;

for ki = 1:numel(K_values)
    K = K_values(ki);
    fprintf('\n╔══ K = %d simultaneous slots ══╗\n', K);

    for ni = 1:numel(drone_counts)
        N = drone_counts(ni);
        block_idx = block_idx + 1;
        fprintf('\n  ── N = %d drones  [block %d/%d] ──\n', N, block_idx, total_blocks);

        cfg_n = cfg_base;
        cfg_n.dronesPerSlot = K;
        cfg_n.maxStartDelay = cfg_n.minStartDelay + det_spacing * max(N - 1, 1);

        sc.policies  = {'pf-classic'};
        sc.scenarios = define_routing_scenarios(N);
        sc.base_cfg  = cfg_n;
        sc.headless  = headless;

        outDir_n = fullfile(OUT_ROOT, 'sweep_routing', ...
                            sprintf('K%d', K), sprintf('N%03d', N));
        run_routing_sweep(sc, outDir_n);
    end
end

fprintf('\n[PF] Completed in %.2f h\n', toc(sw_t0)/3600);

fprintf('\n═══════════════════════════════════════════════════════════════\n');
fprintf('  EXPERIMENT_ROUTING_PF COMPLETED\n');
fprintf('  Finished: %s\n', datestr(now)); %#ok<TNOW1,DATST>
fprintf('  Output merged into: %s/sweep_routing/\n', OUT_ROOT);
fprintf('  Next: run aggregate_sim_data.R to rebuild agg/ for dashboard.\n');
fprintf('  Log: %s\n', logFile);
fprintf('═══════════════════════════════════════════════════════════════\n\n');
diary off;
end


%% ═══════════════════════════════════════════════════════════════════
%%  Helpers (mirrors experiment_routing)
%% ═══════════════════════════════════════════════════════════════════
function scenarios = define_routing_scenarios(N_tot)
base = build_scenario_config();
nR   = numel(base.routes);

r1           = base.routes(1);
r1.numDrones = N_tot;

s1.label = '1-route static'; s1.routing_mode = 'static';     s1.routes = r1;
s2.label = '1-route A*';     s2.routing_mode = 'risk_astar'; s2.routes = r1;

routes_n  = base.routes;
per_route = floor(N_tot / nR);
remainder = N_tot - per_route * nR;
for k = 1:nR
    routes_n(k).numDrones = per_route + (k <= remainder);
end
lbl_n = sprintf('%d-route', nR);
s3.label = [lbl_n ' static']; s3.routing_mode = 'static';     s3.routes = routes_n;
s4.label = [lbl_n ' A*'];     s4.routing_mode = 'risk_astar'; s4.routes = routes_n;

scenarios = [s1, s2, s3, s4];
end


function cfg = base_config_routing()
cfg = build_scenario_config();
cfg.csvExport         = true;
cfg.use_normalization = false;
cfg.V_s               = 1.0;
cfg.V_v               = 1.0;
cfg.kappa             = 0.0;
end
