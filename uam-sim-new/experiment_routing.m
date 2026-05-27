function experiment_routing(varargin)
% EXPERIMENT_ROUTING  Cross-product: all scheduling policies × 4 routing scenarios.
%
%  Routing scenarios (5 drones total in all cases):
%    s1 — 1 route,  static routing
%    s2 — 1 route,  Risk-A* routing
%    s3 — 2 routes, static routing   (3 + 2 drones)
%    s4 — 2 routes, Risk-A* routing  (3 + 2 drones)
%
%  Policies: round-robin, round-robin-aware, aoi-pure, risk-aware, max-weight
%
%  Usage:
%    experiment_routing()           % run all 20 combinations (asks for confirmation)
%    experiment_routing('--yes')    % skip confirmation prompt
%
%  Output (csv_export/routing_experiment/):
%    routing_experiment_<ts>.mat          — cell array of all result structs
%    routing_experiment_<ts>_summary.csv  — one row per (scenario, policy)

setup_paths();

auto_confirm = any(strcmp(varargin, '--yes'));

POLICIES  = {'round-robin','round-robin-aware','aoi-pure','risk-aware','max-weight'};
scenarios = define_scenarios();

nP = numel(POLICIES);
nS = numel(scenarios);

% ── Output directory ─────────────────────────────────────────────────
OUT_DIR = fullfile('csv_export', 'routing_experiment');
if ~exist(OUT_DIR, 'dir'), mkdir(OUT_DIR); end
ts       = datestr(now, 'yyyymmdd_HHMMSS'); %#ok<TNOW1,DATST>
mat_file = fullfile(OUT_DIR, sprintf('routing_experiment_%s.mat',          ts));
csv_file = fullfile(OUT_DIR, sprintf('routing_experiment_%s_summary.csv',  ts));
log_file = fullfile(OUT_DIR, sprintf('routing_experiment_%s.log',          ts));

diary(log_file); diary on;

% ── Header ───────────────────────────────────────────────────────────
fprintf('\n');
fprintf('╔═══════════════════════════════════════════════════════════════════╗\n');
fprintf('║  ROUTING EXPERIMENT                                               ║\n');
fprintf('║  %d policies × %d scenarios = %2d runs                              ║\n', nP, nS, nP*nS);
fprintf('║  Started: %-55s║\n', datestr(now)); %#ok<TNOW1,DATST>
fprintf('╚═══════════════════════════════════════════════════════════════════╝\n\n');

for si = 1:nS
    sc = scenarios(si);
    nd = sum([sc.routes.numDrones]);
    fprintf('  Scenario %d: %-22s  routing=%-12s  routes=%d  drones=%d\n', ...
            si, sc.label, sc.routing_mode, numel(sc.routes), nd);
end
fprintf('\n  Policies: %s\n\n', strjoin(POLICIES, ' | '));

if ~auto_confirm
    fprintf('Press ENTER to start %d runs, or Ctrl-C to abort.\n', nP*nS);
    pause;
end

% ── Run matrix ───────────────────────────────────────────────────────
% Use a cell array to avoid struct-field mismatch between successful results
% (many fields from collect_results) and error stubs.
results_all = cell(nP * nS, 1);
run_idx = 0;

for si = 1:nS
    sc = scenarios(si);
    fprintf('\n── Scenario %d/%d: %s (%s) ──────────────────────────────────────\n', ...
            si, nS, sc.label, sc.routing_mode);

    for pi = 1:nP
        run_idx = run_idx + 1;
        pol = POLICIES{pi};

        fprintf('[%2d/%2d] %-22s  %-20s  ...', run_idx, nP*nS, sc.label, pol);
        t0 = tic;

        cfg = build_run_cfg(sc);
        cfg.schedulingPolicy = pol;

        try
            res = run_sim(cfg);
            % Tag with experiment metadata
            res.scenario_idx = si;
            res.scenario_lbl = sc.label;
            res.routing_mode = sc.routing_mode;
            res.num_routes   = numel(sc.routes);
            res.run_error    = '';
            % Time-averaged h1/h2 (drop first 20% warm-up)
            res.h1_tavg = tavg(res.h1MeanV);
            res.h2_tavg = tavg(res.h2MeanV);

        catch ME
            res = struct( ...
                'scenario_idx', si,              ...
                'scenario_lbl', sc.label,        ...
                'routing_mode', sc.routing_mode, ...
                'num_routes',   numel(sc.routes),...
                'policy',       pol,             ...
                'numDrones',    sum([sc.routes.numDrones]), ...
                'Jhat_emp',     NaN, 'J_LB',    NaN, ...
                'rho_emp',      NaN, 'Rsys_emp', NaN, ...
                'h1_tavg',      NaN, 'h2_tavg',  NaN, ...
                'h1MeanV',      [], 'h2MeanV',   [], ...
                'run_error',    ME.message);
        end

        dt = toc(t0);
        if isempty(res.run_error)
            fprintf('  rho=%5.3f  Rsys=%.2e  h1=%6.1f  h2=%6.1f  (%.0fs)\n', ...
                    res.rho_emp, res.Rsys_emp, res.h1_tavg, res.h2_tavg, dt);
        else
            fprintf('  ERROR: %s\n', res.run_error);
        end

        results_all{run_idx} = res;
    end
end

% ── Save ─────────────────────────────────────────────────────────────
save(mat_file, 'results_all', 'scenarios', 'POLICIES');
fprintf('\nRaw results: %s\n', mat_file);

% ── Summary table and CSV ────────────────────────────────────────────
print_summary(results_all, POLICIES, scenarios);
write_csv(results_all, csv_file);
fprintf('CSV summary:  %s\n', csv_file);
fprintf('Log:          %s\n\n', log_file);

diary off;
end


%% ═══════════════════════════════════════════════════════════════════
%%  Scenario definitions
%% ═══════════════════════════════════════════════════════════════════
function scenarios = define_scenarios()

% Shared corridor geometry (30 km, matching build_scenario_config)
rA.label = 'Route A';  rA.start = [-15000,    0];  rA.goal = [ 15000,    0];
rB.label = 'Route B';  rB.start = [-15000, -200];  rB.goal = [ 15000,  200];

% s1: 1 route, static
s1.label        = '1-route static';
s1.routing_mode = 'static';
r = rA; r.numDrones = 5;
s1.routes = r;

% s2: 1 route, Risk-A*
s2.label        = '1-route A*';
s2.routing_mode = 'risk_astar';
r = rA; r.numDrones = 5;
s2.routes = r;

% s3: 2 routes, static
s3.label        = '2-route static';
s3.routing_mode = 'static';
ra = rA; ra.numDrones = 3;
rb = rB; rb.numDrones = 2;
s3.routes = [ra, rb];

% s4: 2 routes, Risk-A*
s4.label        = '2-route A*';
s4.routing_mode = 'risk_astar';
ra = rA; ra.numDrones = 3;
rb = rB; rb.numDrones = 2;
s4.routes = [ra, rb];

scenarios = [s1, s2, s3, s4];
end


%% ═══════════════════════════════════════════════════════════════════
%%  Build cfg for a single run
%% ═══════════════════════════════════════════════════════════════════
function cfg = build_run_cfg(sc)
cfg = build_scenario_config();
cfg.routes         = sc.routes;
cfg.corridorLength = norm(cfg.routes(1).goal - cfg.routes(1).start);
cfg.flightTime     = cfg.corridorLength / cfg.speedVal;
cfg.routing.mode   = sc.routing_mode;
cfg.headless       = true;
cfg.csvExport      = false;
end


%% ═══════════════════════════════════════════════════════════════════
%%  Time-average helper (warm-up = first 20%)
%% ═══════════════════════════════════════════════════════════════════
function v = tavg(series)
if isempty(series), v = NaN; return; end
T    = numel(series);
warm = max(1, round(0.2 * T));
if T > warm, v = mean(series(warm:end)); else, v = mean(series); end
end


%% ═══════════════════════════════════════════════════════════════════
%%  Console summary table
%% ═══════════════════════════════════════════════════════════════════
function print_summary(results_all, policies, scenarios)
nP = numel(policies);
nS = numel(scenarios);

hdr = sprintf('  %-20s | %10s | %10s | %7s | %10s | %8s | %8s', ...
              'Policy','Jhat_emp','J_LB','rho','Rsys_emp','h1_tavg','h2_tavg');
sep = repmat('-', 1, 82);

fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════════════════════════\n');
fprintf('  RESULTS SUMMARY\n');
fprintf('═══════════════════════════════════════════════════════════════════════════════════\n');

for si = 1:nS
    sc = scenarios(si);
    nd = sum([sc.routes.numDrones]);
    fprintf('\n  ▸ Scenario %d: %s   routing=%-12s  routes=%d  drones=%d\n', ...
            si, sc.label, sc.routing_mode, numel(sc.routes), nd);
    fprintf('  %s\n%s\n  %s\n', sep, hdr, sep);

    for pi = 1:nP
        pol = policies{pi};
        r   = find_result(results_all, sc.label, pol);

        if isempty(r)
            fprintf('  %-20s | NO DATA\n', pol);
        elseif isempty(r.run_error)
            fprintf('  %-20s | %10.3e | %10.3e | %7.3f | %10.3e | %8.1f | %8.1f\n', ...
                    pol, r.Jhat_emp, r.J_LB, r.rho_emp, r.Rsys_emp, ...
                    r.h1_tavg, r.h2_tavg);
        else
            fprintf('  %-20s | ERROR: %s\n', pol, r.run_error);
        end
    end
    fprintf('  %s\n', sep);
end
fprintf('\n');
end


%% ═══════════════════════════════════════════════════════════════════
%%  CSV export
%% ═══════════════════════════════════════════════════════════════════
function write_csv(results_all, csv_file)
fid = fopen(csv_file, 'w');
fprintf(fid, ['scenario,routing_mode,num_routes,num_drones,policy,' ...
              'Jhat_emp,J_LB,rho_emp,Rsys_emp,h1_tavg,h2_tavg,error\n']);
for k = 1:numel(results_all)
    r  = results_all{k};
    nd = 0; if isfield(r,'numDrones'), nd = r.numDrones; end
    fprintf(fid, '%s,%s,%d,%d,%s,%.6e,%.6e,%.6f,%.6e,%.4f,%.4f,"%s"\n', ...
            r.scenario_lbl, r.routing_mode, r.num_routes, nd, r.policy, ...
            r.Jhat_emp, r.J_LB, r.rho_emp, r.Rsys_emp, ...
            r.h1_tavg, r.h2_tavg, strrep(r.run_error, '"', "'"));
end
fclose(fid);
end


%% ═══════════════════════════════════════════════════════════════════
%%  Helper: find result by scenario label + policy
%% ═══════════════════════════════════════════════════════════════════
function r = find_result(results_all, sc_lbl, pol)
r = [];
for k = 1:numel(results_all)
    c = results_all{k};
    if strcmp(c.scenario_lbl, sc_lbl) && strcmp(c.policy, pol)
        r = c;
        return;
    end
end
end
