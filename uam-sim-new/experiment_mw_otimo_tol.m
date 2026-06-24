function experiment_mw_otimo_tol(varargin)
%% EXPERIMENT_MW_OTIMO_TOL
%  Replica de experiment_mw_otimo_a23 com perfil de altitude takeoff/landing:
%    - altProfile  = 'takeoff_landing'
%    - cruiseAlt   = 90 m AGL   (vs 60 m no modo fixo)
%    - takeoffDist = 60 m       (rampa nos 60m iniciais/finais de cada rota)
%    - w_tol_vid   = 0.80       (peso de vídeo durante rampa)
%    - w_tol_unc   = 0.10
%    - w_tol_map   = 0.10
%
%  Políticas : round-robin-sw | pf-classic-sw | mw-sw-otimo | max-risk-sw
%  Cenários  : 2r_static | 2r_a23
%  Eixos     : K ∈ {1,2,4,8,16,32}  ×  C ∈ {2,4,8,16,32,48,64}
%  Total     : 6K × 7C × 2sc × 4pol = 336 runs    T_steady = 900 s
%
%  Saída: uam-sim-new/csv_export_mw_otimo_tol/
%
%  Usage:
%    experiment_mw_otimo_tol()
%    experiment_mw_otimo_tol('--yes')
%    experiment_mw_otimo_tol('--smoke-only')
%    experiment_mw_otimo_tol('--C','4,16,48')
%    experiment_mw_otimo_tol('--K','4,8,32')
%    experiment_mw_otimo_tol('--scenario','2r_a23')
%    experiment_mw_otimo_tol('--policy','max-risk-sw')

setup_paths();

%% ── Argument parsing ─────────────────────────────────────────────────
skip_smoke      = false;
auto_confirm    = false;
smoke_only      = false;
cap_override    = [];
k_override      = [];
scenario_filter = '';
policy_filter   = '';

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
        case '--scenario'
            ai = ai + 1;
            if ai <= numel(varargin)
                scenario_filter = varargin{ai};
            end
        case '--policy'
            ai = ai + 1;
            if ai <= numel(varargin)
                policy_filter = varargin{ai};
            end
    end
    ai = ai + 1;
end

%% ── Sweep axes ───────────────────────────────────────────────────────
if isempty(cap_override)
    capacity_values = [2, 4, 8, 16, 32, 48, 64];
else
    capacity_values = cap_override;
end
if isempty(k_override)
    K_values = [1, 2, 4, 8, 16, 32];
else
    K_values = k_override;
end
policies = {'round-robin-sw', 'pf-classic-sw', 'mw-sw-otimo', 'max-risk-sw'};
if ~isempty(policy_filter)
    policies = policies(strcmp(policies, policy_filter));
    if isempty(policies)
        error('--policy "%s" inválida.', policy_filter);
    end
end
T_STEADY = 900;

SIM_ROOT = fileparts(mfilename('fullpath'));
OUT_ROOT = fullfile(SIM_ROOT, 'csv_export_mw_otimo_tol');

logFile = fullfile(SIM_ROOT, sprintf('experiment_mw_otimo_tol_%s.log', ...
          datestr(now,'yyyymmdd_HHMMSS'))); %#ok<TNOW1,DATST>
diary(logFile);

scenarios = define_scenarios_tol();
if ~isempty(scenario_filter)
    keep = arrayfun(@(s) strcmp(s.label, scenario_filter), scenarios);
    scenarios = scenarios(keep);
    if isempty(scenarios)
        error('--scenario "%s" não encontrado. Opções: %s', scenario_filter, ...
              strjoin({define_scenarios_tol().label}, ', '));
    end
end
n_sc       = numel(scenarios);
total_runs = numel(K_values) * numel(capacity_values) * n_sc * numel(policies);

fprintf('\n═══════════════════════════════════════════════════════════════\n');
fprintf('  EXPERIMENT_MW_OTIMO_TOL  (takeoff/landing + phase-aware risk)\n');
fprintf('  %s\n', datestr(now)); %#ok<TNOW1,DATST>
fprintf('═══════════════════════════════════════════════════════════════\n\n');
fprintf('altProfile  : takeoff_landing   cruiseAlt=%dm   takeoffDist=%dm\n', 90, 60);
fprintf('Risk ramp   : w_vid=0.80  w_unc=0.10  w_map=0.10\n');
fprintf('Planned     : %dK × %dC × %d cenários × %d políticas = %d runs\n', ...
        numel(K_values), numel(capacity_values), n_sc, numel(policies), total_runs);
fprintf('K values    : %s\n', num2str(K_values));
fprintf('Capacidades : %s\n', num2str(capacity_values));
fprintf('Políticas   : %s\n', strjoin(policies, ' | '));
fprintf('T_steady    : %d s\n', T_STEADY);
fprintf('Saída       : %s\n', OUT_ROOT);

%% ── Base config ──────────────────────────────────────────────────────
cfg_base = build_scenario_variavel();
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
cfg_base.csvDir     = OUT_ROOT;
cfg_base.headless   = true;
% ── Altitude profile ──
cfg_base.altProfile  = 'takeoff_landing';
cfg_base.cruiseAlt   = 90;
cfg_base.takeoffDist = 60;
% ── Phase-aware risk weights ──
cfg_base.w_tol_vid = 0.80;
cfg_base.w_tol_unc = 0.10;
cfg_base.w_tol_map = 0.10;

if ~auto_confirm && ~smoke_only
    fprintf('\nPress ENTER to continue, or Ctrl-C to abort.\n');
    pause;
end

%% ── Smoke test ───────────────────────────────────────────────────────
if ~skip_smoke || smoke_only
    fprintf('\n──────────────────────────────────────────────────────────\n');
    fprintf('  SMOKE TEST (C=2, K=1, T=30s)\n');
    fprintf('──────────────────────────────────────────────────────────\n\n');
    smoke_t0 = tic;
    smoke_ok = run_smoke_tol(OUT_ROOT, cfg_base, policies);
    fprintf('[SMOKE] Completed in %.1f s, status: %s\n', ...
            toc(smoke_t0), iif_tol(smoke_ok, 'PASS', 'FAIL'));
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
combos   = build_combos_tol(K_values, capacity_values, scenarios, policies);
n_combos = numel(combos);

nTarget  = 6;
cluster  = parcluster('local');
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
fprintf('  TOL SWEEP  (%d runs, %d workers)\n', n_combos, nWorkers);
fprintf('──────────────────────────────────────────────────────────\n\n');

sw_t0 = tic;
all_rows_cell = cell(n_combos, 1);

parfor idx = 1:n_combos
    c        = combos(idx);
    outDir_r = fullfile(OUT_ROOT, sprintf('K%d', c.K), ...
                        sprintf('C%02d', c.C), c.sc.label);
    system(sprintf('mkdir -p "%s"', outDir_r));

    cfg = build_run_cfg_tol(cfg_base, c.C, c.K, T_STEADY, c.sc, outDir_r);
    cfg.schedulingPolicy = c.policy;

    fprintf('[K=%2d C=%2d %-12s] %-22s  iniciando...\n', ...
            c.K, c.C, c.sc.label, c.policy);
    t_run = tic;
    try
        results = run_sim(cfg);
        dt = toc(t_run);
        fprintf('[K=%2d C=%2d %-12s] %-22s  %.0f s OK\n', ...
                c.K, c.C, c.sc.label, c.policy, dt);
        all_rows_cell{idx} = build_row_tol(results, c.C, c.K, c.sc, c.policy);
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

fprintf('\n[TOL] Sweep concluído em %.2f h\n', toc(sw_t0)/3600);

%% ── Summary CSVs ─────────────────────────────────────────────────────
for ki = 1:numel(K_values)
    for ci = 1:numel(capacity_values)
        for si = 1:n_sc
            K  = K_values(ki); C = capacity_values(ci); sc = scenarios(si);
            mask = cellfun(@(r) isfield(r,'K') && r.K==K && ...
                                isfield(r,'capacity') && r.capacity==C && ...
                                isfield(r,'scenario') && strcmp(r.scenario, sc.label), ...
                           all_rows_cell);
            write_csv_tol(all_rows_cell(mask), ...
                fullfile(OUT_ROOT, sprintf('K%d',K), sprintf('C%02d',C), ...
                         sc.label, 'summary.csv'));
        end
    end
end

valid   = ~cellfun(@isempty, all_rows_cell);
all_csv = fullfile(OUT_ROOT, 'sweep_summary_all.csv');
write_csv_tol(all_rows_cell(valid), all_csv);

fprintf('\n═══════════════════════════════════════════════════════════════\n');
fprintf('  EXPERIMENT_MW_OTIMO_TOL COMPLETED\n');
fprintf('  Finished: %s\n', datestr(now)); %#ok<TNOW1,DATST>
fprintf('  %d runs total.  Summary: %s\n', total_runs, all_csv);
fprintf('  Para gerar plots:\n');
fprintf('    python plot_mw_otimo.py      --dir csv_export_mw_otimo_tol\n');
fprintf('    python plot_mw_otimo_risk.py --dir csv_export_mw_otimo_tol\n');
fprintf('═══════════════════════════════════════════════════════════════\n\n');
diary off;
end


%% ═══════════════════════════════════════════════════════════════════
%%  Cenários (idênticos ao experiment_mw_otimo_a23)
%% ═══════════════════════════════════════════════════════════════════
function scenarios = define_scenarios_tol()
rA.numDrones = 1; rA.start = [1000, 1000]; rA.goal = [1000, 9000]; rA.label = 'RouteA';
rB.numDrones = 1; rB.start = [-500, 12000]; rB.goal = [-600, 14000]; rB.label = 'RouteB';
rBhalf.numDrones = 1; rBhalf.start = [-350, 9750]; rBhalf.goal = [-700, 14000]; rBhalf.label = 'RouteBhalf';

s1.label = '2r_static'; s1.routing_mode = 'static';
s1.routes = [rA, rB]; s1.route_frac = [1/2, 1/2];

s2.label = '2r_a23'; s2.routing_mode = 'static';
s2.routes = [rA, rBhalf]; s2.route_frac = [2/3, 1/3];

scenarios = [s1, s2];
end


%% ═══════════════════════════════════════════════════════════════════
%%  Build combos
%% ═══════════════════════════════════════════════════════════════════
function combos = build_combos_tol(K_values, capacity_values, scenarios, policies)
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
function cfg = build_run_cfg_tol(cfg_base, C, K, T_steady, sc, outDir)
cfg    = cfg_base;
nR     = numel(sc.routes);
routes = sc.routes;

if isfield(sc, 'route_frac') && ~isempty(sc.route_frac)
    frac = sc.route_frac(:)' / sum(sc.route_frac);
else
    frac = ones(1, nR) / nR;
end
C_per_route = max(1, round(frac * C));
C_per_route(end) = max(1, C - sum(C_per_route(1:end-1)));

ft_per_route = arrayfun( ...
    @(r) norm(r.goal(:)' - r.start(:)') / cfg.speedVal, routes);

for ri = 1:nR
    C_r     = C_per_route(ri);
    spacing = ft_per_route(ri) / max(C_r, 1);
    routes(ri).numDrones = C_r + floor(T_steady / spacing);
end

cfg.routes           = routes;
cfg.corridorLength   = norm(routes(1).goal(:)' - routes(1).start(:)');
cfg.flightTime       = ft_per_route(1);
cfg.corridorCapacity = C;
cfg.steady_duration  = T_steady;
cfg.dronesPerSlot    = K;
cfg.csvDir           = outDir;
cfg.routing.mode     = sc.routing_mode;

if max(ft_per_route) / min(ft_per_route) > 1.5
    cfg.arrivalModel = 'capacity_drain_routes';
end

last_entries = arrayfun(@(ri) ...
    cfg.minStartDelay + (routes(ri).numDrones - 1) * ...
    ft_per_route(ri) / max(C_per_route(ri), 1), 1:nR);
cfg.maxStartDelay = max(last_entries) + 10;

cfg.H_max  = [];
cfg.omega  = [];
cfg.beta_s = [];
cfg.beta_v = [];
end


%% ═══════════════════════════════════════════════════════════════════
%%  Build summary row
%% ═══════════════════════════════════════════════════════════════════
function row = build_row_tol(results, C, K, sc, pol)
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

xs_final = arrayfun(@(u) iif_end(results.xsVal{u}, 0), 1:N);
xv_final = arrayfun(@(u) iif_end(results.xvVal{u}, 0), 1:N);

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
row.alt_profile      = cfg.altProfile;
row.cruise_alt       = cfg.cruiseAlt;
row.error            = '';
end


%% ═══════════════════════════════════════════════════════════════════
%%  Write CSV
%% ═══════════════════════════════════════════════════════════════════
function write_csv_tol(rows, filepath)
if isempty(rows), return; end
[d, ~, ~] = fileparts(filepath);
if ~exist(d, 'dir'), mkdir(d); end
fid = fopen(filepath, 'w');
if fid == -1, error('Cannot open %s', filepath); end
fprintf(fid, ['capacity,K,scenario,routing_mode,num_routes,num_drones_total,policy,' ...
              'mean_h1,mean_h2,mean_r_sys,peak_r_sys,' ...
              'Jhat_emp,J_LB_doc,rho_emp,LB_legacy,' ...
              'qbar_s_mean,qbar_v_mean,q_emp_s_mean,q_emp_v_mean,' ...
              'xs_final_mean,xv_final_mean,n_s,n_v,alt_profile,cruise_alt\n']);
for ri = 1:numel(rows)
    r = rows{ri};
    if ~isfield(r, 'mean_h1') || isnan(r.mean_h1), continue; end
    alt_p = ''; cr_a = NaN;
    if isfield(r, 'alt_profile'), alt_p = r.alt_profile; end
    if isfield(r, 'cruise_alt'),  cr_a  = r.cruise_alt;  end
    fprintf(fid, ['%d,%d,%s,%s,%d,%d,%s,' ...
                  '%.4f,%.4f,%.6e,%.6e,' ...
                  '%.6e,%.6e,%.4f,%.6e,' ...
                  '%.4f,%.4f,%.4f,%.4f,' ...
                  '%.2f,%.2f,%.6e,%.6e,%s,%.1f\n'], ...
        r.capacity, r.K, r.scenario, r.routing_mode, r.num_routes, ...
        r.num_drones_total, r.policy, ...
        r.mean_h1, r.mean_h2, r.mean_r_sys, r.peak_r_sys, ...
        r.Jhat_emp, r.J_LB_doc, r.rho_emp, r.LB_legacy, ...
        r.qbar_s_mean, r.qbar_v_mean, r.q_emp_s_mean, r.q_emp_v_mean, ...
        r.xs_final_mean, r.xv_final_mean, r.n_s, r.n_v, alt_p, cr_a);
end
fclose(fid);
fprintf('[TOL] Summary → %s\n', filepath);
end


%% ═══════════════════════════════════════════════════════════════════
%%  Smoke test
%% ═══════════════════════════════════════════════════════════════════
function ok = run_smoke_tol(outRoot, cfg_base, policies)
ok     = true;
n_fail = 0;
SCALE  = 0.1;
C_smoke = 2; K_smoke = 1; T_smoke = 30;
scs  = define_scenarios_tol();

cfg_s                   = cfg_base;
cfg_s.microBSPos(:,1:2) = cfg_base.microBSPos(:,1:2) * SCALE;
cfg_s.manualHotspots    = [];
cfg_s.numHotspots       = 0;
cfg_s.csvExport         = false;

for si = 1:numel(scs)
    sc       = scs(si);
    sc_smoke = sc;
    for ri = 1:numel(sc.routes)
        sc_smoke.routes(ri).start = sc.routes(ri).start * SCALE;
        sc_smoke.routes(ri).goal  = sc.routes(ri).goal  * SCALE;
    end
    for pi = 1:numel(policies)
        pol   = policies{pi};
        cfg_r = build_run_cfg_tol(cfg_s, C_smoke, K_smoke, T_smoke, ...
                                  sc_smoke, fullfile(outRoot, 'smoke'));
        cfg_r.schedulingPolicy = pol;
        try
            results = run_sim(cfg_r);
            h1_mean = mean(results.h1MeanV(~isnan(results.h1MeanV)));
            if isnan(h1_mean) || h1_mean <= 0
                fprintf('[SMOKE] %-14s %-22s  FAIL: h1=%.3f\n', sc.label, pol, h1_mean);
                n_fail = n_fail + 1;
            else
                fprintf('[SMOKE] %-14s %-22s  h1=%.1f  h2=%.1f  OK\n', ...
                        sc.label, pol, h1_mean, ...
                        mean(results.h2MeanV(~isnan(results.h2MeanV))));
            end
        catch ME
            fprintf('[SMOKE] %-14s %-22s  EXCEPTION: %s\n', sc.label, pol, ME.message);
            n_fail = n_fail + 1;
        end
    end
end
ok = (n_fail == 0);
fprintf('[SMOKE] %s\n', iif_tol(ok, 'Todos os checks passaram.', ...
        sprintf('%d check(s) falharam.', n_fail)));
end


%% ═══════════════════════════════════════════════════════════════════
%%  Helpers
%% ═══════════════════════════════════════════════════════════════════
function v = iif_tol(cond, a, b)
if cond, v = a; else, v = b; end
end

function v = iif_end(x, default)
if isempty(x), v = default; else, v = x(end); end
end
