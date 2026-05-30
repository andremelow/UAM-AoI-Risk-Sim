function sweep_results = run_routing_sweep(sweep_cfg, outDir)
% RUN_ROUTING_SWEEP  Routing-aware analog of run_sweep.
%
%  Runs all (scenario × policy) combinations and exports CSVs.
%  Mirrors run_sweep.m but varies structural routing scenarios instead of a
%  scalar parameter.
%
%  sweep_cfg fields:
%    .policies  — cell array, same as run_sweep
%    .scenarios — struct array, each with fields:
%                   .label         string label
%                   .routing_mode  'static' | 'risk_astar'
%                   .routes        struct array [numDrones, start, goal, label]
%    .base_cfg  — base scenario config (geometry, radio, dual-source params)
%    .T_slots   — optional: simulation duration in slots (default: physics)
%
%  Output CSV columns mirror run_sweep (param_name → scenario, param_value → scenario_idx).
%
%  Usage:
%    sc.policies  = {'round-robin','max-weight', ...};
%    sc.scenarios = define_routing_scenarios();
%    sc.base_cfg  = build_scenario_config();
%    run_routing_sweep(sc, 'csv_export/routing');

setup_paths();
if nargin < 2, outDir = 'csv_export/routing_sweep'; end
if ~exist(outDir,'dir'), mkdir(outDir); end

policies  = sweep_cfg.policies;
scenarios = sweep_cfg.scenarios;
base_cfg  = sweep_cfg.base_cfg;

% Headless: caller controls via sweep_cfg.headless or base_cfg.headless.
% Default is false (dashboard visible) to match run_sweep behaviour.
if isfield(sweep_cfg, 'headless')
    base_cfg.headless = sweep_cfg.headless;
elseif ~isfield(base_cfg, 'headless')
    base_cfg.headless = false;
end
base_cfg.csvExport = true;
base_cfg.csvDir    = outDir;

nPolicies  = numel(policies);
nScenarios = numel(scenarios);
totalRuns  = nPolicies * nScenarios;

fprintf('[ROUTING-SWEEP] %d scenarios × %d policies = %d runs\n', ...
        nScenarios, nPolicies, totalRuns);
fprintf('[ROUTING-SWEEP] Output: %s\n\n', outDir);

sweep_results = struct();
run_idx = 0;

for si = 1:nScenarios
    sc = scenarios(si);
    for pi = 1:nPolicies
        pol = policies{pi};
        run_idx = run_idx + 1;

        cfg = apply_routing_scenario(base_cfg, sc);
        cfg.schedulingPolicy = pol;
        cfg = validate_uam_config(cfg);   % derive numDrones, n_s, n_v, qbar — mirrors run_sweep

        fprintf('[ROUTING-SWEEP] Run %d/%d: scenario=%-22s  policy=%s\n', ...
                run_idx, totalRuns, sc.label, pol);

        try
            results = run_sim(cfg);
            N = cfg.numDrones;

            % Legacy Kadota LB (same as run_sweep)
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

            sweep_results(run_idx).scenario_idx    = si;
            sweep_results(run_idx).scenario        = sc.label;
            sweep_results(run_idx).routing_mode    = sc.routing_mode;
            sweep_results(run_idx).num_routes      = numel(sc.routes);
            sweep_results(run_idx).policy          = pol;
            sweep_results(run_idx).num_drones      = N;
            sweep_results(run_idx).mean_h1         = mean(results.h1MeanV);
            sweep_results(run_idx).mean_h2         = mean(results.h2MeanV);
            sweep_results(run_idx).mean_r_sys      = results.Rsys_emp;
            sweep_results(run_idx).peak_r_sys      = max(results.rSysData);
            sweep_results(run_idx).Jhat_emp        = results.Jhat_emp;
            sweep_results(run_idx).J_LB_doc        = results.J_LB;
            sweep_results(run_idx).rho_emp         = results.rho_emp;
            sweep_results(run_idx).LB_legacy       = LB_legacy;
            sweep_results(run_idx).qbar_s_mean     = mean(cfg.qbar_s);
            sweep_results(run_idx).qbar_v_mean     = mean(cfg.qbar_v);
            sweep_results(run_idx).q_emp_s_mean    = mean(results.q_emp_s);
            sweep_results(run_idx).q_emp_v_mean    = mean(results.q_emp_v);
            sweep_results(run_idx).xs_final_mean   = mean(xs_final);
            sweep_results(run_idx).xv_final_mean   = mean(xv_final);
            sweep_results(run_idx).n_s             = cfg.n_s;
            sweep_results(run_idx).n_v             = cfg.n_v;
            sweep_results(run_idx).error           = '';

        catch ME
            fprintf('[ROUTING-SWEEP] WARNING: run failed — %s\n', ME.message);
            if ~isempty(ME.stack)
                fprintf('[ROUTING-SWEEP] Stack: %s:%d\n', ME.stack(1).name, ME.stack(1).line);
            end
            sweep_results(run_idx).scenario_idx = si;
            sweep_results(run_idx).scenario      = sc.label;
            sweep_results(run_idx).routing_mode  = sc.routing_mode;
            sweep_results(run_idx).num_routes    = numel(sc.routes);
            sweep_results(run_idx).policy        = pol;
            sweep_results(run_idx).error         = ME.message;
        end
    end
end

% ── Summary CSV ──────────────────────────────────────────────────────
summaryFile = fullfile(outDir, 'sweep_routing_summary.csv');
fid = fopen(summaryFile, 'w');
fprintf(fid, ['scenario,routing_mode,num_routes,num_drones,policy,' ...
              'mean_h1,mean_h2,mean_r_sys,peak_r_sys,' ...
              'Jhat_emp,J_LB_doc,rho_emp,LB_legacy,' ...
              'qbar_s_mean,qbar_v_mean,q_emp_s_mean,q_emp_v_mean,' ...
              'xs_final_mean,xv_final_mean,n_s,n_v\n']);
for r = 1:run_idx
    s = sweep_results(r);
    if isfield(s,'error') && ~isempty(s.error), continue; end
    fprintf(fid, ['%s,%s,%d,%d,%s,' ...
                  '%.4f,%.4f,%.6e,%.6e,' ...
                  '%.6e,%.6e,%.4f,%.6e,' ...
                  '%.4f,%.4f,%.4f,%.4f,' ...
                  '%.2f,%.2f,%.6e,%.6e\n'], ...
        s.scenario, s.routing_mode, s.num_routes, s.num_drones, s.policy, ...
        s.mean_h1, s.mean_h2, s.mean_r_sys, s.peak_r_sys, ...
        s.Jhat_emp, s.J_LB_doc, s.rho_emp, s.LB_legacy, ...
        s.qbar_s_mean, s.qbar_v_mean, s.q_emp_s_mean, s.q_emp_v_mean, ...
        s.xs_final_mean, s.xv_final_mean, s.n_s, s.n_v);
end
fclose(fid);
fprintf('\n[ROUTING-SWEEP] Summary → %s\n', summaryFile);

% ── rho_emp matrix (scenarios × policies) ────────────────────────────
fprintf('\n========== rho_emp matrix ==========\n');
fprintf('%-24s', 'scenario');
for pi = 1:nPolicies, fprintf(' | %-18s', policies{pi}); end
fprintf('\n%s\n', repmat('-', 1, 26 + 21*nPolicies));
for si = 1:nScenarios
    fprintf('%-24s', scenarios(si).label);
    for pi = 1:nPolicies
        idx = (si-1)*nPolicies + pi;
        if idx <= numel(sweep_results) && isfield(sweep_results(idx),'rho_emp') ...
                && ~isempty(sweep_results(idx).rho_emp) && ~isnan(sweep_results(idx).rho_emp)
            fprintf(' | %-18.4f', sweep_results(idx).rho_emp);
        else
            fprintf(' | %-18s', 'FAIL');
        end
    end
    fprintf('\n');
end
fprintf('====================================\n\n');
fprintf('[ROUTING-SWEEP] %d runs completed.\n', run_idx);
end


function cfg = apply_routing_scenario(base_cfg, sc)
cfg                = base_cfg;
cfg.routes         = sc.routes;
cfg.corridorLength = norm(cfg.routes(1).goal - cfg.routes(1).start);
cfg.flightTime     = cfg.corridorLength / cfg.speedVal;
cfg.routing.mode   = sc.routing_mode;
cfg.omega          = [];   % reset so validate_uam_config re-defaults to numDrones
end


function v = ifempty(x, default)
if isempty(x), v = default; else, v = x(end); end
end
