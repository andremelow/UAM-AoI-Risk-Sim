function sweep_results = run_sweep(sweep_cfg, outDir)
% RUN_SWEEP  Runs multiple scenarios and exports consolidated CSVs.
%
%  sweep_cfg is a struct with:
%    .policies     — cell array, e.g.
%                    {'round-robin','round-robin-aware','aoi-pure',
%                     'risk-aware','max-weight'}
%    .param_name   — string: field in cfg to vary, e.g. 'numDrones'
%    .param_values — vector of values, e.g. [1 5 10 20]
%    .base_cfg     — base scenario config (from build_scenario_config())
%    .T_slots      — simulation duration in slots (default: 200000)
%
%  Outputs per-policy summary CSV with both legacy (Kadota) and
%  doc-aligned (Sec. 5/6) lower bounds and the empirical optimality
%  ratio rho = Jhat_emp / J_LB_doc.
%
%  Example — sweep N (scalability):
%    sc.policies     = {'round-robin','round-robin-aware','aoi-pure',
%                        'risk-aware','max-weight'};
%    sc.param_name   = 'numDrones';
%    sc.param_values = [1 2 5 10 20];
%    sc.base_cfg     = build_scenario_config();
%    sc.T_slots      = 200000;
%    run_sweep(sc, 'results/scalability');
%
%  Example — sweep p (channel reliability):
%    sc.param_name   = 'p_c2';
%    sc.param_values = [0.5 0.6 0.7 0.8 0.9 0.95];

setup_paths();
if nargin < 2, outDir = 'csv_export/sweep'; end
if ~exist(outDir,'dir'), mkdir(outDir); end

policies     = sweep_cfg.policies;
param_name   = sweep_cfg.param_name;
param_values = sweep_cfg.param_values;
base_cfg     = sweep_cfg.base_cfg;
T_slots      = 200000;
if isfield(sweep_cfg,'T_slots'), T_slots = sweep_cfg.T_slots; end

% flightTime: if corridorLength+speedVal are set, derive from physics.
% Only override if T_slots is explicitly given AND no physical corridor.
if isfield(sweep_cfg,'T_slots') && (~isfield(base_cfg,'corridorLength') || base_cfg.corridorLength==0)
    base_cfg.flightTime = T_slots / base_cfg.updateRate;
elseif isfield(base_cfg,'corridorLength') && isfield(base_cfg,'speedVal') && base_cfg.speedVal>0
    base_cfg.flightTime = base_cfg.corridorLength / base_cfg.speedVal;
end
base_cfg.csvExport = true;
base_cfg.csvDir    = outDir;

nPolicies = numel(policies);
nValues   = numel(param_values);
totalRuns = nPolicies * nValues;

fprintf('[SWEEP] %s × %d values × %d policies = %d runs\n', ...
    param_name, nValues, nPolicies, totalRuns);
fprintf('[SWEEP] Output: %s\n\n', outDir);

sweep_results = struct();
run_idx = 0;

for vi = 1:nValues
    val = param_values(vi);
    for pi = 1:nPolicies
        pol = policies{pi};
        run_idx = run_idx + 1;

        cfg = base_cfg;
        cfg.(param_name)     = val;
        cfg.schedulingPolicy = pol;

        % numDrones change requires resetting omega so validate_uam_config
        % can re-default it to the right size.
        if strcmp(param_name, 'numDrones')
            cfg.omega = [];
        end

        % Re-derive dependent fields (n_s, n_v, q_LB, qbar) for this run
        cfg = validate_uam_config(cfg);

        fprintf('[SWEEP] Run %d/%d: %s=%g, policy=%s\n', ...
            run_idx, totalRuns, param_name, val, pol);

        try
            results = run_sim(cfg);

            % --- Legacy Kadota lower bound (per-source AoI), kept for
            %     backwards comparison with prior experiments.
            N = cfg.numDrones;
            p_emp = zeros(2*N, 1);
            for u = 1:N
                tot_c2  = results.slotsC2(u)  + results.slotsC2Fail(u);
                tot_vid = results.slotsVid(u) + results.slotsVidFail(u);
                p_emp(2*(u-1)+1) = results.slotsC2(u)  / max(tot_c2,  1);
                p_emp(2*(u-1)+2) = results.slotsVid(u) / max(tot_vid, 1);
            end
            LB_legacy = compute_lower_bound(cfg, p_emp);

            % --- Doc-aligned LB (already inside results from collect_results)
            LB_doc      = results.J_LB;
            Jhat_emp    = results.Jhat_emp;
            Rsys_emp    = results.Rsys_emp;
            rho_emp     = results.rho_emp;

            % --- Empirical throughputs vs targets (debt feasibility check) ---
            qbar_s_mean = mean(cfg.qbar_s);
            qbar_v_mean = mean(cfg.qbar_v);
            q_emp_s_mean = mean(results.q_emp_s);
            q_emp_v_mean = mean(results.q_emp_v);

            % --- Final debt-queue magnitudes (stability proxy) ---
            xs_final = arrayfun(@(u) ifempty(results.xsVal{u}, 0), 1:N);
            xv_final = arrayfun(@(u) ifempty(results.xvVal{u}, 0), 1:N);

            % Store
            sweep_results(run_idx).param_name    = param_name;
            sweep_results(run_idx).param_value   = val;
            sweep_results(run_idx).policy        = pol;
            sweep_results(run_idx).num_drones    = cfg.numDrones;
            sweep_results(run_idx).T_slots       = T_slots;
            sweep_results(run_idx).mean_h1       = mean(results.h1MeanV);
            sweep_results(run_idx).mean_h2       = mean(results.h2MeanV);
            sweep_results(run_idx).mean_r_sys    = Rsys_emp;
            sweep_results(run_idx).peak_r_sys    = max(results.rSysData);
            sweep_results(run_idx).Jhat_emp      = Jhat_emp;
            sweep_results(run_idx).J_LB_doc      = LB_doc;
            sweep_results(run_idx).rho_emp       = rho_emp;
            sweep_results(run_idx).LB_legacy     = LB_legacy;
            sweep_results(run_idx).qbar_s_mean   = qbar_s_mean;
            sweep_results(run_idx).qbar_v_mean   = qbar_v_mean;
            sweep_results(run_idx).q_emp_s_mean  = q_emp_s_mean;
            sweep_results(run_idx).q_emp_v_mean  = q_emp_v_mean;
            sweep_results(run_idx).xs_final_mean = mean(xs_final);
            sweep_results(run_idx).xv_final_mean = mean(xv_final);
            sweep_results(run_idx).n_s           = cfg.n_s;
            sweep_results(run_idx).n_v           = cfg.n_v;

        catch ME
            fprintf('[SWEEP] WARNING: run failed — %s\n', ME.message);
            fprintf('[SWEEP] Stack: %s:%d\n', ME.stack(1).name, ME.stack(1).line);
            sweep_results(run_idx).param_name  = param_name;
            sweep_results(run_idx).param_value = val;
            sweep_results(run_idx).policy      = pol;
            sweep_results(run_idx).error       = ME.message;
        end
    end
end

% --- Write consolidated summary CSV ---
summaryFile = fullfile(outDir, sprintf('sweep_%s_summary.csv', param_name));
fid = fopen(summaryFile, 'w');
fprintf(fid, ['param_name,param_value,policy,num_drones,T_slots,' ...
              'mean_h1,mean_h2,mean_r_sys,peak_r_sys,' ...
              'Jhat_emp,J_LB_doc,rho_emp,LB_legacy,' ...
              'qbar_s_mean,qbar_v_mean,q_emp_s_mean,q_emp_v_mean,' ...
              'xs_final_mean,xv_final_mean,n_s,n_v\n']);
for r = 1:run_idx
    s = sweep_results(r);
    if isfield(s, 'error') && ~isempty(s.error), continue; end
    fprintf(fid, ['%s,%g,%s,%d,%d,' ...
                  '%.4f,%.4f,%.6e,%.6e,' ...
                  '%.6e,%.6e,%.4f,%.6e,' ...
                  '%.4f,%.4f,%.4f,%.4f,' ...
                  '%.2f,%.2f,%.6e,%.6e\n'], ...
        s.param_name, s.param_value, s.policy, s.num_drones, s.T_slots, ...
        s.mean_h1, s.mean_h2, s.mean_r_sys, s.peak_r_sys, ...
        s.Jhat_emp, s.J_LB_doc, s.rho_emp, s.LB_legacy, ...
        s.qbar_s_mean, s.qbar_v_mean, s.q_emp_s_mean, s.q_emp_v_mean, ...
        s.xs_final_mean, s.xv_final_mean, s.n_s, s.n_v);
end
fclose(fid);

fprintf('\n[SWEEP] Consolidated summary → %s\n', summaryFile);
fprintf('[SWEEP] %d runs completed.\n', run_idx);

% --- Console summary table (rho per policy per param value) ---
fprintf('\n========== rho_emp matrix ==========\n');
fprintf('%-10s', param_name);
for pi = 1:nPolicies, fprintf(' | %-18s', policies{pi}); end
fprintf('\n%s\n', repmat('-', 1, 12 + 21*nPolicies));
for vi = 1:nValues
    fprintf('%-10g', param_values(vi));
    for pi = 1:nPolicies
        idx = (vi-1)*nPolicies + pi;
        if idx <= numel(sweep_results) && isfield(sweep_results(idx), 'rho_emp') ...
                && ~isempty(sweep_results(idx).rho_emp) && ~isnan(sweep_results(idx).rho_emp)
            fprintf(' | %-18.4f', sweep_results(idx).rho_emp);
        else
            fprintf(' | %-18s', 'FAIL');
        end
    end
    fprintf('\n');
end
fprintf('====================================\n\n');
end


function v = ifempty(x, default)
% Helper: return last entry of x, or default if empty.
if isempty(x), v = default; else, v = x(end); end
end
