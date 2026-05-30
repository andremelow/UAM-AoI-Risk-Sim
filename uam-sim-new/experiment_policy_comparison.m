function results_all = experiment_policy_comparison(varargin)
% EXPERIMENT_POLICY_COMPARISON  Fixed-scenario comparison of all six policies.
%
%  Runs the exact scenario from build_scenario_config() (Interlagos, 10 drones,
%  2 routes, Risk-A* routing) once per policy and reports doc-aligned metrics:
%
%    Jhat_emp   time-average linear risk surrogate (steady-state, drop first 20%)
%    J_LB       closed-form Cauchy-Schwarz lower bound (Sec. 5)
%    rho        empirical optimality ratio = Jhat_emp / J_LB
%    Rsys_emp   mean weighted true risk
%    h1_mean    mean status AoI (slots, steady-state)
%    h2_mean    mean video/window AoI (slots, steady-state)
%    xs_final   mean status debt x_s at end of run (stability proxy)
%    xv_final   mean video  debt x_v at end of run (stability proxy)
%    delivered  mean frames delivered per drone
%
%  Policies:
%    aoi-pure, max-weight, max-weight-drift, pf-classic, risk-aware,
%    round-robin-aware
%
%  Usage:
%    experiment_policy_comparison()            % interactive, shows dashboard
%    experiment_policy_comparison('--headless') % batch / server
%    experiment_policy_comparison('--yes')      % skip confirmation prompt
%
%  Output:
%    csv_export/policy_comparison/policy_comparison_<timestamp>.csv
%    csv_export/policy_comparison/policy_comparison_<timestamp>.log

setup_paths();

% --- Argument parsing ---
headless     = false;
auto_confirm = false;
ai = 1;
while ai <= numel(varargin)
    switch varargin{ai}
        case '--headless', headless     = true;
        case '--yes',      auto_confirm = true;
    end
    ai = ai + 1;
end

policies = { ...
    'aoi-pure', ...
    'max-weight', ...
    'max-weight-drift', ...
    'pf-classic', ...
    'risk-aware', ...
    'round-robin-aware' };

% --- Output directory (absolute, anchored to this script's directory) ---
SIM_ROOT = fileparts(mfilename('fullpath'));
OUT_DIR  = fullfile(SIM_ROOT, 'csv_export', 'policy_comparison');
if ~exist(OUT_DIR, 'dir'), mkdir(OUT_DIR); end

ts      = datestr(now, 'yyyymmdd_HHMMSS'); %#ok<TNOW1,DATST>
logFile = fullfile(OUT_DIR, sprintf('policy_comparison_%s.log', ts));
csvFile = fullfile(OUT_DIR, sprintf('policy_comparison_%s.csv', ts));

diary(logFile); diary on;

fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  EXPERIMENT_POLICY_COMPARISON\n');
fprintf('  Scenario: build_scenario_config() — Interlagos, 2 routes\n');
fprintf('  Policies: %s\n', strjoin(policies, ', '));
fprintf('  Started:  %s\n', datestr(now)); %#ok<TNOW1,DATST>
fprintf('  Log:      %s\n', logFile);
fprintf('═══════════════════════════════════════════════════════════════\n\n');

% --- Confirmation ---
if ~auto_confirm
    fprintf('  %d runs on the full Interlagos scenario.\n', numel(policies));
    fprintf('  Press ENTER to continue, or Ctrl-C to abort.\n\n');
    pause;
end

% --- Base config ---
cfg_base                  = build_scenario_config();
cfg_base.headless         = headless;
cfg_base.csvExport        = true;
cfg_base.csvDir           = OUT_DIR;
cfg_base.use_normalization = false;
cfg_base.V_s              = 1.0;
cfg_base.V_v              = 1.0;
cfg_base.V                = 1.0;
cfg_base.kappa            = 0.0;

% --- Run each policy ---
n_pol       = numel(policies);
results_all = cell(n_pol, 1);
t_run       = zeros(n_pol, 1);

for pi = 1:n_pol
    pol = policies{pi};
    fprintf('\n[%d/%d] Policy: %s\n', pi, n_pol, pol);
    fprintf('  %s\n', repmat('-', 1, 50));

    cfg                  = cfg_base;
    cfg.schedulingPolicy = pol;

    t0 = tic;
    try
        results_all{pi} = run_sim(cfg);
        results_all{pi}.policy = pol;
    catch ME
        fprintf('  ERROR: %s\n', ME.message);
        if ~isempty(ME.stack)
            fprintf('  Stack: %s:%d\n', ME.stack(1).name, ME.stack(1).line);
        end
        results_all{pi} = struct('policy', pol, 'error', ME.message, ...
            'Jhat_emp', NaN, 'J_LB', NaN, 'rho_emp', NaN, ...
            'Rsys_emp', NaN, 'xsVal', {{}}, 'xvVal', {{}}, ...
            'h1Val', {{}}, 'h2Val', {{}}, 'nDelivered', NaN);
    end
    t_run(pi) = toc(t0);
    fprintf('  Elapsed: %.1f s\n', t_run(pi));
end

% --- Summary table ---
fprintf('\n\n');
fprintf('═══════════════════════════════════════════════════════════════════════════════════════\n');
fprintf('  RESULTS SUMMARY\n');
fprintf('═══════════════════════════════════════════════════════════════════════════════════════\n');
hdr = sprintf('%-22s | %11s | %11s | %7s | %11s | %8s | %8s | %8s | %8s | %9s', ...
    'Policy', 'Jhat_emp', 'J_LB', 'rho', 'Rsys_emp', ...
    'h1_mean', 'h2_mean', 'xs_fin', 'xv_fin', 'delivered');
fprintf('%s\n', hdr);
fprintf('%s\n', repmat('-', 1, numel(hdr)));

for pi = 1:n_pol
    r   = results_all{pi};
    pol = policies{pi};

    if isfield(r, 'error') && ~isempty(r.error)
        fprintf('%-22s | ERROR: %s\n', pol, r.error);
        continue;
    end

    h1_ss   = ss_mean(r.h1Val);
    h2_ss   = ss_mean(r.h2Val);
    xs_fin  = tail_mean(r.xsVal);
    xv_fin  = tail_mean(r.xvVal);
    deliv   = mean(r.nDelivered);

    fprintf('%-22s | %11.4e | %11.4e | %7.3f | %11.4e | %8.2f | %8.2f | %8.2f | %8.2f | %9.2f\n', ...
        pol, r.Jhat_emp, r.J_LB, r.rho_emp, r.Rsys_emp, ...
        h1_ss, h2_ss, xs_fin, xv_fin, deliv);
end

fprintf('%s\n', repmat('═', 1, numel(hdr)));
fprintf('  Runtime: %s\n', ...
    strjoin(arrayfun(@(t) sprintf('%.0fs', t), t_run, 'UniformOutput', false), ', '));

% --- CSV export ---
write_csv(csvFile, policies, results_all);
fprintf('\n  CSV: %s\n', csvFile);
fprintf('  Log: %s\n\n', logFile);
fprintf('═══════════════════════════════════════════════════════════════════════════════════════\n\n');

diary off;
end


%% ── Helpers ─────────────────────────────────────────────────────────────────

function m = ss_mean(val_cell)
% Mean over drones of the steady-state portion of a per-drone time series.
vals = cellfun(@(v) ss_scalar(v), val_cell);
m    = mean(vals(~isnan(vals)));
end

function v = ss_scalar(vec)
% Mean of the last 80% of a vector (mirrors collect_results warm-up drop).
if isempty(vec), v = NaN; return; end
T    = numel(vec);
warm = max(1, round(0.2 * T));
v    = mean(vec(warm:end));
end

function m = tail_mean(val_cell)
% Mean of the last value in each drone's time series.
vals = cellfun(@(v) safe_last(v), val_cell);
m    = mean(vals(~isnan(vals)));
end

function v = safe_last(vec)
if isempty(vec), v = NaN; else, v = vec(end); end
end

function write_csv(csvFile, policies, results_all)
fid = fopen(csvFile, 'w');
fprintf(fid, 'policy,Jhat_emp,J_LB,rho_emp,Rsys_emp,h1_mean,h2_mean,xs_final,xv_final,delivered\n');
for pi = 1:numel(policies)
    r   = results_all{pi};
    pol = policies{pi};
    if isfield(r, 'error') && ~isempty(r.error)
        fprintf(fid, '%s,error,,,,,,,,%s\n', pol, strrep(r.error, ',', ';'));
        continue;
    end
    fprintf(fid, '%s,%.6e,%.6e,%.6f,%.6e,%.4f,%.4f,%.4f,%.4f,%.4f\n', ...
        pol, r.Jhat_emp, r.J_LB, r.rho_emp, r.Rsys_emp, ...
        ss_mean(r.h1Val), ss_mean(r.h2Val), ...
        tail_mean(r.xsVal), tail_mean(r.xvVal), ...
        mean(r.nDelivered));
end
fclose(fid);
end
