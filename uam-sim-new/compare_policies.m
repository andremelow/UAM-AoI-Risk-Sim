function compare_policies()
% COMPARE_POLICIES  Side-by-side run of all policies, doc-aligned metrics.
%
%  Reports:
%    Jhat_emp / J_LB  -> empirical optimality ratio rho per policy
%    Rsys_emp         -> mean weighted true risk
%    final debt-queue magnitudes (stability proxy)

setup_paths();

policies = {'round-robin', 'round-robin-aware', 'aoi-pure', 'risk-aware', 'max-weight'};

base = build_scenario_config();
base.csvExport = false;

fprintf('\n%-22s | %12s | %12s | %8s | %10s | %10s\n', ...
        'Policy', 'Jhat_emp', 'J_LB', 'rho', 'mean x_s', 'mean x_v');
fprintf('%s\n', repmat('-', 1, 95));

results_all = struct();
for k = 1:numel(policies)
    cfg = base;
    cfg.schedulingPolicy = policies{k};
    res = run_sim(cfg);
    results_all.(strrep(policies{k}, '-', '_')) = res;
    fprintf('%-22s | %12.4e | %12.4e | %8.3f | %10.2f | %10.2f\n', ...
        policies{k}, res.Jhat_emp, res.J_LB, res.rho_emp, ...
        mean(cellfun(@(x) ifelse(isempty(x), 0, x(end)), res.xsVal)), ...
        mean(cellfun(@(x) ifelse(isempty(x), 0, x(end)), res.xvVal)));
end
end

function y = ifelse(c, a, b)
if c, y = a; else, y = b; end
end
