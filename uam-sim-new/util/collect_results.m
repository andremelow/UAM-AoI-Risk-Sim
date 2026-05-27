function results = collect_results(state, infra, cfg)
% COLLECT_RESULTS  Packages simulation state into a clean results struct.
%
%  Doc-aligned additions (Phase 1-4):
%    - Jhat empirical (time-average of linear surrogate)
%    - J_LB closed-form (Cauchy-Schwarz, Sec. 5)
%    - Optimality ratio rho = Jhat / J_LB
%    - Debt-queue trajectories x_s, x_v
%    - Coherence-window AoI z_u trajectory

N = cfg.numDrones;

% --- Timing ---
results.startTimes   = infra.startTimes;
results.endTimes     = infra.endTimes;
results.startSlots   = infra.startSlots;
results.endSlots     = infra.endSlots;
results.arrivalModel = infra.arrivalMeta.model;

% --- System-level time series ---
results.rSysTime = state.rSysTime;
results.rSysData = state.rSysData;
results.JhatTime = state.JhatTime;     % aggregate linear surrogate per slot
results.JhatData = state.JhatData;

results.uncMeanT = state.uncMeanT;  results.uncMeanV = state.uncMeanV;
results.mapMeanT = state.mapMeanT;  results.mapMeanV = state.mapMeanV;
results.vidMeanT = state.vidMeanT;  results.vidMeanV = state.vidMeanV;
results.h1MeanT  = state.h1MeanT;   results.h1MeanV  = state.h1MeanV;
results.h2MeanT  = state.h2MeanT;   results.h2MeanV  = state.h2MeanV;

% --- Per-drone time series ---
results.riskUnc  = state.riskUnc;
results.riskMap  = state.riskMap;
results.riskVid  = state.riskVid;
results.riskTot  = state.riskTot;
results.riskHat  = state.riskHat;
results.riskTime = state.riskTime;

results.h1Time   = state.h1Time;   results.h1Val = state.h1Val;
results.h2Time   = state.h2Time;   results.h2Val = state.h2Val;
results.zuTime   = state.zuTime;   results.zuVal = state.zuVal;
results.xsVal    = state.xsVal;    results.xvVal = state.xvVal;

% --- Counters ---
results.txSuccess    = state.txSuccess;
results.txFail       = state.txFail;
results.slotsC2      = state.slotsC2;
results.slotsC2Fail  = state.slotsC2Fail;
results.slotsVid     = state.slotsVid;
results.slotsVidFail = state.slotsVidFail;

results.nDelivered    = arrayfun(@(u) state.dual(u).n_delivered,   1:N);
results.frameCounter  = arrayfun(@(u) state.dual(u).frame_counter, 1:N);

% --- SNR / scheduling series ---
results.snrSeries   = state.snrSeries;
results.snrSlots    = state.snrSlots;
results.schedSeries = state.schedSeries;
results.schedSlots  = state.schedSlots;
results.txSuccSeries = state.txSuccSeries;
results.txSuccSlots  = state.txSuccSlots;

% --- Empirical objectives & optimality ratio ---
%  Time-averages computed only over the steady-state portion (drop first 20%)
T = numel(results.JhatData);
warm = max(1, round(0.2 * T));
if T > warm
    results.Jhat_emp = mean(results.JhatData(warm:end));
    results.Rsys_emp = mean(results.rSysData(warm:end));
else
    results.Jhat_emp = NaN;
    results.Rsys_emp = NaN;
end

% Lower bound (Sec. 5 of doc) and ratio
[J_LB, lb_info] = compute_lower_bound_doc(cfg);
results.J_LB     = J_LB;
results.lb_info  = lb_info;
results.rho_emp  = results.Jhat_emp / max(J_LB, 1e-12);

% --- Empirical throughputs (for debt-queue feasibility check) ---
T_active = max(1, max(infra.endSlots) - min(infra.startSlots));
results.q_emp_s = state.slotsC2  / T_active;
results.q_emp_v = arrayfun(@(u) state.dual(u).n_delivered, 1:N).' / T_active;

% --- Scenario ---
results.numDrones = N;
results.policy    = cfg.schedulingPolicy;
results.cfg       = cfg;

% --- Console summary ---
fprintf('\n============= Simulation summary =============\n');
fprintf('  Policy:           %s\n', cfg.schedulingPolicy);
fprintf('  N=%d, K=%d, L=%d, p_c2=%.2f, p_vid=%.2f\n', ...
        N, cfg.dronesPerSlot, cfg.L_vid, cfg.p_c2, cfg.p_vid);
fprintf('  n_s=%.4e  n_v=%.4e  H_nom=%.2f  H_max=%.2f\n', ...
        cfg.n_s, cfg.n_v, cfg.H_nom_slots, cfg.H_max);
fprintf('  Jhat_emp = %.4e   J_LB = %.4e   rho = %.3f\n', ...
        results.Jhat_emp, results.J_LB, results.rho_emp);
fprintf('  Rsys_emp = %.4e\n', results.Rsys_emp);
fprintf('  qbar_s mean = %.4f  q_emp_s mean = %.4f\n', mean(cfg.qbar_s), mean(results.q_emp_s));
fprintf('  qbar_v mean = %.4f  q_emp_v mean = %.4f\n', mean(cfg.qbar_v), mean(results.q_emp_v));
fprintf('  x_s final mean = %.2f   x_v final mean = %.2f\n', ...
        mean(arrayfun(@(u) state.dual(u).x_s, 1:N)), ...
        mean(arrayfun(@(u) state.dual(u).x_v, 1:N)));
fprintf('===============================================\n\n');
end
