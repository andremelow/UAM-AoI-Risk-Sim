function txSlots = policy_mw_v9_nsw(eligible, K_free, state, ~, cfg)
% POLICY_MW_V9_NSW  Gradient-risk channel-aware Max-Weight (NON-switching).
%
%  policy_mw_otimo_ch with the RISK GRADIENT injected: the constant surrogate
%  slopes n_s, n_v are replaced by the instantaneous marginal risk at the
%  current state (same alpha(t) as policy_mr_mw_ch_sw / policy_mw_sw_v9):
%
%    alpha_{u,s}(t) = w^eff_unc.C_Nav_1 + w^eff_map.(2.R_gnd(t).v/r_u)  [= dR/dh_s]
%    alpha_{u,v}(t) = w^eff_vid / H_nom                                  [= dR/dz_u]
%
%    W_{u,s}(t) = sqrt(omega_u . alpha_{u,s}(t) . p_{u,s}(t))     . h_{u,s}(t)
%    W_{u,v}(t) = sqrt(omega_u . alpha_{u,v}(t) . p_{u,v}(t)/L)   . z_u(t)
%
%  i.e. the NON-SWITCHING counterpart of the Risk-Aware heuristic: position-,
%  convexity- and phase-aware weights, channel gate p.1{SNR>=thr}, but video
%  frames force-continue for L slots once started (schedule_sources reserves
%  the channel; no preemption). Complementary experiment RC10-nsw evaluates
%  whether the gradient survives the loss of preemption flexibility.
%
%  Same SCA caveat as MR/V9: with state-dependent alpha(t) the constant-alpha
%  optimality guarantee does not apply.

p_s = cfg.p_c2;
p_v = cfg.p_vid;
L   = cfg.L_vid;
thr = cfg.thresholdSNR;
v   = cfg.v_max / cfg.updateRate;
Hn  = cfg.H_nom_slots;
Rgnd0 = cfg.rho_0 * pi * cfg.r_min^2;   % warmup fallback (AoI=0 ground floor)

n       = numel(eligible);
sources = zeros(1, 2*n);
weights = zeros(1, 2*n);

for idx = 1:n
    u   = eligible(idx);
    om  = cfg.omega(min(u, numel(cfg.omega)));
    h_s = state.dual(u).h1;
    z_u = state.dual(u).z_u;
    r_u = state.aoiRadius(u);
    if r_u <= 0, r_u = cfg.r_min; end

    % risk components from previous step_risk (warmup fallback)
    if ~isempty(state.riskMap{u}), R_gnd = state.riskMap{u}(end); else, R_gnd = Rgnd0; end

    % phase-aware weights (w_tol in ramp)
    if state.dual(u).in_ramp
        we_u = cfg.w_tol_unc; we_m = cfg.w_tol_map; we_v = cfg.w_tol_vid;
    else
        we_u = cfg.w_unc;     we_m = cfg.w_map;     we_v = cfg.w_vid;
    end

    alpha_s = we_u * cfg.C_Nav_1 + we_m * (2 * R_gnd * v / r_u);
    alpha_v = we_v / Hn;

    % channel-conditioned success probability (0 in outage)
    snr_u  = state.snrLast(u);
    snr_ok = isnan(snr_u) || (snr_u >= thr);
    ps_eff = p_s * snr_ok;
    pv_eff = p_v * snr_ok;

    W_s = sqrt(om * alpha_s * ps_eff)     * h_s;
    W_v = sqrt(om * alpha_v * pv_eff / L) * z_u;

    sources(2*idx-1) = 2*(u-1) + 1;  weights(2*idx-1) = W_s;
    sources(2*idx)   = 2*(u-1) + 2;  weights(2*idx)   = W_v;
end

candidates = find(weights > 0);
if isempty(candidates)
    txSlots = [];
    return;
end

[~, order] = sort(weights(candidates), 'descend');
txSlots     = [];
used_drones = [];
for oi = 1:numel(order)
    if numel(txSlots) >= K_free, break; end
    src = sources(candidates(order(oi)));
    u   = ceil(src / 2);
    if ~ismember(u, used_drones)
        txSlots(end+1)     = src; %#ok<AGROW>
        used_drones(end+1) = u;   %#ok<AGROW>
    end
end
end
