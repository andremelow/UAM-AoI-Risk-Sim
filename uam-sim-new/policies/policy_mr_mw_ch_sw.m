function txSlots = policy_mr_mw_ch_sw(eligible, K_free, state, ~, cfg)
% POLICY_MR_MW_CH_SW  Gradient-risk, channel-aware Max-Weight (switching).
%
%  Hybrid of the heuristic Max-Risk (MR) signal and the optimal Max-Weight
%  (MW) structure, with the channel gate (CH), switching (SW):
%
%   - MW: keeps the guarantee-form √(α·p)·h weight (not MR's endogenous R_u·h).
%   - MR: replaces the constant surrogate slopes n_s,n_v by the INSTANTANEOUS
%         marginal risk (gradient ∂R_u/∂AoI) at the current state, so the
%         weight is convex-aware (grows with r_u), position-aware (ρ via R_gnd)
%         and phase-aware (w_tol in takeoff/landing ramps).
%   - CH: p replaced by the channel-conditioned p_{u,k}(t)=p_k·1{SNR>=thr}.
%
%     α_{u,s}(t) = (v/r_u)·( w^eff_unc·R_nav + 2·w^eff_map·R_gnd )   [= ∂R_u/∂h_s]
%     α_{u,v}(t) = w^eff_vid / H_nom                                  [= ∂R_u/∂z_u]
%     W_{u,s}    = √( ω_u · α_{u,s} · p_{u,s}(t) ) · h_{u,s}
%     W_{u,v}    = √( ω_u · α_{u,v} · p_{u,v}(t)/L ) · z_u
%
%  with v = v_max/updateRate (radius growth per slot of AoI). Risk components
%  (R_nav, R_gnd) and in_ramp are read from the previous step_risk (1-slot lag,
%  like SNR); a warmup fallback uses the AoI=0 ground floor.
%
%  NOTE: ψ≤2 is proved for CONSTANT α; with state-dependent α(t) this is a
%  successive-convex-approximation heuristic (convex-aware), not the strict
%  bound — the price of matching the convex risk.

v   = cfg.v_max / cfg.updateRate;
L   = cfg.L_vid;
Hn  = cfg.H_nom_slots;
thr = cfg.thresholdSNR;
p_s = cfg.p_c2;
p_v = cfg.p_vid;
Rgnd0 = cfg.rho_0 * pi * cfg.r_min^2;   % warmup fallback (AoI=0 ground floor)

n       = numel(eligible);
sources = zeros(1, 2*n);
weights = zeros(1, 2*n);

for idx = 1:n
    u   = eligible(idx);
    om  = cfg.omega(u);
    h_s = state.dual(u).h1;
    z_u = state.dual(u).z_u;
    r_u = state.aoiRadius(u);
    if r_u <= 0, r_u = cfg.r_min; end

    % risk components from previous step_risk (warmup fallback)
    if ~isempty(state.riskMap{u}), R_gnd = state.riskMap{u}(end); else, R_gnd = Rgnd0; end
    if ~isempty(state.riskUnc{u}), R_nav = state.riskUnc{u}(end); else, R_nav = 0;     end

    % phase-aware weights (w_tol in ramp)
    if state.dual(u).in_ramp
        we_u = cfg.w_tol_unc; we_m = cfg.w_tol_map; we_v = cfg.w_tol_vid;
    else
        we_u = cfg.w_unc;     we_m = cfg.w_map;     we_v = cfg.w_vid;
    end

    % marginal risk of status AoI: keep the surrogate collision term C_Nav_1
    % (expected collision over the AoI cycle — the local R_nav gradient is ~0
    % when no overlap yet, which would starve C2 in safe areas), plus the
    % POSITION-dependent ground gradient 2·R_gnd·v/r_u (= 2·ρ·π·v·r_u, grows
    % with r_u → convex-aware, large in hotspots → position-aware).
    alpha_s = we_u * cfg.C_Nav_1 + we_m * (2 * R_gnd * v / r_u);
    alpha_v = we_v / Hn;

    % channel-conditioned success probability (0 in outage)
    snr_u  = state.snrLast(u);
    snr_ok = isnan(snr_u) || (snr_u >= thr);
    ps_eff = p_s * snr_ok;
    pv_eff = p_v * snr_ok;

    W_s = sqrt(om * alpha_s * ps_eff)       * h_s;
    W_v = sqrt(om * alpha_v * pv_eff / L)   * z_u;

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
