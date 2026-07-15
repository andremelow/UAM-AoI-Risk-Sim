function txSlots = policy_mw_z_v9(eligible, K_free, state, ~, cfg)
% POLICY_MW_Z_V9  MW-Z over V9: envelope virtual queue + instantaneous
%  risk-gradient alphas.
%
%  Same two-pass structure as policy_mw_sw_v9 (gradient alpha(t), SCA
%  recompute of Theta/q^LB every slot), but the weights blend the drift
%  term with the envelope queue Z(u):
%
%    W_{u,s} = (V*V_{u,s}(t) + Z(u)*alpha_{u,s}(t)) * p_{u,s}(t)   * h_{u,s}
%    W_{u,v} = (V*V_{u,v}(t) + Z(u)*alpha_{u,v}(t)) * p_{u,v}(t)/L * h_{u,v}
%
%  Z is updated in step_transmit with the SAME gradient alphas (mode is
%  selected by cfg.schedulingPolicy == 'mw-z-v9'):
%    Z(u) <- max(0, Z(u) + alpha_s(t)*h1 + alpha_v(t)*h2 - b_u)
%  so Z accumulates exactly where the AoI-attributable risk rate exceeds
%  the budget — over the crowd Rhat ~ 1-2 (queue engages), elsewhere
%  ~0.2 (queue drains): a position-aware envelope debt.
%
%  With Z == 0 and cfg.V = 1 this reduces EXACTLY to policy_mw_sw_v9
%  (sanity-checked in test_sanity_doc).

p_s = cfg.p_c2;
p_v = cfg.p_vid;
L   = cfg.L_vid;
K   = cfg.dronesPerSlot;
thr = cfg.thresholdSNR;
v   = cfg.v_max / cfg.updateRate;
Hn  = cfg.H_nom_slots;
Rgnd0 = cfg.rho_0 * pi * cfg.r_min^2;   % warmup fallback (AoI=0 ground floor)

n = numel(eligible);

% ── Pass 1: instantaneous gradient alpha(t) per eligible drone ────────
a_s = zeros(1, n);
a_v = zeros(1, n);
for idx = 1:n
    u  = eligible(idx);
    om = cfg.omega(min(u, numel(cfg.omega)));
    r_u = state.aoiRadius(u);
    if r_u <= 0, r_u = cfg.r_min; end
    if ~isempty(state.riskMap{u}), R_gnd = state.riskMap{u}(end); else, R_gnd = Rgnd0; end
    if state.dual(u).in_ramp
        we_u = cfg.w_tol_unc; we_m = cfg.w_tol_map; we_v = cfg.w_tol_vid;
    else
        we_u = cfg.w_unc;     we_m = cfg.w_map;     we_v = cfg.w_vid;
    end
    a_s(idx) = om * (we_u * cfg.C_Nav_1 + we_m * (2 * R_gnd * v / r_u));
    a_v(idx) = om * we_v / Hn;
end

% ── SCA lower bound at alpha(t): Theta over the eligible set ──────────
Theta = sum(sqrt(a_s / p_s)) + sum(sqrt(a_v * L / p_v));
if Theta <= 0
    txSlots = [];
    return;
end

% ── Pass 2: V(t) = alpha(t)/q^LB(t) + envelope term Z*alpha(t) ────────
sources = zeros(1, 2*n);
weights = zeros(1, 2*n);
for idx = 1:n
    u = eligible(idx);

    q_s = min(K * sqrt(a_s(idx) * p_s)     / Theta, p_s);
    q_v = min(K * sqrt(a_v(idx) * p_v / L) / Theta, p_v / L);
    V_s = a_s(idx) / q_s;
    V_v = a_v(idx) / q_v;

    h_s = state.dual(u).h1;
    h_v = state.dual(u).h2;
    Z_u = state.Z_env(u);

    snr_u  = state.snrLast(u);
    snr_ok = isnan(snr_u) || (snr_u >= thr);
    ps_eff = p_s * snr_ok;
    pv_eff = p_v * snr_ok;

    W_s = (cfg.V * V_s + Z_u * a_s(idx)) * ps_eff * h_s;
    W_v = (cfg.V * V_v + Z_u * a_v(idx)) * pv_eff / L * h_v;

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
