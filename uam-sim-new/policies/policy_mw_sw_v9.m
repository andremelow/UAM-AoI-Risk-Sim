function txSlots = policy_mw_sw_v9(eligible, K_free, state, ~, cfg)
% POLICY_MW_SW_V9  V8 drift Max-Weight with the INSTANTANEOUS RISK GRADIENT.
%
%  Completes the 2x2 factorial {alpha const, alpha grad} x {z_u, h2}:
%
%                       video = z_u (window)   video = h2 (completed frame)
%    alpha constant     mw-sw-otimo-ch         mw-sw-v8
%    alpha gradient     mr-mw-ch-sw (MR)       mw-sw-v9  <-- this policy
%
%  Same structure as policy_mw_sw_v8 (LB -> V=alpha/q^LB -> W=V.p.h, largest K),
%  but the surrogate coefficients alpha are replaced per slot by the gradient of
%  the TRUE risk at the current state (same signal as policy_mr_mw_ch_sw):
%
%    alpha_{u,s}(t) = omega_u.( w^eff_unc.C_Nav_1 + w^eff_map.(2.R_gnd(t).v/r_u) )
%    alpha_{u,v}(t) = omega_u.  w^eff_vid / H_nom
%
%  SCA recompute (option A): the lower bound is re-solved every slot with the
%  current alpha(t) over the ELIGIBLE set —
%    Theta(t)   = sum_u [ sqrt(a_s(t)/p_s) + sqrt(a_v(t).L/p_v) ]
%    q_s^LB(t)  = min( K.sqrt(a_s(t).p_s)   / Theta(t),  p_s   )
%    q_v^LB(t)  = min( K.sqrt(a_v(t).p_v/L) / Theta(t),  p_v/L )
%    V(t)       = a(t)/q^LB(t)
%  Theta(t) is common to all drones, so ABSENT clipping the ranking equals the
%  MR ranking sqrt(a(t).p).h — the structural differences vs MR are exactly:
%  (i) video urgency h2 (completed-frame AoI) instead of z_u, and (ii) the box
%  clipping, under which V = a/p becomes LINEAR in alpha (amplifies hotspots).
%
%  Weights use the channel-conditioned p (SNR gate, like all -ch policies);
%  the LB keeps the nominal p_c2/p_vid. With alpha(t) state-dependent the
%  psi<=4 guarantee does NOT apply (successive-convex-approximation heuristic).

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

% ── Pass 2: V(t) = alpha(t)/q^LB(t), weights with SNR-gated p ─────────
sources = zeros(1, 2*n);
weights = zeros(1, 2*n);
for idx = 1:n
    u = eligible(idx);

    q_s = min(K * sqrt(a_s(idx) * p_s)     / Theta, p_s);
    q_v = min(K * sqrt(a_v(idx) * p_v / L) / Theta, p_v / L);
    V_s = a_s(idx) / q_s;
    V_v = a_v(idx) / q_v;

    h_s = state.dual(u).h1;      % status AoI h_{u,s}
    h_v = state.dual(u).h2;      % AoI of latest COMPLETED video frame h_{u,v}

    snr_u  = state.snrLast(u);
    snr_ok = isnan(snr_u) || (snr_u >= thr);
    ps_eff = p_s * snr_ok;
    pv_eff = p_v * snr_ok;

    W_s = V_s * ps_eff * h_s;
    W_v = V_v * pv_eff / L * h_v;

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
