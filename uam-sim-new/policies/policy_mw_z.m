function txSlots = policy_mw_z(eligible, K_free, state, ~, cfg)
% POLICY_MW_Z  Drift-plus-penalty Max-Weight with envelope virtual queue Z.
%
%  Same structure as policy_mw_sw_v8 (LB -> V = alpha/q^LB -> top-K), but the
%  scheduling weights blend the AoI drift term with an envelope-violation
%  term driven by the per-UAV virtual queue Z(u) (updated in step_transmit):
%
%    W_{u,s} = (V*V_{u,s} + Z(u)*alpha_{u,s}) * p_{u,s}(t)   * h_{u,s}(t)
%    W_{u,v} = (V*V_{u,v} + Z(u)*alpha_{u,v}) * p_{u,v}(t)/L * h_{u,v}(t)
%
%    Z(u) <- max(0, Z(u) + alpha_s*h_s + alpha_v*h_v - b_u)   [end of slot]
%
%  With Z == 0 and cfg.V = 1 the weights reduce EXACTLY to policy_mw_sw_v8
%  (sanity-checked in test_sanity_doc). cfg.V and cfg.b_u are validated in
%  validate_uam_config (defaults 1 and 1).
%
%  Switching mode: all K channels renegotiated every slot, at most one
%  source per drone per slot (same selection mechanism as V8).

p_s = cfg.p_c2;
p_v = cfg.p_vid;
n_s = cfg.n_s;
n_v = cfg.n_v;
L   = cfg.L_vid;
K   = cfg.dronesPerSlot;
thr = cfg.thresholdSNR;

% ── Lower bound + Lyapunov coefficients (identical to policy_mw_sw_v8) ──
N_eff = cfg.corridorCapacity;
om_v  = cfg.omega(:);
om_v  = om_v(1:min(N_eff, numel(om_v)));
Theta = sum(sqrt(om_v * n_s / p_s)) + sum(sqrt(om_v * n_v * L / p_v));

n       = numel(eligible);
sources = zeros(1, 2*n);
weights = zeros(1, 2*n);

for idx = 1:n
    u   = eligible(idx);
    om  = cfg.omega(min(u, numel(cfg.omega)));
    a_s = om * n_s;                                  % alpha_{u,s}
    a_v = om * n_v;                                  % alpha_{u,v}

    q_s = min(K * sqrt(a_s * p_s)     / Theta, p_s);
    q_v = min(K * sqrt(a_v * p_v / L) / Theta, p_v / L);

    V_s = a_s / q_s;
    V_v = a_v / q_v;

    h_s = state.dual(u).h1;
    h_v = state.dual(u).h2;
    Z_u = state.Z_env(u);

    snr_u  = state.snrLast(u);
    snr_ok = isnan(snr_u) || (snr_u >= thr);
    ps_eff = p_s * snr_ok;
    pv_eff = p_v * snr_ok;

    W_s = (cfg.V * V_s + Z_u * a_s) * ps_eff * h_s;
    W_v = (cfg.V * V_v + Z_u * a_v) * pv_eff / L * h_v;

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
