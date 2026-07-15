function txSlots = policy_mw3_nsw(eligible, K_free, state, ~, cfg)
% POLICY_MW3_NSW  Channel-conditioned drift Max-Weight (NON-switching).
%
%  Max-Weight v3: like policy_max_risk_v2, but the constant success
%  probability p is replaced by the per-drone, channel-CONDITIONED success
%  probability p_{u,k}(t). This restores the channel-awareness that the
%  Lyapunov-drift Max-Weight theory prescribes (weight = deliverability x
%  urgency) but that the constant-p implementation neutralised.
%
%  Delivery model (step_channel/step_transmit): a transmission succeeds iff
%  SNR_u(t) >= thresholdSNR AND Bernoulli(p). Hence the EXACT expected
%  success probability of scheduling drone u this slot is
%
%      p_{u,k}(t) = p_k * 1{ SNR_u(t) >= thresholdSNR },   k in {s,v}.
%
%  Plugging this into the drift index zeroes the weight of any drone in
%  outage (SNR below threshold) -> Max-Weight stops wasting slots on
%  undeliverable drones (the behaviour PF approximates), while still ranking
%  the deliverable ones by AoI/drift urgency + throughput debt:
%
%      W_{u,s}(t) = p_{u,s}(t) * ( V*omega_u*n_s*h_{u,s}  + x_{u,s} )
%      W_{u,v}(t) = p_{u,v}(t) * ( V*omega_u*n_v*z_u/L     + x_{u,v} )
%
%  SNR_u(t) is state.snrLast(u) (updated by step_channel in the previous
%  slot; NaN before the first measurement -> treated as usable, like
%  step_transmit's link_ok). Non-switching: video frames force-continue for L slots (schedule_sources);
%  eligible excludes in-flight frames, so W_v is a video-START weight.

L   = cfg.L_vid;
V   = getfield_default(cfg, 'V',     1.0);
n_s = getfield_default(cfg, 'n_s',   1.0);
% Density-coupled video coefficient: n_v = k * n_s (see policy_mw3_sn).
k_nv = getfield_default(cfg, 'n_v_ratio', 0.25);
n_v  = k_nv * n_s;
p_s = getfield_default(cfg, 'p_c2',  1.0);
p_v = getfield_default(cfg, 'p_vid', 1.0);
thr = getfield_default(cfg, 'thresholdSNR', -Inf);
eps_tie = 1e-9;

n       = numel(eligible);
sources = zeros(1, 2*n);
weights = zeros(1, 2*n);

for idx = 1:n
    u = eligible(idx);

    if isfield(state, 'omega') && numel(state.omega) >= u
        omega_u = state.omega(u);
    else
        omega_u = 1.0;
    end

    % --- channel-conditioned success probability (deliverability) ---
    snr_u  = state.snrLast(u);
    snr_ok = isnan(snr_u) || (snr_u >= thr);   % NaN (unmeasured) -> give a chance
    p_s_eff = p_s * snr_ok;
    p_v_eff = p_v * snr_ok;

    h_s = state.dual(u).h1;
    z_u = state.dual(u).z_u;
    x_s = dual_field(state, u, 'x_s', 0.0);
    x_v = dual_field(state, u, 'x_v', 0.0);

    % Everything scaled by p_eff -> outage drone (snr_ok=0) gets weight 0.
    W_s = p_s_eff * ( V * omega_u * n_s * h_s       + x_s + eps_tie * h_s );
    W_v = p_v_eff * ( V * omega_u * n_v * (z_u / L) + x_v + eps_tie * (z_u / L) );

    sources(2*idx-1) = 2*(u-1) + 1;  weights(2*idx-1) = W_s;
    sources(2*idx)   = 2*(u-1) + 2;  weights(2*idx)   = W_v;
end

candidates = find(weights > 0);
if isempty(candidates)
    txSlots = [];
    return;
end

[~, order]  = sort(weights(candidates), 'descend');
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

% ========================================================================
function val = getfield_default(s, name, default)
if isfield(s, name) && ~isempty(s.(name))
    val = s.(name);
else
    val = default;
end
end

function val = dual_field(state, u, name, default)
if isfield(state.dual(u), name) && ~isempty(state.dual(u).(name))
    val = state.dual(u).(name);
else
    val = default;
end
end
