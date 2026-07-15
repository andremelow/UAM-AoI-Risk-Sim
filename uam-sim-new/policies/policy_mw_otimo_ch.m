function txSlots = policy_mw_otimo_ch(eligible, K_free, state, ~, cfg)
% POLICY_MW_OTIMO_CH  Channel-aware optimal Max-Weight (NON-switching).
%
%  Same √(α)-form optimal Max-Weight as policy_mw_sw_otimo (the 2-optimal
%  policy of the lower-bound theorem), but with the constant success
%  probability p replaced by the channel-CONDITIONED success probability
%  p_{u,k}(t) = p_k * 1{ SNR_u(t) >= thresholdSNR }.
%
%  Delivery succeeds iff SNR >= threshold AND Bernoulli(p), so the expected
%  success of scheduling drone u this slot is exactly p_k·1{SNR>=thr}. With
%  p inside the √, a drone in outage gets p_eff=0 -> weight = √0 = 0 -> it is
%  never scheduled. This restores the deliverability×urgency structure the
%  Max-Weight theory prescribes, so the policy stops wasting slots on
%  undeliverable drones (the channel-awareness PF has empirically), while
%  keeping the optimal √-coefficient ranking among deliverable drones:
%
%      W_{u,s}(t) = √(ω_u·n_s·p_{u,s}(t))·h_{u,s}(t)
%      W_{u,v}(t) = √(ω_u·n_v·p_{u,v}(t)/L)·z_u(t)
%
%  SNR_u(t) = state.snrLast(u) (from step_channel, previous slot; NaN before
%  the first measurement -> treated as usable, like step_transmit's link_ok).
%  Non-switching: video frames force-continue for L slots (schedule_sources);

p_s = cfg.p_c2;
p_v = cfg.p_vid;
n_s = cfg.n_s;
n_v = cfg.n_v;
L   = cfg.L_vid;
thr = cfg.thresholdSNR;

n       = numel(eligible);
sources = zeros(1, 2*n);
weights = zeros(1, 2*n);

for idx = 1:n
    u   = eligible(idx);
    om  = cfg.omega(u);
    h_s = state.dual(u).h1;
    h_v = state.dual(u).z_u;

    % channel-conditioned success probability (0 in outage)
    snr_u  = state.snrLast(u);
    snr_ok = isnan(snr_u) || (snr_u >= thr);
    ps_eff = p_s * snr_ok;
    pv_eff = p_v * snr_ok;

    W_s = sqrt(om * n_s * ps_eff) * h_s;
    W_v = sqrt(om * n_v * pv_eff / L) * h_v;

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
