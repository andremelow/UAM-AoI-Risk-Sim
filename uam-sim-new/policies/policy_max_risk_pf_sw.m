function txSlots = policy_max_risk_pf_sw(eligible, K_free, state, ~, cfg)
% POLICY_MAX_RISK_PF_SW  Risk-weighted Proportional-Fair scheduling (switching).
%
%  Fuses the risk-aware urgency of max-risk with the anti-starvation
%  fairness of Proportional-Fair: the PF numerator is replaced by the
%  max-risk weight (risk x normalised AoI), and each source is normalised
%  by its own EMA throughput T_avg_s so no single high-risk drone can
%  monopolise the channel:
%
%    score_{u,s}(t) = R_u(t) * h_{u,s}(t)        / T_avg_{u,s}(t)     (C2)
%    score_{u,v}(t) = R_u(t) * z_u(t) / L        / T_avg_{u,v}(t)     (video)
%
%  where R_u = w_unc*R_unc + w_map*R_map + w_vid*R_vid (phase-aware, read
%  from state.riskTot), h_{u,s} is the C2 AoI (slots), z_u/L the video AoI
%  in frame units (symmetric with h_{u,s}=1), and T_avg the per-source EMA
%  throughput maintained in step_transmit (same estimator PF-classic uses).
%
%  Compared with:
%    - max-risk-sw:  identical numerator, but NO fairness division -> a
%                    persistently high-risk drone can starve the rest.
%    - pf-classic-sw: identical fairness division, but numerator is the raw
%                    achievable rate (risk-blind).
%  max-risk-pf-sw keeps the risk ranking while spreading service over time.
%
%  Switching mode: all K_free channels are renegotiated every slot; partial
%  video frames are preempted and resumed (vid_remaining preserved).
%  At most one source (C2 or video) per drone per slot (one radio).

L = cfg.L_vid;
n = numel(eligible);
sources = zeros(1, 2*n);
weights = zeros(1, 2*n);

for idx = 1:n
    u   = eligible(idx);
    R_u = risk_composite(state, u);

    h_s = state.dual(u).h1;      % C2 AoI  (slots since last successful C2 tx)
    z_u = state.dual(u).z_u;     % video AoI proxy (slots since last vid delivery)

    T_c2  = max(state.avgThroughputC2(u),  1e-9);
    T_vid = max(state.avgThroughputVid(u), 1e-9);

    W_s = (R_u + 1e-9) * h_s        / T_c2;
    W_v = (R_u + 1e-9) * (z_u / L)  / T_vid;

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


function R = risk_composite(state, u)
% Pre-computed phase-aware composite risk from step_risk (state.riskTot).
if ~isempty(state.riskTot{u})
    R = state.riskTot{u}(end);
else
    R = 0;
end
end
