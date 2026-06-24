function txSlots = policy_max_risk_nsw(eligible, K_free, state, ~, cfg)
% POLICY_MAX_RISK_NSW  Risk-weighted Max-Weight scheduling (non-switching).
%
%  Non-switching counterpart of POLICY_MAX_RISK_SW: the per-slot scoring is
%  identical, but the policy operates under the no-switching constraint —
%  once a video frame starts it occupies the channel for L consecutive slots
%  (force-continued by schedule_sources, which reserves the channel and
%  reduces K_free).  Here `eligible` already excludes drones with an in-flight
%  video frame, and the policy only fills the remaining K_free channels.
%
%  Scores every source by the product of instantaneous composite risk and
%  normalised queue urgency:
%
%    W_{u,s}(t) = R_u(t) * h_{u,s}(t)          (risk x C2 AoI)
%    W_{u,v}(t) = R_u(t) * z_u(t) / L           (risk x video AoI / frame length)
%
%  where R_u = w_unc*R_unc + w_map*R_map + w_vid*R_vid.  Drones with R_u = 0
%  receive a small epsilon so they are not permanently starved before the
%  risk map arrives.

L   = cfg.L_vid;
n   = numel(eligible);
sources = zeros(1, 2*n);
weights = zeros(1, 2*n);

for idx = 1:n
    u   = eligible(idx);
    R_u = risk_composite(state, u, cfg);

    h_s = state.dual(u).h1;      % C2 AoI (slots since last successful C2 tx)
    z_u = state.dual(u).z_u;     % video AoI proxy (slots since last vid delivery)

    W_s = (R_u + 1e-9) * h_s;
    W_v = (R_u + 1e-9) * z_u / L;

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


function R = risk_composite(state, u, ~)
% Reads the pre-computed phase-aware composite risk from step_risk.
if ~isempty(state.riskTot{u})
    R = state.riskTot{u}(end);
else
    R = 0;
end
end
