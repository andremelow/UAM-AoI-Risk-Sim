function txSlots = policy_risk_aware(activeDrones, K, state, ~, cfg)
% POLICY_RISK_AWARE  Risk-weighted AoI on 2N sources.
%
%  Operates directly on the 2N sources (C2 and video for each drone).
%  Score for each source = R_u(t) × AoI_source(u,t)
%
%    score_C2(u)  = R_u * h1(u)   [risk × C2 staleness]
%    score_Vid(u) = R_u * h2(u)   [risk × video staleness]
%
%  where R_u = w_unc*R_unc + w_map*R_map + w_vid*R_vid (weighted composite risk).
%
%  Selects top-K sources by score.
%  Sources from high-risk drones with stale information are prioritised.
%  The small offsets (+1e-9, +1) prevent collapse to zero when R_u=0 or AoI=0.

n = numel(activeDrones);
sources = zeros(1, 2*n);
scores  = zeros(1, 2*n);

for idx = 1:n
    u   = activeDrones(idx);
    R_u = weighted_risk(state, u, cfg);
    % C2 source
    sources(2*idx-1) = 2*(u-1) + 1;
    scores(2*idx-1)  = (R_u + 1e-9) * (state.dual(u).h1 + 1);
    % Video source
    sources(2*idx)   = 2*(u-1) + 2;
    scores(2*idx)    = (R_u + 1e-9) * (state.dual(u).h2 + 1);
end

[~, order] = sort(scores, 'descend');
txSlots = sources(order(1:min(K, 2*n)));
end

function R = weighted_risk(state, u, cfg)
if ~isempty(state.riskUnc{u})
    R = cfg.w_unc * state.riskUnc{u}(end) + ...
        cfg.w_map * state.riskMap{u}(end) + ...
        cfg.w_vid * state.riskVid{u}(end);
else
    R = 0;
end
end
