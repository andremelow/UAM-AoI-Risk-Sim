function txSlots = policy_best_cqi(eligible, K, state, ~, ~)
% POLICY_BEST_CQI  Best CQI / Max-SNR / opportunistic scheduling.
%
%  Score(u) = C_u(t) = B log2(1 + SNR_u(t))  [Shannon capacity, Mbps]
%           = state.rawThroughput(u)
%
%  Maximises instantaneous channel utilisation. Ignores AoI, risk, and
%  queue state — throughput-optimal but AoI-agnostic baseline.
%  C2 source gets a tiny tie-break preference over video for the same drone.

n = numel(eligible);
sources = zeros(1, 2*n);
scores  = zeros(1, 2*n);
for idx = 1:n
    u   = eligible(idx);
    C_u = max(state.rawThroughput(u), 0);
    sources(2*idx-1) = 2*(u-1) + 1;   % C2
    sources(2*idx)   = 2*(u-1) + 2;   % video
    scores(2*idx-1)  = C_u + 1e-12;   % C2 preferred on ties
    scores(2*idx)    = C_u;
end
[~, order] = sort(scores, 'descend');
txSlots = sources(order(1:min(K, 2*n)));
end
