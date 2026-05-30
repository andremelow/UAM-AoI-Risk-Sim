function txSlots = policy_pf_classic(activeDrones, K, state, ~)
% POLICY_PF_CLASSIC  Classic Proportional Fair on 2N sources.
%
%  Operates directly on the 2N sources (C2 and video for each drone),
%  each with an independent per-source EMA throughput T_avg.
%
%  Score_s(t) = r_u(t) / T_avg_s(t)
%
%  where:
%    r_u(t)    = rawThroughput(u)  — instantaneous Shannon capacity [Mbps]
%                (same for C2 and Video of the same drone: same channel)
%    T_avg_s   = avgThroughputC2(u) or avgThroughputVid(u)
%                (per-source EMA, updated in step_transmit separately)
%
%  Selecting top-K sources by score allows PF to balance C2 and Video
%  slots independently, without a secondary h1/h2 heuristic.

n = numel(activeDrones);
% Build 2N source list and scores
sources = zeros(1, 2*n);
scores  = zeros(1, 2*n);
for idx = 1:n
    u = activeDrones(idx);
    r_u = max(state.rawThroughput(u), 1e-9);
    % C2 source
    src_c2 = 2*(u-1) + 1;
    sources(2*idx-1) = src_c2;
    scores(2*idx-1)  = r_u / max(state.avgThroughputC2(u),  1e-9);
    % Video source
    src_vid = 2*(u-1) + 2;
    sources(2*idx)   = src_vid;
    scores(2*idx)    = r_u / max(state.avgThroughputVid(u), 1e-9);
end

[~, order] = sort(scores, 'descend');
txSlots = sources(order(1:min(K, 2*n)));
end
