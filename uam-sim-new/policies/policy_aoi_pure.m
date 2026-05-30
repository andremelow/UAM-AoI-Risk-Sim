function txSlots = policy_aoi_pure(activeDrones, K, state, ~, cfg)
% POLICY_AOI_PURE  Max-Weight AoI on 2N sources.
%
%  Operates directly on the 2N sources (C2 and video for each drone).
%  Score for each source = its current AoI in slots.
%
%    score_C2(u)  = h1(u)   [C2 AoI — staleness of telemetry/control]
%    score_Vid(u) = h2(u)   [Video AoI — staleness of last delivered frame]
%
%  Selects top-K sources by AoI score.
%  This is the Max-Weight AoI policy (Kadota & Modiano) generalised to
%  dual-source streams. When K=1 it reduces to scheduling the single
%  most stale source across all 2N streams.
%
%  No hard SNR filter is applied here.  A binary exclusion would starve
%  persistently bad-channel drones: their AoI would grow without bound
%  because the growing score never brings them back into an excluded pool.
%  AoI-pure is self-correcting: failed transmissions leave h high, which
%  raises the drone's score and guarantees eventual selection.

n = numel(activeDrones);
sources = zeros(1, 2*n);
scores  = zeros(1, 2*n);

for idx = 1:n
    u = activeDrones(idx);
    % C2 source
    sources(2*idx-1) = 2*(u-1) + 1;
    scores(2*idx-1)  = state.dual(u).h1;   % slots
    % Video source
    sources(2*idx)   = 2*(u-1) + 2;
    scores(2*idx)    = state.dual(u).h2;   % slots
end

[~, order] = sort(scores, 'descend');
txSlots = sources(order(1:min(K, 2*n)));
end
