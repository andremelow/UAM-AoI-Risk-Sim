function txSlots = policy_pf_classic_sw(eligible, K, state, ~)
% POLICY_PF_CLASSIC_SW  Switching Proportional Fair (preemptive).
%
%  Same PF scoring as pf-classic (r_u / T_avg per source), but runs in
%  switching mode: eligible contains ALL active drones, including those
%  with vid_remaining > 0. The scheduler picks the top-K sources by score
%  every slot with no prior reservations.
%
%  Preemption occurs naturally when a drone's C2 source has a higher PF
%  score than its video source — the scheduler will allocate the slot to
%  C2 even if a video frame is partially in progress. The partial frame is
%  preserved (vid_remaining unchanged) and resumed when the video source
%  wins a future slot.
%
%  Score per source s of drone u:
%    score_s(t) = r_u(t) / T_avg_s(t)
%
%  where T_avg_s is the per-source EMA throughput (updated in step_transmit).

n = numel(eligible);
sources = zeros(1, 2*n);
scores  = zeros(1, 2*n);

for idx = 1:n
    u   = eligible(idx);
    r_u = max(state.rawThroughput(u), 1e-9);

    src_c2 = 2*(u-1) + 1;
    sources(2*idx-1) = src_c2;
    scores(2*idx-1)  = r_u / max(state.avgThroughputC2(u),  1e-9);

    src_vid = 2*(u-1) + 2;
    sources(2*idx)   = src_vid;
    scores(2*idx)    = r_u / max(state.avgThroughputVid(u), 1e-9);
end

% Pick top-K sources with at most one source per drone (one radio per drone).
[~, order] = sort(scores, 'descend');
txSlots      = [];
used_drones  = [];
for oi = 1:numel(order)
    if numel(txSlots) >= K, break; end
    src = sources(order(oi));
    u   = ceil(src / 2);
    if ~ismember(u, used_drones)
        txSlots(end+1)     = src; %#ok<AGROW>
        used_drones(end+1) = u;   %#ok<AGROW>
    end
end
end
