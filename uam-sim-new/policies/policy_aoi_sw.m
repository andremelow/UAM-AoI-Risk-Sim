function txSlots = policy_aoi_sw(eligible, K_free, state, ~, cfg)
% POLICY_AOI_SW  Switching AoI-Normalized scheduling (preemptive).
%
%  Score per source, normalised by service cost so C2 and video are
%  comparable in AoI reduction per slot invested:
%
%    score_C2 (u) = h1(u) / L_c2        L_c2 = 1  (1 slot per C2 packet)
%    score_Vid(u) = h2(u) / L_vid       L_vid = cfg.L_vid  (100 by default)
%
%  Runs in switching mode: eligible includes drones with vid_remaining > 0.
%  The scheduler picks the top-K_free sources every slot with the one-radio
%  constraint (at most one source per drone per slot).

L_c2  = cfg.L_c2;
L_vid = cfg.L_vid;

n       = numel(eligible);
sources = zeros(1, 2*n);
scores  = zeros(1, 2*n);

for idx = 1:n
    u = eligible(idx);
    sources(2*idx-1) = 2*(u-1) + 1;
    scores(2*idx-1)  = state.dual(u).h1 / L_c2;
    sources(2*idx)   = 2*(u-1) + 2;
    scores(2*idx)    = state.dual(u).h2 / L_vid;
end

% Pick top-K_free with at most one source per drone (one radio per drone).
candidates = find(scores > 0);
if isempty(candidates)
    txSlots = [];
    return;
end

[~, order] = sort(scores(candidates), 'descend');
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
