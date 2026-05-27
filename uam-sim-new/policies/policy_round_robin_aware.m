function [txSlots, state] = policy_round_robin_aware(eligible, K_free, state, cfg)
% POLICY_ROUND_ROBIN_AWARE  Round-robin baseline that skips drones in
% deep coverage gaps (low SNR) and falls back to classic RR if all
% eligible drones are in a dead zone.
%
%  Designed as a STRONGER baseline against the doc-aligned Max-Weight
%  policy: classic RR is SNR-blind and burns L_vid slots on a doomed
%  video transmission whenever the in-flight UAV has no link. RR-aware
%  retains the simplicity of round-robin (no AoI/risk weighting, no debt
%  queues) but at least respects channel feasibility.
%
%  Skip criterion (per slot):
%     state.snrLast(u) <= cfg.thresholdSNR  OR  isnan(state.snrLast(u))
%
%  When skipped, BOTH the C2 and Video sources of that UAV are
%  excluded for this slot. This matches the physical reality that a
%  UAV with no link cannot deliver either telemetry or video.
%
%  Fallback: if every eligible UAV is in the dead zone, the policy
%  degrades to classic RR (better to attempt than waste the slot).
%
%  Returns updated state because rrPointer advances internally.
%  txSlots: vector of source IDs in {1..2N}.

N_total = 2 * cfg.numDrones;

% Determine which UAVs are link-feasible right now.
linkOK = false(cfg.numDrones, 1);
for u = eligible
    s = state.snrLast(u);
    linkOK(u) = ~isnan(s) && (s > cfg.thresholdSNR);
end

% If nobody is link-feasible, fall back to classic RR over all eligible.
if ~any(linkOK(eligible))
    eligible_aware = eligible;
else
    eligible_aware = eligible(linkOK(eligible));
end

% Same global-source RR walk as classic, restricted to eligible_aware.
ptr = mod(state.rrPointer - 1, N_total) + 1;
txSlots = [];
chosen = 0;
scanned = 0;
while chosen < K_free && scanned < N_total
    src = ptr;
    u_src = ceil(src/2);
    isVid = (mod(src, 2) == 0);
    if ismember(u_src, eligible_aware)
        txSlots(end+1) = src; %#ok<AGROW>
        if isVid
            state.dual(u_src).vid_in_flight = true;
        end
        chosen = chosen + 1;
    end
    ptr = mod(ptr, N_total) + 1;
    scanned = scanned + 1;
end
state.rrPointer = ptr;
end
