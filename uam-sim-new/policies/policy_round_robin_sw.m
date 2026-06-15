function [txSlots, state] = policy_round_robin_sw(eligible, K_free, state, cfg)
% POLICY_ROUND_ROBIN_SW  Switching round-robin (preemptive).
%
%  Identical walk to classic RR over the global 2N source list, but runs
%  in switching mode: eligible contains ALL active drones, including those
%  with vid_remaining > 0. No channel is pre-reserved; the scheduler makes
%  a fresh decision every slot.
%
%  Consequence: a drone mid-video may receive a C2 slot if its C2 source
%  comes next in the cycle, preempting the partial video frame. The frame
%  is not lost — vid_remaining is preserved — and will be resumed if/when
%  the video source is selected in a future slot.

% Each drone has one radio: at most one source (C2 or video) per slot.
% Track which drones already have a slot assigned this cycle.
N_total         = 2 * cfg.numDrones;
n_elig          = numel(eligible);
ptr             = mod(state.rrPointer - 1, N_total) + 1;
txSlots         = [];
chosen          = 0;
scanned         = 0;
scheduled_drones = [];

% Guard on `chosen < n_elig`: when K >= N_eff the loop would otherwise scan
% through all inactive drones and wrap ptr back to its start position,
% causing the same fixed set of C2 sources to be scheduled every slot and
% starving video completely.
while chosen < K_free && chosen < n_elig && scanned < N_total
    src   = ptr;
    u_src = ceil(src / 2);
    if ismember(u_src, eligible) && ~ismember(u_src, scheduled_drones)
        txSlots(end+1)          = src;   %#ok<AGROW>
        scheduled_drones(end+1) = u_src; %#ok<AGROW>
        chosen = chosen + 1;
    end
    ptr     = mod(ptr, N_total) + 1;
    scanned = scanned + 1;
end

state.rrPointer = ptr;
end
