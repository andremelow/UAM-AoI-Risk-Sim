function [txSlots, state] = schedule_sources(state, slot, cfg)
% SCHEDULE_SOURCES  Selects source(s) for transmission each slot.
%
%  Doc-aligned (Sec. 4.1, 4.5):
%    - Ongoing video transmissions (vid_remaining > 0) reserve a channel.
%    - K_free(t) = K - sum_u 1{r_u(t) > 0} channels remain available
%      for new scheduling decisions.
%    - Drones with r_u(t) > 0 cannot be issued a NEW video start (b_u=0)
%      but their ongoing transmission is automatically continued.
%
%  Policies:
%    'round-robin'         — cycles [C2_1, vid_1, C2_2, vid_2, ...] over eligible sources
%    'round-robin-aware'   — RR but skips drones below SNR threshold (still naive about AoI/risk)
%    'pf-classic'          — source-level PF: score = r_u / T_avg_source
%    'aoi-pure'            — Max-Weight AoI: score = h_source
%    'risk-aware'          — Risk x AoI heuristic (legacy)
%    'max-weight'          — Doc Sec. 4.5: full Max-Weight with W_{u,s}, W_{u,v}^start
%    'max-weight-drift'    — Analysis Sec. 2: quadratic-drift Max-Weight (beta_s/beta_v/V)
%    'best-cqi'            — Max-SNR / opportunistic: schedule highest-capacity drone each slot

activeDrones = state.activeDrones;
txSlots = [];
if isempty(activeDrones), return; end

% --- Compute K_free: subtract channels held by ongoing video tx ---
%
%  Doc Sec. 4.1: r_u(t) > 0 means a video transmission is ALREADY in
%  progress (a channel was allocated in a previous slot). It is NOT the
%  same as having a frame pending in the buffer (vid_remaining > 0): a
%  pending frame is only "ongoing" once the scheduler has started it.
%  We track this with the explicit flag vid_in_flight, set in
%  step_transmit when the first packet of a frame is sent and cleared
%  when the frame finishes.
busy_uavs = activeDrones(arrayfun(@(u) state.dual(u).vid_in_flight, activeDrones));
n_busy = numel(busy_uavs);
K_free = max(cfg.dronesPerSlot - n_busy, 0);

% Force-continue ongoing video transmissions
% (encoded as the "video" source id of each busy UAV; step_transmit will
%  decrement vid_remaining and potentially complete the frame)
for k = 1:n_busy
    u = busy_uavs(k);
    txSlots(end+1) = 2*(u-1) + 2;     %#ok<AGROW>  video source
end

if K_free == 0
    return;
end

% --- Eligible drones for NEW decisions: those NOT in mid-transmission ---
eligible = setdiff(activeDrones, busy_uavs);
if isempty(eligible)
    return;
end

% --- Dispatch policy ---
switch cfg.schedulingPolicy

    case 'round-robin'
        % Stable global source IDs: src in {1..2N}.
        % src odd  -> C2 of UAV ceil(src/2)
        % src even -> Vid of UAV src/2
        N_total = 2 * cfg.numDrones;
        % rrPointer indexes the LAST source served + 1, in this global list.
        ptr = mod(state.rrPointer - 1, N_total) + 1;
        chosen = 0;
        scanned = 0;
        while chosen < K_free && scanned < N_total
            src = ptr;
            u_src = ceil(src/2);
            isVid = (mod(src, 2) == 0);
            % Eligibility: drone must be currently active AND not already
            % busy with an in-flight video this slot.
            if ismember(u_src, eligible)
                txSlots(end+1) = src; %#ok<AGROW>
                chosen = chosen + 1;
                if isVid
                    state.dual(u_src).vid_in_flight = true;
                end
            end
            ptr = mod(ptr, N_total) + 1;
            scanned = scanned + 1;
        end
        state.rrPointer = ptr;

    case 'round-robin-aware'
        [new_slots, state] = policy_round_robin_aware(eligible, K_free, state, cfg);
        txSlots = [txSlots, new_slots];

    case 'pf-classic'
        new_slots = policy_pf_classic(eligible, K_free, state, cfg);
        txSlots = [txSlots, new_slots];
        state = mark_vid_starts(state, new_slots);

    case 'aoi-pure'
        new_slots = policy_aoi_pure(eligible, K_free, state, slot, cfg);
        txSlots = [txSlots, new_slots];
        state = mark_vid_starts(state, new_slots);

    case 'risk-aware'
        new_slots = policy_risk_aware(eligible, K_free, state, slot, cfg);
        txSlots = [txSlots, new_slots];
        state = mark_vid_starts(state, new_slots);

    case 'max-weight'
        new_slots = policy_max_weight_doc(eligible, K_free, state, slot, cfg);
        txSlots = [txSlots, new_slots];
        state = mark_vid_starts(state, new_slots);

    case 'max-weight-drift'
        new_slots = policy_max_weight_drift(eligible, K_free, state, slot, cfg);
        txSlots = [txSlots, new_slots];
        state = mark_vid_starts(state, new_slots);

    case 'best-cqi'
        new_slots = policy_best_cqi(eligible, K_free, state, slot, cfg);
        txSlots = [txSlots, new_slots];
        state = mark_vid_starts(state, new_slots);

    otherwise
        error(['schedule_sources: unknown policy "%s".\n' ...
               'Valid: round-robin, round-robin-aware, pf-classic, aoi-pure, risk-aware, max-weight, max-weight-drift, best-cqi'], ...
              cfg.schedulingPolicy);
end
end


function state = mark_vid_starts(state, new_slots)
% Mark video starts so K_free reserves the channel next slot (Sec. 4.1).
for s = 1:numel(new_slots)
    src = new_slots(s);
    if mod(src, 2) == 0
        u_v = src/2;
        state.dual(u_v).vid_in_flight = true;
    end
end
end
