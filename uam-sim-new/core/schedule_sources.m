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
%    'aoi-normalized'      — Normalised AoI: score = h_source / L_source  (slots/frame)
%    'risk-aware'          — Risk x AoI heuristic (legacy)
%    'max-weight'          — Doc Sec. 4.5: full Max-Weight with W_{u,s}, W_{u,v}^start
%    'max-weight-drift'    — Analysis Sec. 2: quadratic-drift Max-Weight (beta_s/beta_v/V)
%    'best-cqi'            — Max-SNR / opportunistic: schedule highest-capacity drone each slot
%
%  Policy mode (declared in each policy's companion _info function):
%    'no-switching'  — non-preemptive: in-flight video frames hold the channel
%                      until completion (force-continue, K_free reduced).
%    'switching'     — preemptive: scheduler decides freely every slot;
%                      vid_in_flight flags are cleared before dispatch so
%                      no channel is pre-reserved; partial frames resume
%                      (vid_remaining preserved) if the drone is rescheduled.

activeDrones = state.activeDrones;
txSlots = [];
if isempty(activeDrones), return; end

is_switching = get_policy_mode(cfg.schedulingPolicy);

if ~is_switching
    % --- NO-SWITCHING: force-continue in-flight video, reduce K_free ---
    %
    %  Doc Sec. 4.1: vid_in_flight marks channels reserved by ongoing frames.
    %  Force-continued drones are pre-added to txSlots; the policy only
    %  fills the remaining K_free slots from the non-busy eligible set.
    busy_uavs = activeDrones(arrayfun(@(u) state.dual(u).vid_in_flight, activeDrones));
    n_busy = numel(busy_uavs);
    K_free = max(cfg.dronesPerSlot - n_busy, 0);

    for k = 1:n_busy
        u = busy_uavs(k);
        txSlots(end+1) = 2*(u-1) + 2;  %#ok<AGROW>  video source
    end

    if K_free == 0, return; end

    eligible = setdiff(activeDrones, busy_uavs);
    if isempty(eligible), return; end

else
    % --- SWITCHING: clear all in-flight reservations, full K_free ---
    %
    %  No channel is pre-reserved. The policy has full control over all
    %  K slots. Drones with vid_remaining > 0 remain in eligible so the
    %  policy can resume partial frames; step_transmit continues from
    %  where vid_remaining left off if the drone is rescheduled.
    for u = activeDrones
        state.dual(u).vid_in_flight = false;
    end
    K_free = cfg.dronesPerSlot;
    eligible = activeDrones;
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

    case 'aoi-normalized'
        new_slots = policy_aoi_normalized(eligible, K_free, state, slot, cfg);
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

    % --- Switching variants ---
    case 'round-robin-sw'
        [new_slots, state] = policy_round_robin_sw(eligible, K_free, state, cfg);
        txSlots = [txSlots, new_slots];

    case 'pf-classic-sw'
        new_slots = policy_pf_classic_sw(eligible, K_free, state, cfg);
        txSlots = [txSlots, new_slots];

    case 'max-weight-sw'
        new_slots = policy_max_weight_sw(eligible, K_free, state, slot, cfg);
        txSlots = [txSlots, new_slots];

    otherwise
        error(['schedule_sources: unknown policy "%s".\n' ...
               'Valid: round-robin, round-robin-aware, pf-classic, aoi-pure, aoi-normalized, ' ...
               'risk-aware, max-weight, max-weight-drift, best-cqi, ' ...
               'round-robin-sw, pf-classic-sw, max-weight-sw'], ...
              cfg.schedulingPolicy);
end
end


function state = mark_vid_starts(state, new_slots)
% Mark video starts so K_free reserves the channel next slot (Sec. 4.1).
% In switching mode this flag is cleared at the top of the next call, so
% it only has effect in no-switching policies.
for s = 1:numel(new_slots)
    src = new_slots(s);
    if mod(src, 2) == 0
        u_v = src/2;
        state.dual(u_v).vid_in_flight = true;
    end
end
end


function is_sw = get_policy_mode(policy_name)
% GET_POLICY_MODE  Returns true if the policy is a switching (preemptive) policy.
%
%  Each policy declares its mode via a companion _info function.
%  The mapping from policy string to info function mirrors the dispatcher above.
%  Unknown policies default to no-switching (safe fallback).
switch policy_name
    case 'round-robin'
        info = policy_round_robin_info();
    case 'round-robin-aware'
        info = policy_round_robin_aware_info();
    case 'pf-classic'
        info = policy_pf_classic_info();
    case 'aoi-pure'
        info = policy_aoi_pure_info();
    case 'aoi-normalized'
        info = policy_aoi_normalized_info();
    case 'risk-aware'
        info = policy_risk_aware_info();
    case 'max-weight'
        info = policy_max_weight_doc_info();
    case 'max-weight-drift'
        info = policy_max_weight_drift_info();
    case 'best-cqi'
        info = policy_best_cqi_info();
    case 'round-robin-sw'
        info = policy_round_robin_sw_info();
    case 'pf-classic-sw'
        info = policy_pf_classic_sw_info();
    case 'max-weight-sw'
        info = policy_max_weight_sw_info();
    otherwise
        warning('schedule_sources: no _info found for policy "%s", assuming no-switching.', ...
                policy_name);
        is_sw = false;
        return;
end
is_sw = strcmp(info.mode, 'switching');
end
