function state = step_transmit(state, txSlots, slot, cfg)
% STEP_TRANSMIT  Dual-source transmission and AoI evolution.
%
%  Doc-aligned (Sec. 4.2 of analysis doc):
%    - Status success mu_{u,s}(t) ~ Bernoulli(p_c2) when scheduled
%    - Video frame completion c_u(t) when r_u(t) was 1 entering this slot
%    - AoI: h_{u,s}(t+1) = h_{u,s}(t) + 1 - mu * h_{u,s}(t)
%    - Debt queues:
%        x_{u,s}(t+1) = [x_{u,s}(t) + qbar_s - mu_{u,s}(t)]^+
%        x_{u,v}(t+1) = [x_{u,v}(t) + qbar_v - c_u(t)]^+
%
%  The SNR/threshold check still applies as a hard physical gate;
%  on top of it, p_c2 / p_vid model residual link reliability per the
%  doc (independent of the geometric SNR mask).

% --- Decode scheduled sources ---
txDrones  = zeros(1, numel(txSlots));
txIsVideo = false(1, numel(txSlots));
for s = 1:numel(txSlots)
    src = txSlots(s);
    txDrones(s)  = ceil(src / 2);
    txIsVideo(s) = (mod(src, 2) == 0);
end

for i = state.activeDrones

    mySlotIndices = find(txDrones == i);   % ALL sources scheduled for drone i
    isMyTurn      = ~isempty(mySlotIndices);

    snr_ok = (~isnan(state.snrLast(i))) && (state.snrLast(i) > cfg.thresholdSNR);

    mu_s = 0;     % status success indicator this slot
    c_u  = 0;     % video frame completion indicator this slot

    if isMyTurn
        cap_Mbps = state.rawThroughput(i);
        state.avgThroughput(i) = 0.9 * state.avgThroughput(i) + 0.1 * cap_Mbps;

        % Determine which source types are scheduled for throughput decay logic.
        has_c2  = any(~txIsVideo(mySlotIndices));
        has_vid = any( txIsVideo(mySlotIndices));
        if ~has_vid, state.avgThroughputVid(i) = 0.9 * state.avgThroughputVid(i); end
        if ~has_c2,  state.avgThroughputC2(i)  = 0.9 * state.avgThroughputC2(i);  end

        % Process each scheduled source independently (C2 and/or video).
        for si = 1:numel(mySlotIndices)
            isVideo = txIsVideo(mySlotIndices(si));

            % Combined success: SNR mask AND Bernoulli reliability
            if snr_ok
                if ~isVideo
                    pkt_ok = (rand() < cfg.p_c2);
                else
                    pkt_ok = (rand() < cfg.p_vid);
                end
            else
                pkt_ok = false;
            end

            % Tx error accounting per source: [att, outage_fail, bern_fail]
            % outage_fail = SNR below threshold; bern_fail = in coverage but
            % Bernoulli(p) miss. cols 1-3 = C2, cols 4-6 = video.
            if ~isVideo
                state.txStat(1) = state.txStat(1) + 1;
                if ~snr_ok,     state.txStat(2) = state.txStat(2) + 1;
                elseif ~pkt_ok, state.txStat(3) = state.txStat(3) + 1; end
            else
                state.txStat(4) = state.txStat(4) + 1;
                if ~snr_ok,     state.txStat(5) = state.txStat(5) + 1;
                elseif ~pkt_ok, state.txStat(6) = state.txStat(6) + 1; end
            end

            if pkt_ok
                state.txSuccess(i) = state.txSuccess(i) + 1;

                if ~isVideo
                    state.slotsC2(i) = state.slotsC2(i) + 1;
                    state.avgThroughputC2(i) = 0.9*state.avgThroughputC2(i) + 0.1*cap_Mbps;
                    % AoI reseta para a idade da amostra C2 retida:
                    % generate-at-will -> c2_gen_slot=slot -> h1=0 (modelo atual);
                    % periódico       -> h1 = slot - c2_gen_slot (idade > 0).
                    state.dual(i).h1        = max(0, slot - state.dual(i).c2_gen_slot);
                    state.dual(i).lastRx_c2 = slot;
                    state.lastRxSlot(i)     = slot;
                    mu_s = 1;
                else
                    state.slotsVid(i) = state.slotsVid(i) + 1;
                    state.avgThroughputVid(i) = 0.9*state.avgThroughputVid(i) + 0.1*cap_Mbps;
                    if state.dual(i).vid_remaining > 0
                        % Mark channel as in-flight (so scheduler reserves it
                        % next slot if not yet finished).
                        state.dual(i).vid_in_flight = true;
                        state.dual(i).vid_remaining = state.dual(i).vid_remaining - 1;
                        if state.dual(i).vid_remaining == 0
                            % Frame fully delivered -> completion event c_u(t)=1
                            state.dual(i).vid_in_flight = false;
                            state.dual(i).h2 = max(0, slot - state.dual(i).vid_gen_slot - 1);
                            state.dual(i).lastRx_vid  = slot;
                            state.dual(i).n_delivered = state.dual(i).n_delivered + 1;
                            state.dual(i).frame_gen_hist = ...
                                [state.dual(i).vid_gen_slot; ...
                                 state.dual(i).frame_gen_hist(1:end-1)];
                            state.lastRxSlot(i) = slot;
                            c_u = 1;
                            % Load next pending frame immediately (will be
                            % started by scheduler in a FUTURE slot — not
                            % auto-continued: vid_in_flight stays false).
                            if state.dual(i).vid_pending
                                state.dual(i).vid_remaining   = state.dual(i).vid_pending_L;
                                state.dual(i).vid_gen_slot    = state.dual(i).vid_pending_slot;
                                state.dual(i).vid_L           = state.dual(i).vid_pending_L;
                                state.dual(i).vid_pending     = false;
                            end
                        end
                    end
                end
            else
                state.txFail(i) = state.txFail(i) + 1;
                if ~isVideo
                    state.slotsC2Fail(i)  = state.slotsC2Fail(i)  + 1;
                else
                    state.slotsVidFail(i) = state.slotsVidFail(i) + 1;
                    % NOTE: a failed packet on an in-flight frame does NOT
                    % clear vid_in_flight — the channel stays reserved until
                    % the frame either completes or is cancelled.
                end
            end
        end   % end per-source loop
    else
        % Not scheduled
        link_ok = isnan(state.snrLast(i)) || state.snrLast(i) >= cfg.thresholdSNR;
        if link_ok
            state.avgThroughput(i)    = 0.9 * state.avgThroughput(i);
            state.avgThroughputC2(i)  = 0.9 * state.avgThroughputC2(i);
            state.avgThroughputVid(i) = 0.9 * state.avgThroughputVid(i);
        end
    end

    % --- Debt queue updates (Sec. 4.2 of doc) ---
    state.dual(i).x_s = max(state.dual(i).x_s + cfg.qbar_s(i) - mu_s, 0);
    state.dual(i).x_v = max(state.dual(i).x_v + cfg.qbar_v(i) - c_u, 0);

    % --- AoI aging (slots) ---
    state.dual(i).h1 = state.dual(i).h1 + 1;
    state.dual(i).h2 = state.dual(i).h2 + 1;

    % --- MW-Z envelope virtual queue: after AoI aging ---
    %  Z <- max(0, Z + Rhat - b_u).  Two Rhat modes:
    %    mw-z-v9 : gradient alphas (same as policy_mw_z_v9) — position-aware
    %    others  : constant alphas alpha=omega*(n_s, n_v) (policy_mw_z spec)
    %  Updated for every policy (read only by mw-z*; logged for diagnosis).
    om_i = cfg.omega(min(i, numel(cfg.omega)));
    if strcmp(cfg.schedulingPolicy, 'mw-z-v9')
        r_ui = state.aoiRadius(i);
        if r_ui <= 0, r_ui = cfg.r_min; end
        if ~isempty(state.riskMap{i})
            R_gnd_i = state.riskMap{i}(end);
        else
            R_gnd_i = cfg.rho_0 * pi * cfg.r_min^2;
        end
        if state.dual(i).in_ramp
            we_u = cfg.w_tol_unc; we_m = cfg.w_tol_map; we_v = cfg.w_tol_vid;
        else
            we_u = cfg.w_unc;     we_m = cfg.w_map;     we_v = cfg.w_vid;
        end
        v_i  = cfg.v_max / cfg.updateRate;
        a_si = om_i * (we_u * cfg.C_Nav_1 + we_m * (2 * R_gnd_i * v_i / r_ui));
        a_vi = om_i * we_v / cfg.H_nom_slots;
        Rhat = a_si * state.dual(i).h1 + a_vi * state.dual(i).h2;
    else
        Rhat = om_i * (cfg.n_s * state.dual(i).h1 + cfg.n_v * state.dual(i).h2);
    end
    b_i  = cfg.b_u(min(i, numel(cfg.b_u)));
    state.Z_env(i) = max(0, state.Z_env(i) + Rhat - b_i);

    % --- Video frame system time g_u (doc switching model) ---
    %  g_u(t) = age of the frame currently in service = slot - vid_gen_slot
    %  while a frame is loaded (vid_remaining > 0); 0 when idle. Tracked here
    %  for logging/inspection; policy_mw_sw_doc_g_u recomputes it at decision
    %  time from vid_gen_slot for exactness.
    if state.dual(i).vid_remaining > 0
        state.dual(i).g_u = max(slot - state.dual(i).vid_gen_slot, 0);
    else
        state.dual(i).g_u = 0;
    end

    % --- AoI radius from C2 ---
    state.aoiRadius(i) = cfg.r_min + (cfg.v_max / cfg.updateRate) * state.dual(i).h1;

    % --- Logging ---
    state.h1Time{i}(end+1) = slot;
    state.h1Val{i}(end+1)  = state.dual(i).h1;
    state.h2Time{i}(end+1) = slot;
    state.h2Val{i}(end+1)  = state.dual(i).h2;
    state.zuTime{i}(end+1) = slot;
    state.zuVal{i}(end+1)  = state.dual(i).z_u;
    state.xsVal{i}(end+1)  = state.dual(i).x_s;
    state.xvVal{i}(end+1)  = state.dual(i).x_v;
    state.ZenvVal{i}(end+1) = state.Z_env(i);
end
end
