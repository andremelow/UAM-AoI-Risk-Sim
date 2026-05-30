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

    mySlotIdx = find(txDrones == i, 1);
    isMyTurn  = ~isempty(mySlotIdx);

    snr_ok = (~isnan(state.snrLast(i))) && (state.snrLast(i) > cfg.thresholdSNR);

    mu_s = 0;     % status success indicator this slot
    c_u  = 0;     % video frame completion indicator this slot

    if isMyTurn
        isVideo = txIsVideo(mySlotIdx);
        cap_Mbps = state.rawThroughput(i);
        state.avgThroughput(i) = 0.9 * state.avgThroughput(i) + 0.1 * cap_Mbps;
        if ~isVideo
            state.avgThroughputVid(i) = 0.9 * state.avgThroughputVid(i);
        else
            state.avgThroughputC2(i)  = 0.9 * state.avgThroughputC2(i);
        end

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

            if pkt_ok
                state.txSuccess(i) = state.txSuccess(i) + 1;

                if ~isVideo
                    state.slotsC2(i) = state.slotsC2(i) + 1;
                    state.avgThroughputC2(i) = 0.9*state.avgThroughputC2(i) + 0.1*cap_Mbps;
                    state.dual(i).h1        = 0;
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
end
end
