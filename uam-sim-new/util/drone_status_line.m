function str = drone_status_line(k, time, state, txSlots, cfg)
% DRONE_STATUS_LINE  Formats one line of the drone status panel.
%  Shows h1 (C2 AoI) and h2 (Video AoI) instead of legacy AoI/FDR.

txDroneSet = unique(ceil(txSlots / 2));

if time < state.startTimes(k)
    statusStr = 'WT';  pct = 0;
elseif time > state.endTimes(k)
    statusStr = 'CO';  pct = 100;
elseif ismember(k, txDroneSet)
    statusStr = 'TR';
    pct = (time - state.startTimes(k)) / (state.endTimes(k) - state.startTimes(k)) * 100;
else
    statusStr = 'FL';
    pct = (time - state.startTimes(k)) / (state.endTimes(k) - state.startTimes(k)) * 100;
end

filled = round(pct / 12.5);
bar    = [repmat(char(9608), 1, filled), repmat(char(9617), 1, 8-filled)];

% h1 (C2 AoI) — convert slots to ms
ms_per_slot = 1000 / cfg.updateRate;
if time < state.startTimes(k) || time > state.endTimes(k)
    h1Str = '   --- ';
else
    h1_ms = state.dual(k).h1 * ms_per_slot;
    if h1_ms >= 1000, h1Str = sprintf('%4.1fs ', h1_ms/1000);
    else,             h1Str = sprintf('%4.0fms', h1_ms);
    end
end

% h2 (Video AoI) — convert slots to ms
if time < state.startTimes(k) || time > state.endTimes(k)
    h2Str = '   --- ';
else
    h2_ms = state.dual(k).h2 * ms_per_slot;
    if h2_ms >= 1000, h2Str = sprintf('%4.1fs ', h2_ms/1000);
    else,             h2Str = sprintf('%4.0fms', h2_ms);
    end
end

% SNR
if isnan(state.snrLast(k)) || time < state.startTimes(k)
    snrStr = ' ---';
elseif state.snrLast(k) >= cfg.thresholdSNR
    snrStr = sprintf('%+5.1fdB ', state.snrLast(k));
else
    snrStr = sprintf('%+5.1fdB*', state.snrLast(k));
end

okStr  = sprintf('%4d', state.txSuccess(k));
errStr = sprintf('%4d', state.txFail(k));

str = sprintf('U%02d %-3s %s h1:%s h2:%s ok:%s e:%s S:%s', ...
              k, statusStr, bar, h1Str, h2Str, okStr, errStr, snrStr);
end
