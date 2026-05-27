function state = step_channel(state, infra, ~, cfg)
% STEP_CHANNEL  Computes SNR and capacity for all active drones.
%
%  Extracted from monolith per-drone loop: channel estimation block.
%  Runs for ALL active drones (not just scheduled ones) so the scheduler
%  has fresh channel quality estimates.

for i = state.activeDrones
    pos = state.pendingMsg{i}.pos;

    % Nearest micro-BS
    % microBSPos convention: col1=East, col2=North, col3=Alt(neg=above ground)
    % pos from uavScenario: pos(1)=North, pos(2)=East, pos(3)=Down
    % Convert both to NED [North; East; Down] for nrPathLoss
    bsNED = [infra.microBSPos(:,2), infra.microBSPos(:,1), -infra.microBSPos(:,3)];
    droneNED = [pos(1); pos(2); -pos(3)];
    dist  = sqrt(sum((bsNED - droneNED').^2, 2));
    [~, idxBS] = min(dist);
    posBS = bsNED(idxBS,:)';
    posUE = droneNED;

    PL  = nrPathLoss(infra.plCfg, cfg.fc, true, posBS, posUE);
    SNR = (cfg.pTransmitDrone + cfg.gNB_Gain - PL) - (cfg.thermalNoise + cfg.noiseFigure);
    state.snrLast(i) = SNR;

    % Effective deliverable rate: 0 when SNR < threshold (link unusable).
    % Using effective rate (not raw Shannon cap) as PF numerator ensures
    % drones in outage get score=0 and do not monopolise slots.
    if SNR >= cfg.thresholdSNR
        cap_Mbps = (cfg.bw * log2(1 + 10^(SNR/10))) / 1e6;
    else
        cap_Mbps = 0;
    end
    state.rawThroughput(i) = cap_Mbps;
end
end
