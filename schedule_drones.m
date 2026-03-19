function [txSlots, rrPointer] = schedule_drones( ...
    activeDrones, K, policy, rrPointer, ...
    lastRxTime, now, avgThrput, rawThrput, riskUnc, riskMap, riskVid, ...
    w_unc, w_map, w_vid, alpha, startTimes)
%SCHEDULE_DRONES  Select up to K drones for the current TX slot.
%
%  startTimes  — [numDrones×1] entry time of each drone (from generate_arrival_times).
%                Required for correct AoI computation: drones that have never
%                transmitted have NaN in lastRxTime, so AoI must be measured
%                from their entry time, not from t=0.
%
%  Fix (pf-aoi / risk-aware / hybrid starvation):
%    aoi_seconds() used to return 0 for NaN lastRxTime, giving these drones
%    score=0 and permanent starvation.  Now returns (now - startTime) instead,
%    which is the true AoI for a drone that has never had an update delivered.

txSlots = [];
n = numel(activeDrones);
if n == 0, return; end
K = min(K, n);

%% ------------------------------------------------------------------
%  Channel quality estimate for every active drone.
%  Scheduled last step  → rawThrput  (fresh SNR measurement)
%  Not scheduled        → avgThrput  (EMA proxy)
%  Prevents PF-classic starvation caused by rawThrput==0.
%% ------------------------------------------------------------------
chanEst = zeros(1, n);
for idx = 1:n
    u = activeDrones(idx);
    if rawThrput(u) > 1e-6
        chanEst(idx) = rawThrput(u);
    else
        chanEst(idx) = max(avgThrput(u), 1e-6);
    end
end

switch policy

    case 'round-robin'
        ptr = mod(rrPointer-1, n) + 1;
        for s = 1:K
            txSlots(end+1) = activeDrones(ptr); %#ok<AGROW>
            ptr = mod(ptr, n) + 1;
        end
        rrPointer = ptr;

    case 'pf-classic'
        scores = zeros(1, n);
        for idx = 1:n
            u = activeDrones(idx);
            scores(idx) = chanEst(idx) / max(avgThrput(u), 1e-9);
        end
        [~, order] = sort(scores, 'descend');
        txSlots = activeDrones(order(1:K));

    case 'pf-aoi'
        % score_u = h_u(t) / T_avg(u)
        scores = zeros(1, n);
        for idx = 1:n
            u = activeDrones(idx);
            h_u = aoi_seconds(lastRxTime(u), now, startTimes(u));
            scores(idx) = h_u / max(avgThrput(u), 1e-9);
        end
        [~, order] = sort(scores, 'descend');
        txSlots = activeDrones(order(1:K));

    case 'risk-aware'
        scores = zeros(1, n);
        for idx = 1:n
            u   = activeDrones(idx);
            h_u = aoi_seconds(lastRxTime(u), now, startTimes(u));
            R_u = read_risk(riskUnc, riskMap, riskVid, u, w_unc, w_map, w_vid);
            scores(idx) = (R_u + 1e-9) * (h_u + 1e-3);
        end
        [~, order] = sort(scores, 'descend');
        txSlots = activeDrones(order(1:K));

    case 'hybrid'
        pf_scores   = zeros(1, n);
        risk_scores = zeros(1, n);
        for idx = 1:n
            u   = activeDrones(idx);
            h_u = aoi_seconds(lastRxTime(u), now, startTimes(u));
            pf_scores(idx)   = h_u / max(avgThrput(u), 1e-9);
            R_u              = read_risk(riskUnc, riskMap, riskVid, u, w_unc, w_map, w_vid);
            risk_scores(idx) = (R_u + 1e-9) * (h_u + 1e-3);
        end
        pf_scores   = safe_norm(pf_scores);
        risk_scores = safe_norm(risk_scores);
        combined    = alpha * pf_scores + (1-alpha) * risk_scores;
        [~, order]  = sort(combined, 'descend');
        txSlots     = activeDrones(order(1:K));

    otherwise
        error('schedule_drones: unknown policy "%s". Valid: round-robin, pf-classic, pf-aoi, risk-aware, hybrid', policy);
end
end

%% ======================================================================
%  LOCAL HELPERS
%% ======================================================================

function h = aoi_seconds(lastRx, now, entryTime)
%AOI_SECONDS  True AoI in seconds.
%  - Never transmitted (NaN): AoI = time since drone entered the corridor.
%  - Transmitted before:      AoI = time since last successful reception.
if isnan(lastRx)
    h = max(now - entryTime, 0);
else
    h = max(now - lastRx, 0);
end
end

function R = read_risk(riskUnc, riskMap, riskVid, u, w_unc, w_map, w_vid)
%READ_RISK  Safe read of last risk sample; returns 0 if history empty.
if ~isempty(riskUnc{u})
    R = w_unc*riskUnc{u}(end) + w_map*riskMap{u}(end) + w_vid*riskVid{u}(end);
else
    R = 0;
end
end

function v = safe_norm(v)
%SAFE_NORM  Normalise to [0,1]; uniform weights if all scores equal.
span = max(v) - min(v);
if span < 1e-12
    v = ones(size(v)) / numel(v);
else
    v = (v - min(v)) / span;
end
end