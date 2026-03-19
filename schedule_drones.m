function [txSlots, rrPointer] = schedule_drones( ...
    activeDrones, K, policy, rrPointer, ...
    lastRxTime, now, avgThrput,rawThrput, riskUnc, riskMap, riskVid, ...
    w_unc, w_map, w_vid, alpha)

txSlots = [];
n = numel(activeDrones);
if n == 0, return; end
K = min(K, n);

switch policy

    case 'round-robin'
        ptr = mod(rrPointer-1, n) + 1;
        for s = 1:K
            txSlots(end+1) = activeDrones(ptr); %#ok<AGROW>
            ptr = mod(ptr, n) + 1;
        end
        rrPointer = ptr;

    case 'pf-classic'
        % Classic Proportional Fair:
        %   score_u = R_inst(u) / T_avg(u)
        % R_inst = instantaneous achievable rate this slot (rawThrput)
        % T_avg  = exponential moving average of past throughput (avgThrput)
        % Drones with high current channel quality relative to their
        % historical average are prioritised — fairness over time.
        scores = zeros(1, n);
        for idx = 1:n
            u = activeDrones(idx);
            scores(idx) = rawThrput(u) / max(avgThrput(u), 1e-9);
        end
        [~, order] = sort(scores, 'descend');
        txSlots = activeDrones(order(1:K));

    case 'pf-aoi'
        % score_u = h_u(t) / avg_throughput_u
        % Higher AoI and lower past throughput → higher priority
        scores = zeros(1, n);
        for idx = 1:n
            u = activeDrones(idx);
            h_u = max(now - lastRxTime(u), 0);   % AoI in seconds
            scores(idx) = h_u / max(avgThrput(u), 1e-9);
        end
        [~, order] = sort(scores, 'descend');
        txSlots = activeDrones(order(1:K));

    case 'risk-aware'
        % score_u = R_u(t) * h_u(t)
        % Highest instantaneous risk AND most stale → absolute priority
        scores = zeros(1, n);
        for idx = 1:n
            u = activeDrones(idx);
            h_u = max(now - lastRxTime(u), 0);
            if ~isempty(riskUnc{u})
                R_u = w_unc*riskUnc{u}(end) + w_map*riskMap{u}(end) + w_vid*riskVid{u}(end);
            else
                R_u = 0;
            end
            scores(idx) = R_u * h_u;
        end
        [~, order] = sort(scores, 'descend');
        txSlots = activeDrones(order(1:K));

    case 'hybrid'
        % Convex combination: alpha * PF-AoI score + (1-alpha) * Risk-Aware score
        % Both sub-scores are normalised to [0,1] before mixing
        pf_scores   = zeros(1, n);
        risk_scores = zeros(1, n);
        for idx = 1:n
            u = activeDrones(idx);
            h_u = max(now - lastRxTime(u), 0);
            pf_scores(idx) = h_u / max(avgThrput(u), 1e-9);
            if ~isempty(riskUnc{u})
                R_u = w_unc*riskUnc{u}(end) + w_map*riskMap{u}(end) + w_vid*riskVid{u}(end);
            else
                R_u = 0;
            end
            risk_scores(idx) = R_u * h_u;
        end
        pf_scores   = pf_scores   / max(pf_scores,   [], 'all');  % norm to [0,1]
        risk_scores = risk_scores / max(risk_scores, [], 'all');
        combined = alpha * pf_scores + (1-alpha) * risk_scores;
        [~, order] = sort(combined, 'descend');
        txSlots = activeDrones(order(1:K));

    otherwise
        error("Unknown scheduling policy: %s", policy);
end
end