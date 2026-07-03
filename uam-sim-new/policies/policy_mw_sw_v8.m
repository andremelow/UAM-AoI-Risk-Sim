function txSlots = policy_mw_sw_v8(eligible, K_free, state, ~, cfg)
% POLICY_MW_SW_V8  Drift Max-Weight for the switching system (theory doc V8).
%
%  Implements "Max-Weight Policy for the Switching System" (ref/MW policy for
%  switching (simulation).pdf) exactly as derived:
%
%    1. Lower bound (Sec. 2, closed form via Cauchy-Schwarz + box projection):
%         q_{u,s}^LB = K·sqrt(a_s·p_s) / Theta,   clipped to (0, p_s]
%         q_{u,v}^LB = K·sqrt(a_v·p_v/L) / Theta, clipped to (0, p_v/L]
%         Theta      = sum_u sqrt(a_s/p_s) + sum_u sqrt(a_v·L/p_v)
%       with a_s = omega_u·n_s, a_v = omega_u·n_v (surrogate risk coefficients
%       alpha_{u,s}, alpha_{u,v} of eq. 15) over the N_eff simultaneous UAVs.
%
%    2. Lyapunov coefficients (Sec. 3.1):
%         V_{u,s} = a_s / q_{u,s}^LB,     V_{u,v} = a_v / q_{u,v}^LB
%
%    3. Scheduling weights (Sec. 3.3, drift of L(S) = sum V_s·h_s + V_v·Phi):
%         W_{u,s}(t) = V_{u,s}·p_{u,s}(t)·h_{u,s}(t)
%         W_{u,v}(t) = V_{u,v}·p_{u,v}(t)/L · h_{u,v}(t)
%
%  h_{u,v} is the AoI of the latest COMPLETED video frame (state.dual(u).h2,
%  eq. 5 of the doc) — NOT the coherence-window AoI z_u used by mw-sw-otimo.
%
%  Success probability: the doc's p_{u,s}, p_{u,v} are theoretical constants.
%  As in the other -ch policies we use the channel-CONDITIONED probability in
%  the WEIGHTS, p_{u,·}(t) = p·1{SNR_u(t) >= thresholdSNR} (outage -> weight 0),
%  while the offline lower bound q^LB keeps the nominal p_c2/p_vid.
%
%  Differences vs policy_mw_sw_otimo_ch (same seed-for-seed scenario):
%    - video urgency h_{u,v} (completed-frame AoI) instead of z_u;
%    - V from the box-CLIPPED q^LB (the sqrt form ignores clipping);
%    - p enters linearly (drift form) instead of inside the sqrt.
%
%  Switching mode: all K channels renegotiated every slot; partial frames
%  preempted/resumed. At most one source (C2 or video) per drone per slot.

p_s = cfg.p_c2;
p_v = cfg.p_vid;
n_s = cfg.n_s;
n_v = cfg.n_v;
L   = cfg.L_vid;
K   = cfg.dronesPerSlot;
thr = cfg.thresholdSNR;

% ── Lower bound + Lyapunov coefficients (theoretical p, N_eff sources) ──
N_eff = cfg.corridorCapacity;
om_v  = cfg.omega(:);
om_v  = om_v(1:min(N_eff, numel(om_v)));            % omega over simultaneous UAVs
Theta = sum(sqrt(om_v * n_s / p_s)) + sum(sqrt(om_v * n_v * L / p_v));

n       = numel(eligible);
sources = zeros(1, 2*n);
weights = zeros(1, 2*n);

for idx = 1:n
    u   = eligible(idx);
    om  = cfg.omega(min(u, numel(cfg.omega)));
    a_s = om * n_s;                                  % alpha_{u,s}
    a_v = om * n_v;                                  % alpha_{u,v}

    % q^LB closed form, projected onto the box constraints of Sec. 2
    q_s = min(K * sqrt(a_s * p_s)     / Theta, p_s);
    q_v = min(K * sqrt(a_v * p_v / L) / Theta, p_v / L);

    V_s = a_s / q_s;
    V_v = a_v / q_v;

    h_s = state.dual(u).h1;      % status AoI h_{u,s}
    h_v = state.dual(u).h2;      % AoI of latest COMPLETED video frame h_{u,v}

    % channel-conditioned success probability (0 in outage)
    snr_u  = state.snrLast(u);
    snr_ok = isnan(snr_u) || (snr_u >= thr);
    ps_eff = p_s * snr_ok;
    pv_eff = p_v * snr_ok;

    W_s = V_s * ps_eff * h_s;
    W_v = V_v * pv_eff / L * h_v;

    sources(2*idx-1) = 2*(u-1) + 1;  weights(2*idx-1) = W_s;
    sources(2*idx)   = 2*(u-1) + 2;  weights(2*idx)   = W_v;
end

candidates = find(weights > 0);
if isempty(candidates)
    txSlots = [];
    return;
end

[~, order] = sort(weights(candidates), 'descend');
txSlots     = [];
used_drones = [];
for oi = 1:numel(order)
    if numel(txSlots) >= K_free, break; end
    src = sources(candidates(order(oi)));
    u   = ceil(src / 2);
    if ~ismember(u, used_drones)
        txSlots(end+1)     = src; %#ok<AGROW>
        used_drones(end+1) = u;   %#ok<AGROW>
    end
end
end
