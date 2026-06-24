function txSlots = policy_mw_otimo(eligible, K_free, state, ~, cfg)
% POLICY_MW_OTIMO  Non-switching Max-Weight with theoretically optimal Lyapunov
%  coefficients (optimality ratio psi <= 2, per the lower-bound theorem).
%
%  Non-switching counterpart of POLICY_MW_SW_OTIMO: the per-slot weights are
%  identical, but the policy operates under the no-switching constraint —
%  once a video frame starts it occupies the channel for L consecutive slots
%  (force-continued by schedule_sources, which reserves the channel and
%  reduces K_free).  Here `eligible` already excludes drones with an in-flight
%  video frame, and the policy only fills the remaining K_free channels with
%  new status updates or new video-frame starts.
%
%  Lyapunov coefficients (Sec. IV, Optimality-Ratio theorem):
%
%    V_{u,s} = (Gamma/K) * sqrt(alpha_{u,s}/p_s)
%    V_{u,v} = (Gamma/(K*sqrt(L))) * sqrt(alpha_{u,v}/p_v)
%
%  with alpha_{u,s} = omega_u * n_s,  alpha_{u,v} = omega_u * n_v, giving the
%  per-slot ranking weights (common Gamma/K factor dropped):
%
%    W_{u,s}(t) = sqrt(omega_u * n_s * p_s)       * h_{u,s}(t)   (status AoI)
%    W_{u,v}(t) = sqrt(omega_u * n_v * p_v / L)    * z_u(t)       (video AoI proxy)

p_s = cfg.p_c2;
p_v = cfg.p_vid;
n_s = cfg.n_s;
n_v = cfg.n_v;
L   = cfg.L_vid;

n       = numel(eligible);
sources = zeros(1, 2*n);
weights = zeros(1, 2*n);

for idx = 1:n
    u   = eligible(idx);
    om  = cfg.omega(u);
    h_s = state.dual(u).h1;   % status AoI  h_{u,s}(t)
    h_v = state.dual(u).z_u;  % video AoI proxy  z_u(t)

    W_s = sqrt(om * n_s * p_s) * h_s;
    W_v = sqrt(om * n_v * p_v / L) * h_v;

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
