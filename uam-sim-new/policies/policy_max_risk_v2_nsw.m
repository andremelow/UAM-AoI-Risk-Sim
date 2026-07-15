function txSlots = policy_max_risk_v2_nsw(eligible, K_free, state, ~, cfg)
% POLICY_MAX_RISK_V2_NSW  Drift-derived Risk-Aware Max-Weight (NON-switching).
%
%  Non-switching counterpart of policy_max_risk_v2. The per-slot index is the
%  same drift-plus-penalty Max-Weight index, but the policy runs under the
%  no-switching constraint: once a video frame starts it occupies the channel
%  for L consecutive slots. schedule_sources force-continues in-flight frames
%  (reserving channels and reducing K_free) and passes here an `eligible` set
%  that EXCLUDES drones with an in-flight frame. Therefore W_{u,v} below is a
%  video-START weight (a_{u,v} for a fresh frame), not a mid-frame packet.
%
%  Like policy_max_risk_v2, it corrects the three flaws of the R_u*h heuristic
%  (policy_max_risk_nsw):
%   (1) Endogenous weight -> uses the constant linear-surrogate slopes n_s, n_v
%       (coefficients of Rhat_u = n_s*h_{u,s} + n_v*z_u), so the index is the
%       feasible drift-minimising action and the O(1/V) guarantee applies.
%   (2) Cross-contamination -> each flow weighted ONLY by the risk component
%       its own freshness reduces (C2 -> n_s, video -> n_v).
%   (3) Anti-starvation -> deficit queues x_{u,s}, x_{u,v} (throughput debt vs
%       the per-source targets qbar) instead of an ad-hoc epsilon. Under the
%       non-switching force-continue rule the long-term video COMPLETION rate
%       equals the long-term START rate, so debiting starts stabilises the
%       completion target (Appendix throughput-debt argument).
%
%  Drift-derived index:
%     W_{u,s}(t) = p_{u,s} * ( V * omega_u * n_s * h_{u,s}(t)  + x_{u,s}(t) )
%     W_{u,v}(t) = p_{u,v} * ( V * omega_u * n_v * z_u(t)/L    + x_{u,v}(t) )
%
%  with V (cfg.V, default 1), omega_u (state.omega(u) or 1; cfg.omega is
%  uniform in the a23 sweeps), n_s/n_v (cfg.n_s/cfg.n_v), p_{u,.} (cfg.p_c2/
%  cfg.p_vid), and the deficit queues x_{u,.} (state.dual(u).x_s/.x_v).
%
%  Selection: top-K_free flows by W, at most one flow per drone. Fields are read
%  defensively with fallbacks.

% ---- parameters (defensive reads) --------------------------------------
L   = cfg.L_vid;
V   = getfield_default(cfg, 'V',     1.0);
n_s = getfield_default(cfg, 'n_s',   1.0);
n_v = getfield_default(cfg, 'n_v',   1.0);
p_s = getfield_default(cfg, 'p_c2',  1.0);
p_v = getfield_default(cfg, 'p_vid', 1.0);
eps_tie = 1e-9;

n       = numel(eligible);
sources = zeros(1, 2*n);
weights = zeros(1, 2*n);

for idx = 1:n
    u = eligible(idx);

    if isfield(state, 'omega') && numel(state.omega) >= u
        omega_u = state.omega(u);
    else
        omega_u = 1.0;
    end

    h_s = state.dual(u).h1;     % C2 AoI
    z_u = state.dual(u).z_u;    % video coherence-window AoI proxy

    x_s = dual_field(state, u, 'x_s', 0.0);
    x_v = dual_field(state, u, 'x_v', 0.0);

    W_s = p_s * ( V * omega_u * n_s * h_s         + x_s ) + eps_tie * h_s;
    W_v = p_v * ( V * omega_u * n_v * (z_u / L)   + x_v ) + eps_tie * (z_u / L);

    sources(2*idx-1) = 2*(u-1) + 1;  weights(2*idx-1) = W_s;
    sources(2*idx)   = 2*(u-1) + 2;  weights(2*idx)   = W_v;
end

candidates = find(weights > 0);
if isempty(candidates)
    txSlots = [];
    return;
end

[~, order]  = sort(weights(candidates), 'descend');
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

% ========================================================================
function val = getfield_default(s, name, default)
if isfield(s, name) && ~isempty(s.(name))
    val = s.(name);
else
    val = default;
end
end

function val = dual_field(state, u, name, default)
if isfield(state.dual(u), name) && ~isempty(state.dual(u).(name))
    val = state.dual(u).(name);
else
    val = default;
end
end
