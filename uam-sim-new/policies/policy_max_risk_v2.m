function txSlots = policy_max_risk_v2(eligible, K_free, state, ~, cfg)
% POLICY_MAX_RISK_V2  Drift-derived Risk-Aware Max-Weight (switching/preemptive).
%
%  Corrects the three issues of policy_max_risk_sw (the "R_u * h" heuristic):
%
%   (1) ENDOGENOUS WEIGHT.  The old index multiplied each flow by the *total*
%       composite risk R_u(t), which itself grows with the very AoI being
%       scheduled (R_nav, R_gnd depend on h_{u,s} through r_u). That makes the
%       weight self-referential and the effective penalty super-linear in age,
%       so the Lyapunov O(1/V) guarantee does NOT transfer. Here the weight is
%       the *constant* linear-surrogate slope (n_s for C2, n_v for video),
%       exactly the coefficients of  Rhat_u = n_s*h_{u,s} + n_v*z_u . The index
%       below is, by construction, the feasible action minimising the per-slot
%       drift-plus-penalty bound -> the performance guarantee applies.
%
%   (2) CROSS-CONTAMINATION.  The old index inflated the *video* index by
%       R_nav + R_gnd (positional risk that video service cannot reduce) and the
%       *C2* index by R_vid (coherence risk that C2 cannot reduce). Each flow is
%       now weighted ONLY by the risk component its own freshness reduces:
%           C2     -> n_s   (navigation + ground sensitivity)
%           video  -> n_v   (coherence sensitivity)
%
%   (3) PRINCIPLED ANTI-STARVATION.  The old code bolted on a 1e-9 epsilon to
%       avoid starving un-initialised drones. Starvation is now handled by the
%       deficit queues x_{u,s}, x_{u,v} that track shortfall against the
%       per-source throughput targets qbar (Sec. 6): a starved flow accumulates
%       debt and its index rises until it is served. The epsilon is kept only as
%       a numerical tie-breaker for the cold-start slots before the risk map and
%       deficit queues are populated.
%
%  Drift-derived index (preemptive: every slot all K channels are renegotiated,
%  partial video frames may be preempted and resumed, so no residual-service or
%  D_u^start term appears):
%
%     W_{u,s}(t) = p_{u,s} * ( V * omega_u * n_s * h_{u,s}(t)  + x_{u,s}(t) )
%     W_{u,v}(t) = p_{u,v} * ( V * omega_u * n_v * z_u(t)/L    + x_{u,v}(t) )
%
%  where
%     V        : Lyapunov risk-vs-stability tradeoff  (cfg.V, larger -> closer
%                to J*, larger debt/transient: the O(1/V) knob)
%     omega_u  : per-UAV priority weight              (state.omega(u) or 1)
%     n_s,n_v  : linear risk-surrogate slopes         (cfg.n_s, cfg.n_v)
%     p_{u,.}  : per-slot success probability         (cfg.p_c2, cfg.p_vid)
%     h_{u,s}  : C2 AoI (slots since last C2 delivery)
%     z_u      : coherence-window AoI proxy           (slots since last frame)
%     x_{u,.}  : deficit queues (throughput debt)     (state.dual(u).x_s/.x_v)
%     L        : video frame length in slots          (cfg.L_vid); the 1/L puts
%                video progress (1/L frame per slot) on the C2 slot-scale.
%
%  Selection: top-K_free flows by W, at most one flow per drone (consistent
%  with the MAC constraint of one grant per UAV per slot used throughout the
%  simulator). Ties and cold-start fall back to AoI order via the epsilon.
%
%  NOTE on field names: this reads cfg.V, cfg.n_s, cfg.n_v, cfg.p_c2, cfg.p_vid
%  and state.dual(u).x_s/.x_v/.h1/.z_u, state.omega. All are read defensively
%  with fallbacks, so the policy still runs if a field is absent (it then
%  degrades gracefully toward the surrogate without debt). In this codebase
%  omega lives in cfg.omega (uniform in the a23 sweeps), so omega_u defaults
%  to 1 here — set state.omega if a per-UAV priority is needed.

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

    % per-UAV priority (eq. R_sys weight omega_u); default 1
    if isfield(state, 'omega') && numel(state.omega) >= u
        omega_u = state.omega(u);
    else
        omega_u = 1.0;
    end

    h_s = state.dual(u).h1;     % C2 AoI
    z_u = state.dual(u).z_u;    % video coherence-window AoI proxy

    % deficit queues (debt); 0 if not yet tracked
    x_s = dual_field(state, u, 'x_s', 0.0);
    x_v = dual_field(state, u, 'x_v', 0.0);

    % ---- drift-derived Max-Weight index (NOT R_total * h) ----
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
% Scalar config read with fallback.
if isfield(s, name) && ~isempty(s.(name))
    val = s.(name);
else
    val = default;
end
end

function val = dual_field(state, u, name, default)
% Per-drone dual-struct read with fallback (handles missing deficit queues).
if isfield(state.dual(u), name) && ~isempty(state.dual(u).(name))
    val = state.dual(u).(name);
else
    val = default;
end
end
