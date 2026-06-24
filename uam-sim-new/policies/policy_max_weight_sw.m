function txSlots = policy_max_weight_sw(eligible, K_free, state, slot, cfg)
% POLICY_MAX_WEIGHT_SW  Switching Max-Weight scheduling (preemptive).
%
%  Implements a quadratic-drift Max-Weight policy aligned with the
%  Lyapunov analysis in the model paper (Sec. IV).
%
%  Status weight W_{u,s}:
%
%    W_{u,s} = omega_u * p_c2 * [ n_s * h_s*(h_s+1) + V_s * x_s ]
%
%  Video weight W_{u,v} — quadratic in z_u, normalised by L_eff:
%
%    z_nat = L_vid * N_eff / K           (natural z_u in slots)
%    L_eff = n_v * z_nat^2 / n_s         (dimensionally derived)
%    W_{u,v} = omega_u * p_vid * [ n_v * z_u*(z_u+1) / L_eff + V_v * x_v ]
%            = omega_u * p_vid * [ n_s * z_u*(z_u+1) / z_nat^2 + V_v * x_v ]
%
%  Derivation: with L_eff = n_v*z_nat^2/n_s the balance W_s=W_v reduces to
%    (z_u/z_nat)^2 = h_s*(h_s+1)    [independent of N_eff, K, n_s, n_v]
%  so video can only preempt C2 when z_u > sqrt(h_s*(h_s+1)) * z_nat —
%  i.e., when its delay exceeds z_nat by the same factor that C2 urgency
%  exceeds its own unit scale. This gives K-invariant balance:
%    h_s=0: video wins (serve video when C2 is freshly delivered)
%    h_s=1: video wins only if z_u > 1.41 * z_nat (41 % above natural)
%    h_s=2: video wins only if z_u > 2.45 * z_nat
%
%  Preemption: eligible includes drones with r>0 (partial frame). The
%  policy evaluates both W_s and W_v every slot and selects the larger,
%  so C2 can preempt video when h_s is large.

n       = numel(eligible);
sources = zeros(1, 2*n);
weights = zeros(1, 2*n);

% Derived normalisation: L_eff = n_v*z_nat^2/n_s where z_nat=L_vid*N_eff/K.
% Balance W_s=W_v reduces to (z_u/z_nat)^2 = h_s*(h_s+1), K-invariant.
if isfield(cfg, 'corridorCapacity') && cfg.corridorCapacity > 0
    N_eff_sw = cfg.corridorCapacity;
else
    N_eff_sw = cfg.numDrones;
end
K_sw     = min(cfg.dronesPerSlot, N_eff_sw);
z_nat    = cfg.L_vid * N_eff_sw / K_sw;
L_eff    = cfg.n_v * z_nat * z_nat / cfg.n_s;

for idx = 1:n
    u   = eligible(idx);
    om  = cfg.omega(u);
    h_s = state.dual(u).h1;
    z_u = state.dual(u).z_u;
    x_s = state.dual(u).x_s;
    x_v = state.dual(u).x_v;

    % --- Status weight W_{u,s} ---
    % h_s*(h_s+1): zero at h_s=0 (video can compete when fresh),
    % equals linear at h_s=1, quadratic for h_s>=2 (urgency without
    % starving video at high K where h_s stays at 1-2).
    W_s = om * cfg.p_c2 * (cfg.n_s * h_s * (h_s + 1) + cfg.V_s * x_s);

    % --- Video weight W_{u,v}: quadratic in z_u, normalised by L_eff ---
    W_v = om * cfg.p_vid * (cfg.n_v * z_u * (z_u + 1) / L_eff ...
                           + cfg.V_v * x_v);

    sources(2*idx-1) = 2*(u-1) + 1;  weights(2*idx-1) = W_s;
    sources(2*idx)   = 2*(u-1) + 2;  weights(2*idx)   = W_v;
end

% Pick top-K_free with at most one source per drone (one radio per drone).
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
