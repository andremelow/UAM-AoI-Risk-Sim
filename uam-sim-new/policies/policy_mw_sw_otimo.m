function txSlots = policy_mw_sw_otimo(eligible, K_free, state, ~, cfg)
% POLICY_MW_SW_OTIMO  Switching Max-Weight with theoretically optimal Lyapunov
%  coefficients (optimality ratio ψ ≤ 2, per the lower-bound theorem).
%
%  Derived from the paper's Theorem on Optimality Ratio (Sec. IV).
%  The Lyapunov coefficients are chosen as:
%
%    V_{u,s} = (Γ/K) · √(α_{u,s}/p_s)
%    V_{u,v} = (Γ/(K√L)) · √(α_{u,v}/p_v)
%
%  where  α_{u,s} = ω_u · n_s,  α_{u,v} = ω_u · n_v,
%  and    Γ = Σ_u √(α_{u,s}/p_s) + Σ_u √(L·α_{u,v}/p_v).
%
%  The resulting per-slot weights are (Γ/K common factor dropped; does not
%  affect the ranking):
%
%    W_{u,s}(t) = √(ω_u · n_s · p_s) · h_{u,s}(t)       (status AoI)
%    W_{u,v}(t) = √(ω_u · n_v · p_v / L) · z_u(t)        (video AoI proxy)
%
%  Operating in switching mode: all K channels are renegotiated every slot;
%  partial video frames can be preempted and resumed in a future slot.

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
    h_v = state.dual(u).z_u;  % video AoI proxy  z_u(t)  (resets to ~L after delivery)

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
