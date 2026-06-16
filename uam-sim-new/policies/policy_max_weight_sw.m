function txSlots = policy_max_weight_sw(eligible, K_free, state, slot, cfg)
% POLICY_MAX_WEIGHT_SW  Switching Max-Weight scheduling (preemptive).
%
%  Implements a quadratic-drift Max-Weight policy aligned with the
%  Lyapunov analysis in the model paper (Sec. IV).
%
%  Status weight W_{u,s} (quadratic drift):
%
%    W_{u,s} = omega_u * p_c2 * [ n_s * (h_s + 1)^2 + V_s * x_s ]
%
%  The (h_s+1)^2 term is the quadratic Lyapunov drift of (h_s+1): its
%  one-step increase is 2*(h_s+1), so the weight grows FAST with AoI and
%  the equilibrium h_s drops from ~6 (linear) to ~2 (quadratic).
%  The "+1" baseline keeps W_s > 0 at h_s=0, preventing video monopolisation
%  in underloaded corridors (see L_eff comment below).
%
%  Video weight W_{u,v} (unchanged from the linear-drift policy):
%
%    W_{u,v} = omega_u * [ (p_vid * n_v / L_eff) * D + p_vid * V_v * x_v - kappa * L_eff ]
%
%    D       — expected z_u reduction when frame completes (linear in z_u)
%    L_eff   = max(1, 10 * n_v / n_s) — prevents video monopolisation at low C
%
%  Preemption: eligible includes drones with r>0 (partial frame). The
%  policy evaluates both W_s and W_v every slot and selects the larger,
%  so C2 can preempt video when h_s is large.

L     = cfg.L_vid;
alpha = cfg.alpha_w(:);
w     = cfg.w_coh;

n       = numel(eligible);
sources = zeros(1, 2*n);
weights = zeros(1, 2*n);

for idx = 1:n
    u   = eligible(idx);
    om  = cfg.omega(u);
    h_s = state.dual(u).h1;
    z_u = state.dual(u).z_u;
    x_s = state.dual(u).x_s;
    x_v = state.dual(u).x_v;
    r   = state.dual(u).vid_remaining;

    % --- Status weight W_{u,s} (quadratic drift) ---
    % (h_s+1)^2 = h_s^2 + 2*h_s + 1: the Lyapunov drift of (h_s+1)^2 is
    % 2*(h_s+1), giving quadratic growth in h_s while staying positive at h_s=0.
    W_s = om * cfg.p_c2 * (cfg.n_s * (h_s + 1)^2 + cfg.V_s * x_s);

    % --- D term: expected z_u reduction at frame completion ---
    %  Identical for fresh start and continuation: the frame was generated
    %  at vid_gen_slot; completing it gives the same AoI update regardless
    %  of how many packets remain.
    nd = state.dual(u).n_delivered;
    D_sum = 0;
    if nd >= 1
        for ii = 2:w
            if ii-1 <= nd && ~isnan(state.dual(u).frame_gen_hist(ii-1))
                h_im1 = slot - state.dual(u).frame_gen_hist(ii-1);
            else
                h_im1 = z_u;
            end
            D_sum = D_sum + alpha(ii) * (h_im1 + L);
        end
    else
        for ii = 2:w
            D_sum = D_sum + alpha(ii) * (z_u + L);
        end
    end
    D = z_u + L - alpha(1)*L - D_sum;

    % --- Video weight W_{u,v} ---
    % L_eff normalises the video threshold across corridor sizes.
    % Without it, small N_eff (n_s→0) lets video monopolise and spikes
    % R_gnd via Jensen (R_gnd ∝ h1²). Clamped at 1 for large N_eff (C≥16).
    L_eff = max(1, 10 * cfg.n_v / cfg.n_s);
    W_v = om * ( (cfg.p_vid * cfg.n_v / L_eff) * D ...
               + cfg.p_vid * cfg.V_v * x_v ...
               - cfg.kappa * L_eff );

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
