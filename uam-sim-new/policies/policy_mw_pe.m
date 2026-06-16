function txSlots = policy_mw_pe(eligible, K_free, state, slot, cfg)
% POLICY_MW_PE  Max-Weight with Explicit Preemption.
%
%  Identical to policy_max_weight_sw with one additional rule:
%
%    if vid_remaining > 0  AND  h_s >= cfg.h_preempt
%        W_v <- 0   (C2 wins unconditionally)
%
%  This guarantees h1 <= cfg.h_preempt by construction, preventing video
%  from monopolising the channel in underloaded corridors (C << K) where
%  n_s/n_v ≈ 1 makes video weights comparable to C2 weights.
%
%  cfg.h_preempt  C2 AoI hard ceiling for preemption (default: 3).
%                 Set to Inf to disable preemption (reduces to mw-sw).

if isfield(cfg, 'h_preempt')
    H_MAX = cfg.h_preempt;
else
    H_MAX = 3;
end

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

    % --- Status weight W_{u,s} ---
    W_s = om * cfg.p_c2 * (cfg.n_s * (h_s + 1) + cfg.V_s * x_s);

    % --- D term ---
    nd = state.dual(u).n_delivered;
    D_sum = 0;
    if nd >= 1
        for ii = 2:w
            if ii-1 <= nd && ~isnan(state.dual(u).frame_gen_hist(ii-1))
                h_im1 = slot - state.dual(u).frame_gen_hist(ii-1);
            else
                h_im1 = z_u;
            end
            D_sum = D_sum + alpha(ii) * (h_im1 + cfg.L_vid);
        end
    else
        for ii = 2:w
            D_sum = D_sum + alpha(ii) * (z_u + cfg.L_vid);
        end
    end
    D = z_u + cfg.L_vid - alpha(1)*cfg.L_vid - D_sum;

    % --- Video weight ---
    L_eff = max(1, 10 * cfg.n_v / cfg.n_s);
    W_v = om * ( (cfg.p_vid * cfg.n_v / L_eff) * D ...
               + cfg.p_vid * cfg.V_v * x_v ...
               - cfg.kappa * L_eff );

    % --- Explicit preemption: h_s at ceiling -> C2 wins unconditionally ---
    if r > 0 && h_s >= H_MAX
        W_v = 0;
    end

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
