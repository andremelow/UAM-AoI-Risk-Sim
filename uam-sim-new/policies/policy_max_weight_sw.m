function txSlots = policy_max_weight_sw(eligible, K_free, state, slot, cfg)
% POLICY_MAX_WEIGHT_SW  Switching Max-Weight scheduling (preemptive).
%
%  Extends policy_max_weight_doc to switching mode: eligible contains ALL
%  active drones, including those with vid_remaining > 0. No channel is
%  pre-reserved; the policy decides every slot from scratch.
%
%  Video weight for drone u:
%
%    vid_remaining == 0  (no frame in progress or fresh start):
%      W_{u,v}^start = standard Doc Sec. 4.5 formula with L = cfg.L_vid
%
%    vid_remaining == r > 0  (partial frame, r packets left):
%      W_{u,v}^cont  = omega_u [ (p_vid * n_v / r) * D_cont
%                               + p_vid * V_v * x_v
%                               - kappa * r ]
%
%      where D_cont = D_start  (same AoI reduction at completion;
%      the frame was generated at the same vid_gen_slot regardless of
%      how many packets have been sent).
%
%  The 1/r factor increases the per-slot benefit as the frame nears
%  completion, naturally biasing the policy toward finishing partial frames
%  rather than abandoning them (avoids wasted work while still allowing
%  preemption when C2 urgency is sufficiently high).
%
%  Status weight W_{u,s} is identical to policy_max_weight_doc.

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
    r   = state.dual(u).vid_remaining;   % packets left in partial frame

    % --- Status weight W_{u,s} ---
    % Uses (h_s + 1) instead of h_s: Lyapunov drift of h_s^2 per slot is
    % 2*h_s + 1, so the "+1" baseline ensures W_s > 0 even when h_s = 0.
    % Without it, W_s → 0 as h_s → 0 and video monopolises the channel in
    % underloaded corridors (C ≪ K), inflating risk via Jensen on R_gnd.
    W_s = om * cfg.p_c2 * (cfg.n_s * (h_s + 1) + cfg.V_s * x_s);

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

    % --- Video weight ---
    % L_eff normalises threshold = 3*L_eff*(n_s/n_v)*h1 to a constant ~30
    % across corridor sizes.  Without this, small N_eff (n_s→0) drives the
    % threshold to ~5*h1, letting video monopolise and spiking R_gnd via
    % Jensen (R_gnd ∝ h1²).  Clamped at 1 for large N_eff (C≥16).
    L_eff = max(1, 10 * cfg.n_v / cfg.n_s);
    W_v = om * ( (cfg.p_vid * cfg.n_v / L_eff) * D ...
               + cfg.p_vid * cfg.V_v * x_v ...
               - cfg.kappa * L_eff );

    sources(2*idx-1) = 2*(u-1) + 1;  weights(2*idx-1) = W_s;
    sources(2*idx)   = 2*(u-1) + 2;  weights(2*idx)   = W_v;
end

% Skip sources with non-positive weight (same strict reading as doc version).
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
