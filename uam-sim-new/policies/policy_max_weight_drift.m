function txSlots = policy_max_weight_drift(eligible, K_free, state, slot, cfg)
% POLICY_MAX_WEIGHT_DRIFT  Non-switching duration-aware Max-Weight (analysis Sec. 2).
%
%  Faithful implementation of the debt-augmented Max-Weight policy whose
%  optimality ratio is proved in the analysis (rho^MW <= 6 + sqrt(Psi)/(N LB)).
%  Ongoing video frames are forced to continue OUTSIDE this function: the
%  caller passes K_free = K - #(ongoing videos), and "eligible" already holds
%  only drones with a fresh frame ready to start (l_u = L) and r_u(t) = 0.
%
%  For each eligible drone u, two candidate sources compete:
%
%    Status (quadratic Lyapunov drift + debt):
%        W_{u,s} = 1/2 * a_{u,s} * p_{u,s} * (h_{u,s}^2 + 2 h_{u,s})
%                + V * p_{u,s} * x_{u,s}
%
%    Video-start (freshest-frame surrogate + debt):
%        W_{u,v} = 1/2 * a_{u,v} * (2 h_{u,v} + 1) + V * x_{u,v}
%
%    a_{u,s} = omega_u * beta_s,   a_{u,v} = omega_u * beta_v.
%
%  Differences vs. policy_max_weight_doc (intentional, to match the proof):
%    1. Status weight is QUADRATIC in h, not linear.
%    2. Video-start uses the freshest-frame surrogate h_{u,v} = h^(1) only.
%       The full window reduction D_start is NOT used: its historical terms
%       are not controllable by the current action (controllable-renewal
%       argument). The window AoI z_u is REPORTED in the cost, not used here.
%    3. No 1/L normalization, no -kappa*L penalty, no p_{u,v} on the video
%       start term. In the non-switching model the L-slot cost is carried by
%       forced continuation and the throughput-debt targets, not by the weight.
%
%  Source IDs:  src_C2(u) = 2*(u-1)+1,  src_vid(u) = 2*(u-1)+2
%
%  Required cfg fields:
%     omega(u)   priority weight of drone u
%     beta_s     status Lyapunov coefficient   (defaults to n_s)
%     beta_v     video  Lyapunov coefficient   (defaults to n_v)
%     p_c2       status success probability p_{u,s}
%     V          throughput-debt coefficient   (single V; replaces V_s, V_v)
%  Required state.dual(u) fields:
%     h1               status AoI h_{u,s}(t)   (legacy field name)
%     z_u              window AoI (fallback for video freshness before 1st delivery)
%     x_s, x_v         status / video throughput-debt queues
%     frame_gen_hist   generation slots of delivered frames (h^(1) = slot - hist(1))
%     n_delivered      number of frames delivered so far

% No hard SNR filter: p_c2 multiplies W_s, already discounting bad channels.
% Hard exclusion would starve persistently low-SNR drones permanently.

n   = numel(eligible);
src = zeros(1, 2*n);
w   = zeros(1, 2*n);

for idx = 1:n
    u  = eligible(idx);
    om = cfg.omega(u);

    a_s = om * cfg.beta_s;        % alpha_{u,s} = omega_u * beta_s
    a_v = om * cfg.beta_v;        % alpha_{u,v} = omega_u * beta_v

    h_s = state.dual(u).h1;       % status AoI h_{u,s}(t)
    x_s = state.dual(u).x_s;      % status throughput debt
    x_v = state.dual(u).x_v;      % video  throughput debt

    % --- Freshest delivered video AoI h_{u,v}(t) = h^(1) ---
    nd = state.dual(u).n_delivered;
    if nd >= 1 && ~isnan(state.dual(u).frame_gen_hist(1))
        h_v = slot - state.dual(u).frame_gen_hist(1);
    else
        % No frame delivered yet: fall back to the window proxy.
        h_v = state.dual(u).z_u;
    end

    % --- Status weight: quadratic drift + debt (Sec. 2.16 / 2.24) ---
    W_s = 0.5 * a_s * cfg.p_c2 * (h_s^2 + 2*h_s) + cfg.V * cfg.p_c2 * x_s;

    % --- Video-start weight: freshest-frame surrogate + debt (Sec. 2.10) ---
    W_v = 0.5 * a_v * (2*h_v + 1) + cfg.V * x_v;

    src(2*idx-1) = 2*(u-1) + 1;  w(2*idx-1) = W_s;
    src(2*idx)   = 2*(u-1) + 2;  w(2*idx)   = W_v;
end

% Select the K_free largest POSITIVE weights (Sec. 2.11 / 2.17), ties arbitrary.
pos = find(w > 0);
if isempty(pos)
    txSlots = [];
    return;
end

[~, order] = sort(w(pos), 'descend');
take    = min(K_free, numel(order));
txSlots = src(pos(order(1:take)));
end
