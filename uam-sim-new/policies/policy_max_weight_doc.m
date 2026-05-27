function txSlots = policy_max_weight_doc(eligible, K_free, state, slot, cfg)
% POLICY_MAX_WEIGHT_DOC  Doc-aligned Max-Weight scheduling (Sec. 4.5).
%
%  For each eligible drone u (with r_u(t)=0), compute two scheduling weights:
%
%    W_{u,s}(t)        = omega_u p_{u,s} ( n_s h_{u,s}(t) + V_s x_{u,s}(t) )
%
%    W_{u,v}^start(t)  = omega_u [ (p_{u,v} n_v / L) D_u^start(t)
%                                + p_{u,v} V_v x_{u,v}(t)
%                                - kappa L ]
%
%  where
%
%    D_u^start(t) = z_u(t) + L
%                 - alpha_1 L
%                 - sum_{i=2}^{w} alpha_i (h_{u,v}^{(i-1)}(t) + L)
%
%  is the explicit L-slot reduction in z_u from completing a freshly
%  started frame.
%
%  Then select the K_free sources with largest weights, breaking ties
%  arbitrarily. Source IDs:
%    src_C2(u)  = 2*(u-1) + 1
%    src_vid(u) = 2*(u-1) + 2

n = numel(eligible);
sources = zeros(1, 2*n);
weights = zeros(1, 2*n);

L     = cfg.L_vid;
alpha = cfg.alpha_w(:);
w     = cfg.w_coh;

for idx = 1:n
    u   = eligible(idx);
    om  = cfg.omega(u);
    h_s = state.dual(u).h1;
    z_u = state.dual(u).z_u;
    x_s = state.dual(u).x_s;
    x_v = state.dual(u).x_v;

    % --- Status weight W_{u,s} ---
    W_s = om * cfg.p_c2 * (cfg.n_s * h_s + cfg.V_s * x_s);

    % --- D_u^start: future L-slot z_u reduction from a fresh frame ---
    %  D_u^start = z_u + L - alpha_1 L - sum_{i>=2} alpha_i (h_{u,v}^{(i-1)} + L)
    %
    %  h_{u,v}^{(i)} = slot - frame_gen_hist(i)  (when available)
    nd = state.dual(u).n_delivered;
    D_start_sum = 0;
    if nd >= 1
        for ii = 2:w
            if ii-1 <= nd && ~isnan(state.dual(u).frame_gen_hist(ii-1))
                h_im1 = slot - state.dual(u).frame_gen_hist(ii-1);
            else
                % Window not yet full: treat missing slot as having h=z_u (worst)
                h_im1 = z_u;
            end
            D_start_sum = D_start_sum + alpha(ii) * (h_im1 + L);
        end
    else
        % No frames delivered yet — approximate via z_u
        for ii = 2:w
            D_start_sum = D_start_sum + alpha(ii) * (z_u + L);
        end
    end
    D_start = z_u + L - alpha(1)*L - D_start_sum;

    % --- Video-start weight W_{u,v}^start ---
    W_v = om * ( (cfg.p_vid * cfg.n_v / L) * D_start ...
               + cfg.p_vid * cfg.V_v * x_v ...
               - cfg.kappa * L );

    sources(2*idx-1) = 2*(u-1) + 1;  weights(2*idx-1) = W_s;
    sources(2*idx)   = 2*(u-1) + 2;  weights(2*idx)   = W_v;
end

% Negative weights mean scheduling that source would INCREASE the drift
% upper bound; if all top candidates are negative we still fill the K_free
% budget (consistent with maximizing the drift-reduction sum), but a stricter
% reading of the doc is to skip negatives. We follow the strict reading.
candidates = find(weights > 0);
if isempty(candidates)
    txSlots = [];
    return;
end

[~, order] = sort(weights(candidates), 'descend');
take = min(K_free, numel(order));
txSlots = sources(candidates(order(1:take)));
end
