function txSlots = policy_mw_sw_doc_g_u(eligible, K_free, state, slot, cfg)
% POLICY_MW_SW_DOC_G_U  Switching Max-Weight, doc-faithful 3-case video weight.
%
%  Faithful transcription of the SWITCHING Max-Weight policy in the model
%  doc ("Switching System Model"). Unlike policy_mw_sw_otimo /
%  policy_max_weight_sw — which collapse the video source into a single
%  coherence-window proxy z_u and emit ONE video weight per slot — this
%  policy reconstructs the doc's per-frame state and emits the three distinct
%  video weights (start / continuation / completion), built from the system
%  time g_u of the in-service video frame.
%
%  State mapping (doc symbol -> code field):
%    h_{u,s}(t)  status AoI                  -> state.dual(u).h1
%    h_{u,v}(t)  freshest delivered video AoI -> state.dual(u).h2
%    ell_u(t)    remaining packets of frame   -> state.dual(u).vid_remaining
%    g_u(t)      system time of in-service frame
%                = slot - state.dual(u).vid_gen_slot   (0 when no frame loaded)
%
%  Derived proxies (doc):
%    A_u = h_{u,v} - g_u     (video waiting-time proxy)
%    B_u = g_u + ell_u       (optimistic delivery-age proxy)
%
%  Coefficients (doc symbol -> cfg). Status and video are kept on the same
%  surrogate scale (n_s, n_v) used by policy_mw_sw_otimo, so the status/video
%  balance matches its sibling policies. Only gamma_v is overridable, since it
%  is the one doc coefficient with no existing config home:
%    eta_s   status Lyapunov coeff   = cfg.n_s
%    beta_v  video waiting coeff     = cfg.n_v
%    gamma_v video completion coeff  = cfg.gamma_v (default beta_v / L_vid)
%  NOTE: cfg.eta_s exists in the config but is the risk-gradient coefficient
%  (a different scale); it is intentionally NOT used here.
%
%  Status weight (doc):
%    W_{u,s} = 1/2 * omega_u * eta_s * p_s * (h_s^2 + 2 h_s)
%
%  Video weight (doc, 3 cases on the remaining-packet count ell_u):
%    ready to start  (ell_u = Lf):       % doc ell=0 case
%       W_v = 1/2 omega_u p_v [ beta_v (2 A_u + 1) - gamma_v Lf^2 ]
%    in progress     (1 < ell_u < Lf):   % doc ell>1 case
%       W_v = 1/2 omega_u p_v gamma_v (2 B_u + 1)
%    last packet     (ell_u = 1):        % doc ell=1 case
%       W_v = 1/2 omega_u p_v [ gamma_v (B_u+1)^2 + beta_v (A_u^2 - (g_u+1)^2) ]
%  where Lf = state.dual(u).vid_L is the loaded frame's length in packets.
%
%  Switching: schedule_sources clears all channel reservations every slot, so
%  `eligible` includes drones holding a partial frame. This policy re-evaluates
%  every source each slot and selects the K_free largest POSITIVE weights, at
%  most one source per drone (one radio per drone).

p_s = cfg.p_c2;
p_v = cfg.p_vid;

eta_s  = cfg.n_s;   % status surrogate coeff (same scale as policy_mw_sw_otimo)
beta_v = cfg.n_v;   % video waiting coeff
if isfield(cfg, 'gamma_v') && ~isempty(cfg.gamma_v)
    gamma_v = cfg.gamma_v;
else
    gamma_v = beta_v / cfg.L_vid;   % completion coeff (default)
end

n       = numel(eligible);
sources = zeros(1, 2*n);
weights = zeros(1, 2*n);

for idx = 1:n
    u   = eligible(idx);
    om  = cfg.omega(u);
    h_s = state.dual(u).h1;   % status AoI  h_{u,s}(t)
    h_v = state.dual(u).h2;   % freshest delivered video AoI  h_{u,v}(t)
    ell = state.dual(u).vid_remaining;   % remaining packets  ell_u(t)

    % --- Status weight (doc) ---
    W_s = 0.5 * om * eta_s * p_s * (h_s^2 + 2*h_s);

    % --- Video weight: reconstruct g_u, A_u, B_u and pick the doc case ---
    if ell > 0
        Lf  = state.dual(u).vid_L;                    % this frame's length
        g_u = max(slot - state.dual(u).vid_gen_slot, 0);  % system time of frame
        A_u = h_v - g_u;                              % waiting-time proxy
        B_u = g_u + ell;                              % optimistic delivery age

        if ell >= Lf
            % START (doc ell=0): no packet sent yet, frame begins transmission
            W_v = 0.5 * om * p_v * (beta_v * (2*A_u + 1) - gamma_v * Lf^2);
        elseif ell == 1
            % COMPLETION (doc ell=1): last packet delivers the frame
            W_v = 0.5 * om * p_v * ( gamma_v * (B_u + 1)^2 ...
                                   + beta_v * (A_u^2 - (g_u + 1)^2) );
        else
            % CONTINUATION (doc ell>1): mid-frame packet
            W_v = 0.5 * om * p_v * gamma_v * (2*B_u + 1);
        end
    else
        W_v = 0;   % no frame loaded -> no video candidate this slot
    end

    sources(2*idx-1) = 2*(u-1) + 1;  weights(2*idx-1) = W_s;
    sources(2*idx)   = 2*(u-1) + 2;  weights(2*idx)   = W_v;
end

% Select the K_free largest POSITIVE weights, one source per drone (one radio).
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
