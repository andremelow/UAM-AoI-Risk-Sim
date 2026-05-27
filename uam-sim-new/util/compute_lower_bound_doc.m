function [J_LB, info] = compute_lower_bound_doc(cfg)
% COMPUTE_LOWER_BOUND_DOC  Cauchy-Schwarz lower bound on the linear
%                          risk surrogate (Sec. 5/6 of analysis doc).
%
%  Implements the closed-form universal lower bound:
%
%    J_LB = (1/2K) [ sum_u sqrt(omega_u n_s / p_{u,s})
%                  + sum_u sqrt(omega_u n_v L / p_{u,v}) ]^2
%         + sum_u omega_u (n_s + n_v)
%
%  This is independent of the scheduling policy and serves as the
%  benchmark for the Max-Weight optimality ratio rho^MW = J^MW / J_LB.
%
%  Inputs:
%    cfg — validated config struct (must contain n_s, n_v, omega,
%          p_c2, p_vid, L_vid, dronesPerSlot)
%
%  Outputs:
%    J_LB — scalar lower bound on the linear risk surrogate
%    info — struct with intermediate quantities (Theta, q_LB vectors)

K   = cfg.dronesPerSlot;
L   = cfg.L_vid;
N   = cfg.numDrones;
n_s = cfg.n_s;
n_v = cfg.n_v;

omega   = cfg.omega(:);
p_s_vec = cfg.p_c2  * ones(N, 1);
p_v_vec = cfg.p_vid * ones(N, 1);

% Theta: sum of sqrt-terms (Cauchy-Schwarz envelope)
T_s = sum( sqrt(omega * n_s ./ p_s_vec) );
T_v = sum( sqrt(omega * n_v * L ./ p_v_vec) );
Theta = T_s + T_v;

% Closed-form LB
J_LB = Theta^2 / (2*K) + sum(omega * (n_s + n_v));

% Optimal LB throughput vector (Sec. 6, Step 2)
q_LB_s = K * sqrt(omega * n_s .* p_s_vec) / Theta;
q_LB_v = K * sqrt(omega * n_v * L .* p_v_vec) / Theta;

% Sanity: resource constraint must hold with equality
res = sum(q_LB_s ./ p_s_vec) + sum(q_LB_v ./ p_v_vec);

info = struct( ...
    'Theta',   Theta, ...
    'T_s',     T_s, ...
    'T_v',     T_v, ...
    'q_LB_s',  q_LB_s, ...
    'q_LB_v',  q_LB_v, ...
    'res_check', res, ...
    'K',       K, ...
    'L',       L, ...
    'N',       N, ...
    'n_s',     n_s, ...
    'n_v',     n_v);

if nargout < 1
    fprintf('[LB-doc] J_LB = %.6e\n', J_LB);
    fprintf('[LB-doc]   Theta = %.4e (T_s=%.4e + T_v=%.4e)\n', Theta, T_s, T_v);
    fprintf('[LB-doc]   resource check sum(q/p) = %.4f (should = K=%d)\n', res, K);
    fprintf('[LB-doc]   q_LB_s: min=%.4f max=%.4f mean=%.4f\n', ...
            min(q_LB_s), max(q_LB_s), mean(q_LB_s));
    fprintf('[LB-doc]   q_LB_v: min=%.4f max=%.4f mean=%.4f\n', ...
            min(q_LB_v), max(q_LB_v), mean(q_LB_v));
end
end
