function LB = compute_lower_bound(cfg, p_success_per_source)
% COMPUTE_LOWER_BOUND  Lower bound on EWSAoI (Kadota & Modiano, Algorithm 1).
%
%  Implements the KKT-based lower bound from:
%    "Optimizing Age of Information without Knowing the AoI" (Kadota 2023)
%    Theorem 2 / Algorithm 1.
%
%  Model: 2N sources (C2 + video per drone), single slot per packet (L=1),
%  reliable delivery (theta_i = 0), generate-at-will (lambda_i = 1).
%
%  Inputs:
%    cfg                   — scenario config struct
%    p_success_per_source  — (2N×1) empirical success prob per source
%                            If omitted: estimated from cfg.thresholdSNR
%                            (Gaussian approximation: p ≈ Q(snr_margin))
%
%  Output:
%    LB — scalar lower bound on EWSAoI (in slots)
%
%  Usage:
%    LB = compute_lower_bound(cfg);            % analytic p estimate
%    LB = compute_lower_bound(cfg, p_emp);     % from simulation CSV

N = cfg.numDrones;
n_sources = 2 * N;
K = cfg.dronesPerSlot;  % transmissions per slot

% --- Source weights alpha_i ---
% C2 sources weighted by w_unc (navigation safety)
% Video sources weighted by w_vid (coherence)
alpha = zeros(n_sources, 1);
for u = 1:N
    alpha(2*(u-1)+1) = cfg.w_unc;   % C2
    alpha(2*(u-1)+2) = cfg.w_vid;   % video
end

% --- Success probabilities p_i^S ---
if nargin < 2 || isempty(p_success_per_source)
    % Analytic estimate: assume all sources identical
    % p_S estimated from SNR margin (simplified Gaussian channel)
    % With hard threshold model: p_S = fraction of time SNR >= threshold
    % Use 0.9 as default (calibrate from actual run data for accuracy)
    p_S = 0.9 * ones(n_sources, 1);
    fprintf('[LB] Using default p_S=0.9. Pass empirical p_success for accuracy.\n');
else
    p_S = p_success_per_source(:);
    if numel(p_S) ~= n_sources
        error('p_success_per_source must have 2*numDrones = %d entries', n_sources);
    end
end

% Clip to avoid numerical issues
p_S = max(min(p_S, 1 - 1e-6), 1e-6);

% --- Generate-at-will: lambda_i = 1 (always fresh packet available) ---
% For video with L_vid > 1: effective lambda = 1/L_vid
% (source can generate a new frame only every L_vid successful video slots)
lambda = ones(n_sources, 1);
for u = 1:N
    lambda(2*(u-1)+2) = 1 / max(cfg.L_vid, 1);  % video source
end

% --- p_i^D = p_S * p_D (delivery prob); theta_i = 0 (no forwarding delay) ---
% In our model: p_D = 1 (gNB receives immediately), theta = 0
p_D = p_S;  % combined success = channel success

% --- Algorithm 1 (KKT solution) ---
% v_i = min(lambda_i, p_S_i * p_D_i)
v = min(lambda, p_S .* p_D);

% gamma_i = alpha_i * p_S_i * p_D_i / (2*N*v_i^2)
gamma_i = alpha .* (p_S .* p_D) ./ (2 * N * v.^2);

% Initial gamma
gamma_tilde_num = sum(sqrt(alpha .* p_S .* p_D));
gamma_tilde = (gamma_tilde_num)^2 / (2 * N * K^2);
gamma = max(gamma_tilde, max(gamma_i));

% Iterative KKT: decrease gamma until sum(q_i / p_i^S p_i^D) = K
tol = 1e-8;
for iter = 1:10000
    q = v .* min(1, sqrt(gamma_i ./ gamma));
    S = sum(q ./ (p_S .* p_D));
    if S >= K - tol, break; end
    gamma = gamma * 0.999;
end

q_LB = q;

% --- Lower bound (Theorem 2, theta_i = 0) ---
% LB = 1/(2N) * sum_i alpha_i * (1/q_LB_i + 1)
LB = sum(alpha .* (1 ./ q_LB + 1)) / (2 * N);

fprintf('[LB] Lower bound = %.4f slots  (gamma*=%.4e, K=%d, 2N=%d)\n', ...
    LB, gamma, K, n_sources);
fprintf('[LB] Per-source q_LB: C2=[%s]  Vid=[%s]\n', ...
    sprintf('%.3f ', q_LB(1:2:end)), sprintf('%.3f ', q_LB(2:2:end)));
end
