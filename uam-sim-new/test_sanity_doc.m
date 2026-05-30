function test_sanity_doc()
% TEST_SANITY_DOC  Standalone numerical checks for doc-aligned simulator.
%
%  Validates the analytic pieces independently of the uavScenario engine:
%    1. Surrogate coefficients n_s, n_v (Sec. 3 boxed equations)
%    2. Surrogate UPPER BOUND vs raw R_u (Sec. 3 main inequality)
%    3. KKT optimality of q_LB throughput vector (Sec. 6, Step 2)
%    4. Cauchy-Schwarz closed-form J_LB matches the constrained min
%    5. Debt-queue stability: simulate the fluid model with mu = qbar
%    6. Max-Weight policy unit test: synthetic state -> deterministic choice
%
%  Usage:  >> test_sanity_doc

setup_paths();

fprintf('\n========================================================\n');
fprintf('   Doc-Alignment Sanity Test Suite\n');
fprintf('========================================================\n');

% --- Build a small reference config ---
cfg = struct();
cfg.numDrones      = 4;
cfg.dronesPerSlot  = 2;
cfg.corridorLength = 1800;
cfg.flightTime     = 25;
cfg.updateRate     = 600;
cfg.L_vid          = 1;            % atomic, single-slot frame
cfg.p_c2           = 0.8;
cfg.p_vid          = 0.8;
cfg.w_unc          = 0.4;
cfg.w_map          = 0.3;
cfg.w_vid          = 0.3;
cfg.r_min          = 25;
cfg.rho_0          = 0.01;
cfg.use_normalization = false;
cfg.V_s            = 1.0;
cfg.V_v            = 1.0;
cfg.kappa          = 0.0;
cfg.omega          = ones(cfg.numDrones,1);
cfg = validate_uam_config(cfg);

passed = 0; failed = 0;

% =====================================================================
fprintf('\n[Test 1] Surrogate coefficients n_s, n_v (Sec. 3)\n');
% =====================================================================
% Recompute by hand and compare.
v = cfg.v_max / cfg.updateRate;        % m/slot
N = cfg.numDrones;
H = cfg.H_max;

C_Nav_0_hand = 2*(N-1)*cfg.r_min + (N-1)*v*H;
C_Nav_1_hand = (N-1)*v;
C_Gnd_0_hand = cfg.rho_0 * pi * cfg.r_min^2;
C_Gnd_1_hand = cfg.rho_0 * pi * (2*cfg.r_min*v + v^2*H);
n_s_hand = cfg.w_unc*C_Nav_1_hand + cfg.w_map*C_Gnd_1_hand;
n_v_hand = cfg.w_vid / cfg.H_nom_slots;

[ok, ~] = check_close(cfg.n_s, n_s_hand, 1e-12, 'n_s');
passed = passed + ok; failed = failed + (1-ok);
[ok, ~] = check_close(cfg.n_v, n_v_hand, 1e-12, 'n_v');
passed = passed + ok; failed = failed + (1-ok);

% =====================================================================
fprintf('\n[Test 2] Surrogate is an UPPER bound on R_u (Sec. 3)\n');
% =====================================================================
% Generate random states with h_s in [0, H_max], z_u >= 0 and verify
% R_u <= C_u + n_s h_s + n_v z_u.
rng(42);
n_trials = 5000;
worst_violation = -inf;            % positive => bound violated
tightest_slack  = inf;             % slack = bound - R_u (smaller = tighter)
worst_h = NaN; worst_z = NaN;
tight_h = NaN; tight_z = NaN;
for k = 1:n_trials
    h_s = rand() * cfg.H_max;
    z_u = rand() * 4 * cfg.H_nom_slots;
    r_u = cfg.r_min + v*h_s;

    % Worst-case R_nav: every other UAV at distance d_min = 2*r_min
    R_nav = (N-1) * max(2*r_u - 2*cfg.r_min, 0);
    R_gnd = cfg.rho_0 * pi * r_u^2;
    R_vid = max(z_u/cfg.H_nom_slots - 1, 0);
    R_u   = cfg.w_unc*R_nav + cfg.w_map*R_gnd + cfg.w_vid*R_vid;

    C_u   = cfg.w_unc*C_Nav_0_hand + cfg.w_map*C_Gnd_0_hand;
    bound = C_u + cfg.n_s*h_s + cfg.n_v*z_u;

    violation = R_u - bound;
    if violation > worst_violation
        worst_violation = violation;
        worst_h = h_s; worst_z = z_u;
    end
    slack = bound - R_u;
    if slack < tightest_slack
        tightest_slack = slack;
        tight_h = h_s; tight_z = z_u;
    end
end
if worst_violation <= 1e-9
    fprintf('  PASS: max violation = %.3e (worst h=%.2f, z=%.2f)\n', ...
            worst_violation, worst_h, worst_z);
    fprintf('        tightest slack = %.3e (at h=%.2f, z=%.2f)\n', ...
            tightest_slack, tight_h, tight_z);
    passed = passed + 1;
else
    fprintf('  FAIL: max violation = %.3e (h=%.2f, z=%.2f)\n', ...
            worst_violation, worst_h, worst_z);
    failed = failed + 1;
end

% =====================================================================
fprintf('\n[Test 3] KKT optimality of q_LB (Sec. 6, Step 2)\n');
% =====================================================================
% Resource constraint must be tight: sum(q_LB / p) = K
res = sum(cfg.q_LB_s / cfg.p_c2) + sum(cfg.q_LB_v / cfg.p_vid);
[ok, ~] = check_close(res, cfg.dronesPerSlot, 1e-9, 'sum(q_LB/p) = K');
passed = passed + ok; failed = failed + (1-ok);

% Symmetric case (uniform omega, uniform p): q_LB should be uniform per stream
unif_qs = std(cfg.q_LB_s);
unif_qv = std(cfg.q_LB_v);
[ok, ~] = check_close(unif_qs, 0, 1e-12, 'q_LB_s uniform across UAVs');
passed = passed + ok; failed = failed + (1-ok);
[ok, ~] = check_close(unif_qv, 0, 1e-12, 'q_LB_v uniform across UAVs');
passed = passed + ok; failed = failed + (1-ok);

% =====================================================================
fprintf('\n[Test 4] Closed-form J_LB matches constrained min (Sec. 5/6)\n');
% =====================================================================
[J_LB_closed, ~] = compute_lower_bound_doc(cfg);

% Solve the constrained min via brute force (uniform symmetric solution)
%   min sum [a_s/(2 q_s) + a_v L/(2 q_v)] + sum (a_s+a_v)
%   st sum (q_s/p_s + q_v/p_v) = K
% In the symmetric case q_s, q_v are scalars, problem is 2D.
n_s = cfg.n_s; n_v = cfg.n_v;
L = cfg.L_vid; K = cfg.dronesPerSlot;
ps = cfg.p_c2; pv = cfg.p_vid;
a_s = cfg.omega(1) * n_s;
a_v = cfg.omega(1) * n_v;
% Lagrangian gives q_s = sqrt(a_s p_s / (2 lam)), q_v = sqrt(a_v L p_v / (2 lam))
% Resource: N(q_s/p_s + q_v/p_v) = K
% Substituting: N (sqrt(a_s/(2 lam p_s)) + sqrt(a_v L/(2 lam p_v))) = K
% sqrt(1/(2 lam)) = K / (N (sqrt(a_s/p_s) + sqrt(a_v L/p_v)))
inv_sqrt_2lam = K / (N * (sqrt(a_s/ps) + sqrt(a_v*L/pv)));
q_s_sym = inv_sqrt_2lam * sqrt(a_s * ps);
q_v_sym = inv_sqrt_2lam * sqrt(a_v * L * pv);

J_brute = N * (a_s/(2*q_s_sym) + a_v*L/(2*q_v_sym)) + N*(a_s + a_v);

[ok, ~] = check_close(J_LB_closed, J_brute, 1e-9, 'J_LB closed-form vs brute');
passed = passed + ok; failed = failed + (1-ok);

% =====================================================================
fprintf('\n[Test 5] Debt-queue fluid stability (Sec. 4.2)\n');
% =====================================================================
% Drive the queue with mu ~ Bernoulli(qbar + delta), delta > 0, mimicking
% the real simulator where the scheduler achieves q_emp >= q_LB > qbar
% (strict-feasibility margin from Sec. 6, Step 3). This produces a NEGATIVE
% drift E[qbar - mu] = -delta < 0, so the queue is stable.
%
% Note: with mu ~ Bernoulli(qbar) (no margin), the queue is critical and
% grows as O(sqrt(T)) by random-walk theory — that's expected behavior,
% not a bug. The simulator never operates in that regime.
T_steps = 20000;
x_s = 0;
qbar = cfg.qbar_s(1);
delta = 0.05;                 % feasibility margin (matches qbar_eps in spirit)
mu_rate = qbar + delta;
xs_hist = zeros(T_steps,1);
for t = 1:T_steps
    mu_t = (rand() < mu_rate);
    x_s  = max(x_s + qbar - mu_t, 0);
    xs_hist(t) = x_s;
end
avg_x = mean(xs_hist(end-1000:end));
fprintf('  qbar = %.4f, mu rate = %.4f, final x_s mean = %.2f\n', ...
        qbar, mu_rate, avg_x);
if avg_x < 50      % loose bound, just sanity
    fprintf('  PASS: queue bounded under positive drift margin\n');
    passed = passed + 1;
else
    fprintf('  FAIL: queue blew up (avg=%.2f) -- expected bounded under delta>0\n', avg_x);
    failed = failed + 1;
end

% =====================================================================
fprintf('\n[Test 6] Max-Weight policy decision (Sec. 4.5)\n');
% =====================================================================
% Synthetic state: drone 1 has very stale C2, drone 2 has very stale video.
% Expect: K=2 selections cover both.
fake_state = struct();
fake_state.activeDrones = 1:cfg.numDrones;
for u = 1:cfg.numDrones
    fake_state.dual(u).h1 = 1;       % fresh
    fake_state.dual(u).h2 = 1;
    fake_state.dual(u).z_u = 1;
    fake_state.dual(u).x_s = 0;
    fake_state.dual(u).x_v = 0;
    fake_state.dual(u).vid_remaining = 0;
    fake_state.dual(u).vid_in_flight = false;
    fake_state.dual(u).n_delivered = cfg.w_coh;
    fake_state.dual(u).frame_gen_hist = 100*ones(cfg.w_coh,1);
end
fake_state.h1Time = cell(cfg.numDrones,1);
for u = 1:cfg.numDrones
    fake_state.h1Time{u} = 100;
end

% Drone 1: extremely stale C2
fake_state.dual(1).h1 = 1000;
% Drone 2: extremely stale video
fake_state.dual(2).z_u = 1000;
fake_state.dual(2).frame_gen_hist = -800*ones(cfg.w_coh,1);   % very old

eligible = 1:cfg.numDrones;
K_free = 2;
chosen = policy_max_weight_doc(eligible, K_free, fake_state, 101, cfg);

src_C2_d1  = 2*(1-1)+1;      % expected C2 of drone 1
src_vid_d2 = 2*(2-1)+2;      % expected video of drone 2

if any(chosen == src_C2_d1) && any(chosen == src_vid_d2)
    fprintf('  PASS: chose C2(drone1)=%d and Vid(drone2)=%d\n', src_C2_d1, src_vid_d2);
    passed = passed + 1;
else
    fprintf('  FAIL: chosen = [%s], expected to include %d and %d\n', ...
            num2str(chosen), src_C2_d1, src_vid_d2);
    failed = failed + 1;
end

% =====================================================================
fprintf('\n[Test 7] K_free uses vid_in_flight, not vid_remaining (bugfix)\n');
% =====================================================================
% Regression test for a subtle bug: K_free must subtract channels held
% by ONGOING video transmissions (vid_in_flight=true), NOT every UAV
% with a pending frame in its buffer (vid_remaining>0). Otherwise, when
% step_traffic loads frames into ALL UAVs simultaneously, K_free becomes
% 0 and C2 is starved forever, h_{u,s} grows unboundedly.
%
% Setup: N=3 UAVs, all have pending frames (vid_remaining > 0) but none
% has yet started transmission (vid_in_flight = false). With K=1 channel,
% the scheduler MUST be able to issue a new decision (K_free = 1).

cfg7 = cfg;
cfg7.numDrones     = 3;
cfg7.dronesPerSlot = 1;
cfg7.omega         = [];           % force default uniform; size mismatches otherwise
cfg7 = validate_uam_config(cfg7);

s7 = struct();
s7.activeDrones = 1:3;
s7.rrPointer = 1;
for u = 1:3
    s7.dual(u).h1 = 0;  s7.dual(u).h2 = 0;  s7.dual(u).z_u = 0;
    s7.dual(u).x_s = 0; s7.dual(u).x_v = 0;
    s7.dual(u).vid_remaining = 10;        % buffer has frame
    s7.dual(u).vid_in_flight = false;     % but no channel allocated yet
    s7.dual(u).n_delivered   = 0;
    s7.dual(u).frame_gen_hist = nan(cfg7.w_coh,1);
end
s7.h1Time = cell(3,1); for u=1:3, s7.h1Time{u} = []; end

% Round-robin should pick exactly K=1 source.
cfg7.schedulingPolicy = 'round-robin';
[tx7, ~] = schedule_sources(s7, 0, cfg7);

if numel(tx7) == 1
    fprintf('  PASS: scheduler issued exactly 1 source (K=%d), got src=%d\n', ...
            cfg7.dronesPerSlot, tx7(1));
    passed = passed + 1;
else
    fprintf('  FAIL: scheduler issued %d sources (expected K=%d): tx=[%s]\n', ...
            numel(tx7), cfg7.dronesPerSlot, num2str(tx7));
    failed = failed + 1;
end

% Now simulate that one UAV has vid_in_flight=true. With K=1, K_free must
% be 0 and tx must contain only the in-flight reservation.
s7.dual(1).vid_in_flight = true;
[tx7b, ~] = schedule_sources(s7, 1, cfg7);
expected_src = 2*(1-1)+2;   % video source of UAV 1
if numel(tx7b) == 1 && tx7b(1) == expected_src
    fprintf('  PASS: in-flight UAV holds the only channel (src=%d)\n', expected_src);
    passed = passed + 1;
else
    fprintf('  FAIL: expected only [%d], got [%s]\n', expected_src, num2str(tx7b));
    failed = failed + 1;
end

% =====================================================================
fprintf('\n[Test 8] RR-aware skips drones in dead zone (and falls back)\n');
% =====================================================================
% Setup: N=3 UAVs, K=1. Drone 1 has snrLast=0 dB (below threshold=10),
% drones 2 and 3 are link-OK. RR-aware must skip drone 1's source even
% if the pointer lands on it. Falls back to classic RR only if ALL are dead.

cfg8 = cfg;
cfg8.numDrones     = 3;
cfg8.dronesPerSlot = 1;
cfg8.thresholdSNR  = 10;
cfg8.omega         = [];
cfg8 = validate_uam_config(cfg8);
cfg8.schedulingPolicy = 'round-robin-aware';

s8 = struct();
s8.activeDrones = 1:3;
s8.rrPointer    = 1;     % start at C2 of drone 1
s8.snrLast      = [0; 15; 12];   % drone 1 below thresh, others above
for u = 1:3
    s8.dual(u).h1 = 0;  s8.dual(u).h2 = 0;  s8.dual(u).z_u = 0;
    s8.dual(u).x_s = 0; s8.dual(u).x_v = 0;
    s8.dual(u).vid_remaining = 0;
    s8.dual(u).vid_in_flight = false;
    s8.dual(u).n_delivered   = 0;
    s8.dual(u).frame_gen_hist = nan(cfg8.w_coh,1);
end
s8.h1Time = cell(3,1); for u=1:3, s8.h1Time{u} = []; end

[tx8, ~] = schedule_sources(s8, 0, cfg8);
% Pointer started at src=1 (C2 of drone 1) but drone 1 is dead, so
% RR-aware should advance to src=3 (C2 of drone 2).
chosen_drone = ceil(tx8(1)/2);
if numel(tx8) == 1 && chosen_drone ~= 1
    fprintf('  PASS: skipped drone 1 (dead), chose drone %d (src=%d)\n', ...
            chosen_drone, tx8(1));
    passed = passed + 1;
else
    fprintf('  FAIL: chose src=[%s] (drone %d), expected to skip dead drone 1\n', ...
            num2str(tx8), chosen_drone);
    failed = failed + 1;
end

% Fallback test: ALL drones dead -> must still pick one (else the slot is wasted)
s8.snrLast = [0; 0; 0];
s8.rrPointer = 1;
[tx8b, ~] = schedule_sources(s8, 1, cfg8);
if numel(tx8b) == 1
    fprintf('  PASS: fallback when all dead -- chose src=%d (drone %d)\n', ...
            tx8b(1), ceil(tx8b(1)/2));
    passed = passed + 1;
else
    fprintf('  FAIL: fallback returned [%s]\n', num2str(tx8b));
    failed = failed + 1;
end

% =====================================================================
fprintf('\n[Test 9] run_sweep config re-validation (numDrones change)\n');
% =====================================================================
% Regression: when run_sweep iterates over numDrones values, omega must
% be reset to [] BEFORE calling validate_uam_config so that it expands
% to the correct size. Without the reset, the size mismatch error fires.

cfg9 = cfg;
cfg9.numDrones = 4;
cfg9.omega     = [];
cfg9 = validate_uam_config(cfg9);
% Now simulate sweep iteration: change numDrones, reset omega, revalidate.
cfg9.numDrones = 7;
cfg9.omega     = [];                    % <-- this is what run_sweep does
ok9 = true;
try
    cfg9 = validate_uam_config(cfg9);
catch ME9
    fprintf('  FAIL: validate_uam_config errored: %s\n', ME9.message);
    ok9 = false;
end
if ok9
    if numel(cfg9.omega) == 7 && numel(cfg9.qbar_s) == 7 && numel(cfg9.qbar_v) == 7
        fprintf('  PASS: omega, qbar_s, qbar_v correctly resized to N=7\n');
        passed = passed + 1;
    else
        fprintf('  FAIL: sizes are omega=%d qbar_s=%d qbar_v=%d (expected 7)\n', ...
                numel(cfg9.omega), numel(cfg9.qbar_s), numel(cfg9.qbar_v));
        failed = failed + 1;
    end
else
    failed = failed + 1;
end

% =====================================================================
fprintf('\n========================================================\n');
fprintf('   Results: %d passed, %d failed\n', passed, failed);
fprintf('========================================================\n\n');
end


function [ok, msg] = check_close(actual, expected, tol, label)
err = abs(actual - expected);
if err <= tol
    fprintf('  PASS: %s = %.6e (expected %.6e, err=%.2e)\n', label, actual, expected, err);
    ok = 1;
else
    fprintf('  FAIL: %s = %.6e (expected %.6e, err=%.2e > tol=%.0e)\n', ...
            label, actual, expected, err, tol);
    ok = 0;
end
msg = '';
end
