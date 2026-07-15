function test_ab_position_engine()
% TEST_AB_POSITION_ENGINE  A/B: uavScenario engine vs analytic positions.
%
%  Part 1 — position parity: advances ONE uavScenario and, at every tick,
%    calls step_read_positions twice (positionEngine='uav-toolbox' vs
%    'analytic') on the same time/slot, comparing state.activePos. This
%    exercises the real analytic code path against read(uavPlatform).
%
%  Part 2 — full-sim A/B: runs run_sim twice on an RC14-like small cell
%    (same seed) and compares runtime + key metrics. Metrics are expected
%    to be statistically close, not bit-identical: sub-metre position
%    differences perturb SNR, which can reorder scheduling decisions and
%    hence the Bernoulli rand() stream.
%
%  Known, accepted difference (verified 2026-07): horizontal positions are
%  EXACT (0.0000 m); altitude deviates only on the takeoff/landing ramps
%  (max ~8.8 m mid-ramp for a 10→60 m climb over 500 m), because
%  waypointTrajectory interpolates the ramp as a cubic S-curve while the
%  analytic mode uses the linear altitude_profile_ned — which is the
%  canonical profile of the risk model (A* mode and phase-aware weights
%  already use it). Cruise segment and ramp endpoints are exact.

setup_paths();

% ── RC14-like small cell: single 2 km lane, GOP video, pads 10 m ──────
cfg = base_config_capacity();
cfg.w_unc = 1/3;  cfg.w_map = 1/3;  cfg.w_vid = 1/3;
cfg.videoModel = 'gop';
cfg.fps  = 30;   cfg.GOP  = 15;
cfg.L_I  = 30;   cfg.L_P  = 3;    cfg.L_vid = 5;   cfg.L_c2 = 1;
cfg.c2_hz = 2;   cfg.w_coh = 5;   cfg.L_nom = 0.01;
cfg.p_c2  = 0.80;  cfg.p_vid = 0.80;  cfg.rho_0 = 0.01;
cfg.altProfile  = 'takeoff_landing';
cfg.takeoffDist = 500;  cfg.cruiseAlt = 60;  cfg.padAlt = 10;
cfg.use_normalization = true;
cfg.corridorWidth     = 100;
cfg.rho_op            = 5000;
cfg.schedulingPolicy  = 'mw-sw-v9';
cfg.headless  = true;
cfg.csvExport = false;

C = 8;  T_steady = 150;                    % small but non-trivial cell
spacing = cfg.flightTime / C;
cfg.routes(1).numDrones = C + floor(T_steady / spacing);
cfg.corridorCapacity = C;
cfg.steady_duration  = T_steady;
cfg.dronesPerSlot    = 2;
cfg.maxStartDelay    = cfg.minStartDelay + ...
    (cfg.routes(1).numDrones - 1) * spacing + 10;

fprintf('\n════ A/B position engine — N=%d drones, K=%d, C=%d ════\n\n', ...
        cfg.routes(1).numDrones, cfg.dronesPerSlot, C);

% ════ Part 1 — position parity on identical clock ════════════════════
cfgE = cfg;  cfgE.positionEngine = 'uav-toolbox';
cfgA = cfg;  cfgA.positionEngine = 'analytic';
cfgE = validate_uam_config(cfgE);
cfgA = validate_uam_config(cfgA);

[scene, drones, infra] = init_scenario(cfgE);
infra.precomputed_trajectory = {};
stateE = init_state(cfgE, infra);
stateA = init_state(cfgA, infra);

maxdev   = zeros(1, 3);      % [North, East, Down] max abs deviation [m]
mismatch = 0;                % ticks where active sets differ
nticks   = 0;
while advance(scene)
    time = scene.CurrentTime;
    slot = round(time * cfgE.updateRate);

    stateE = step_read_positions(stateE, drones, infra, time, slot, cfgE);
    stateA = step_read_positions(stateA, drones, infra, time, slot, cfgA);

    if slot > max(stateE.endSlots) && isempty(stateE.activeDrones), break; end
    nticks = nticks + 1;

    if ~isequal(stateE.activeDrones, stateA.activeDrones)
        mismatch = mismatch + 1;
        act = intersect(stateE.activeDrones, stateA.activeDrones);
    else
        act = stateE.activeDrones;
    end
    if ~isempty(act)
        dev    = abs(stateE.activePos(act,:) - stateA.activePos(act,:));
        maxdev = max(maxdev, max(dev, [], 1));
    end
end

fprintf('Part 1 — position parity over %d ticks:\n', nticks);
fprintf('  max |dNorth| = %.4f m   max |dEast| = %.4f m   max |dDown| = %.4f m\n', ...
        maxdev(1), maxdev(2), maxdev(3));
fprintf('  ticks with active-set mismatch: %d\n\n', mismatch);

% ════ Part 2 — full-sim runtime + metrics ═════════════════════════════
tic;  resE = run_sim(cfgE);  tE = toc;
tic;  resA = run_sim(cfgA);  tA = toc;

mE = ab_metrics(resE);
mA = ab_metrics(resA);

fprintf('\nPart 2 — full sim (policy %s, same seed):\n', cfg.schedulingPolicy);
fprintf('  %-22s %12s %12s %10s\n', 'metric', 'engine', 'analytic', 'diff %%');
rows = {'runtime [s]', tE, tA
        'mean h1 [slots]',   mE.h1,      mA.h1
        'mean h2 [slots]',   mE.h2,      mA.h2
        'Rsys_emp',          mE.rsys,    mA.rsys
        'peak r_sys',        mE.peak,    mA.peak
        'Jhat_emp',          mE.jhat,    mA.jhat
        'sum q_emp_v [1/slot]', mE.frames, mA.frames
        'slots C2 ok',       mE.c2ok,    mA.c2ok};
for r = 1:size(rows, 1)
    a = rows{r,2};  b = rows{r,3};
    fprintf('  %-22s %12.4f %12.4f %+9.2f%%\n', rows{r,1}, a, b, 100*(b-a)/max(abs(a),eps));
end
fprintf('\n  speedup (engine/analytic): %.2fx\n', tE / tA);

% PASS criteria: horizontal must be numerically exact and the active sets
% identical. Vertical tolerance covers the known cubic-vs-linear ramp
% smoothing (see header): bounded by a quarter of the climb height.
vert_tol = max(1, (cfg.cruiseAlt - cfg.padAlt) / 4);
ok = all(maxdev(1:2) < 0.01) && maxdev(3) < vert_tol && mismatch == 0;
if ok
    fprintf(['\nRESULT: PASS — horizontal exact, vertical dev %.2f m < %.2f m ' ...
             '(ramp smoothing, see header)\n'], maxdev(3), vert_tol);
else
    fprintf('\nRESULT: CHECK — position deviation or active-set mismatch above threshold\n');
end
end

function m = ab_metrics(res)
N = res.numDrones;
h1m = zeros(N,1);  h2m = zeros(N,1);
for u = 1:N
    h1m(u) = mean(res.h1Val{u});
    h2m(u) = mean(res.h2Val{u});
end
m.h1     = mean(h1m);
m.h2     = mean(h2m);
m.rsys   = res.Rsys_emp;
m.peak   = res.peak_r_sys;
m.jhat   = res.Jhat_emp;
m.frames = sum(arrayfun(@(u) res.q_emp_v(u), 1:N));
m.c2ok   = sum(res.slotsC2);
end
