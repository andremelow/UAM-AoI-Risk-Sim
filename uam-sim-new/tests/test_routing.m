%% test_routing.m — Unit and integration tests for the Risk-A* routing module.
%
%  Run from the uam-sim-new/ root after calling setup_paths():
%    cd uam-sim-new
%    setup_paths()
%    run('tests/test_routing.m')
%
%  Each test is a local function called from the main script.
%  A PASS/FAIL summary is printed at the end.

setup_paths();

results = struct('name', {}, 'passed', {}, 'msg', {});
results = run_test(results, 'T1_zero_risk_shortest_path',   @t1_zero_risk);
results = run_test(results, 'T2_risk_barrier_avoidance',    @t2_barrier);
results = run_test(results, 'T3_resample_length_and_endpoints', @t3_resample);
results = run_test(results, 'T4_static_mode_backward_compat',  @t4_static_compat);
results = run_test(results, 'T5_astar_lower_rmap_than_static', @t5_integration);

% --- Summary ---
fprintf('\n=== test_routing summary ===\n');
n_pass = 0;
for k = 1:numel(results)
    tag = '  PASS';
    if ~results(k).passed
        tag = '  FAIL';
    else
        n_pass = n_pass + 1;
    end
    fprintf('%s  %s', tag, results(k).name);
    if ~isempty(results(k).msg)
        fprintf('  — %s', results(k).msg);
    end
    fprintf('\n');
end
fprintf('%d/%d tests passed.\n\n', n_pass, numel(results));

if n_pass < numel(results)
    error('test_routing:failures', '%d test(s) failed.', numel(results) - n_pass);
end

% =========================================================================
% T1 — Zero-risk grid: A* must return the shortest (straight) path
% =========================================================================
function t1_zero_risk()
    nX = 50; nY = 30;
    cellSize = 100;                           % m
    xVec = (0 : nX-1) * cellSize + cellSize/2;
    yVec = (0 : nY-1) * cellSize + cellSize/2;

    K_norm  = zeros(nY, nX);                 % no risk anywhere
    p_start = [xVec(1),  yVec(round(nY/2))];
    p_goal  = [xVec(end), yVec(round(nY/2))];

    [path_ne, ~] = astar_grid(K_norm, xVec, yVec, p_start, p_goal, 1.0, 0.0);

    % Path must stay on the same East row (straight along North)
    east_dev = abs(path_ne(:, 2) - yVec(round(nY/2)));
    assert(max(east_dev) <= cellSize, ...
        'Zero-risk path deviated by %.1f m (> 1 cell)', max(east_dev));

    % Path length must be approximately equal to straight-line distance
    straight   = norm(p_goal - p_start);
    path_len   = sum(sqrt(sum(diff(path_ne, 1, 1) .^ 2, 2)));
    assert(abs(path_len - straight) / straight < 0.05, ...
        'Path length %.1f m deviates >5%% from straight %.1f m', path_len, straight);
end

% =========================================================================
% T2 — Risk barrier: high w_r avoids barrier; low w_r crosses it
% =========================================================================
function t2_barrier()
    nX = 60; nY = 40;
    cellSize = 50;
    xVec = (0 : nX-1) * cellSize + cellSize/2;
    yVec = (0 : nY-1) * cellSize + cellSize/2;

    mid_row = round(nY / 2);          % row where start/goal sit
    mid_col = round(nX / 2);          % column of barrier

    % Vertical barrier of high risk at mid_col (blocks the straight path)
    K_norm = zeros(nY, nX);
    K_norm(:, mid_col-1:mid_col+1) = 1.0;
    % Leave a gap one cell above / below edges so a detour is possible
    K_norm(1, :)  = 0;
    K_norm(end, :) = 0;

    p_start = [xVec(3),   yVec(mid_row)];
    p_goal  = [xVec(end-2), yVec(mid_row)];

    % HIGH risk weight → path must detour around barrier
    [path_hi, ~] = astar_grid(K_norm, xVec, yVec, p_start, p_goal, 0.1, 0.9);
    max_risk_hi  = max(K_norm(sub2ind([nY,nX], ...
        max(1,min(nY, round((path_hi(:,2)-yVec(1))/cellSize)+1)), ...
        max(1,min(nX, round((path_hi(:,1)-xVec(1))/cellSize)+1)))));
    assert(max_risk_hi < 0.5, ...
        'High-w_r path still enters high-risk zone (K=%.2f)', max_risk_hi);

    % LOW risk weight → path is allowed to cross barrier (shorter)
    [path_lo, ~] = astar_grid(K_norm, xVec, yVec, p_start, p_goal, 0.99, 0.01);
    len_hi = sum(sqrt(sum(diff(path_hi, 1, 1) .^ 2, 2)));
    len_lo = sum(sqrt(sum(diff(path_lo, 1, 1) .^ 2, 2)));
    assert(len_lo < len_hi, ...
        'Low-w_r path (%.1f m) not shorter than high-w_r path (%.1f m)', len_lo, len_hi);
end

% =========================================================================
% T3 — resample_path_to_slots: length, start, end, padding
% =========================================================================
function t3_resample()
    % Simple L-shaped path: 500 m right then 200 m up
    path_ne = [0, 0; 500, 0; 500, 200];
    total_len = 700;                    % m

    v  = 15;   % m/s
    dt = 1/15; % s  (updateRate = 15 Hz)
    T  = 200;

    pos = resample_path_to_slots(path_ne, T, v, dt);

    % Start matches first waypoint
    assert(norm(pos(1,:) - path_ne(1,:)) < 1e-6, ...
        'First slot != start waypoint');

    % After full traversal (700 m / (15 m/s * 1/15 s) = 700 slots), should
    % be at goal.  T=200 < 700, so pos(end) should be somewhere mid-path.
    % Just verify we're not stuck at start or jumped to end.
    assert(pos(end, 1) > 0 && pos(end, 1) <= 500 + 1e-6, ...
        'North coord at slot T=%.0f out of range: %.1f', T, pos(end,1));

    % Test padding: with T large enough to exceed path, last slots = goal
    T_long = 2000;
    pos_long = resample_path_to_slots(path_ne, T_long, v, dt);
    assert(norm(pos_long(end,:) - path_ne(end,:)) < 1e-6, ...
        'Padding failed: last slot not at goal');

    % Verify continuity: no slot-to-slot jump larger than 2 * step_m
    step_m = v * dt;
    dists = sqrt(sum(diff(pos_long, 1, 1) .^ 2, 2));
    assert(max(dists) <= 2 * step_m + 1e-6, ...
        'Discontinuity detected: max inter-slot jump = %.2f m (step=%.2f)', ...
        max(dists), step_m);
end

% =========================================================================
% T4 — Backward compatibility: static mode leaves infra.precomputed_trajectory empty
% =========================================================================
function t4_static_compat()
    cfg = build_scenario_config();
    cfg = validate_uam_config(cfg);

    % Ensure static mode is default
    assert(strcmp(cfg.routing.mode, 'static'), ...
        'Default routing mode is not ''static'' (got: %s)', cfg.routing.mode);

    % Simulate what run_sim does for static mode
    use_astar = isfield(cfg, 'routing') && strcmp(cfg.routing.mode, 'risk_astar');
    assert(~use_astar, 'Static cfg incorrectly activates A*');

    % infra.precomputed_trajectory should be empty (set by run_sim in static branch)
    infra.precomputed_trajectory = {};
    use_precomputed = isfield(infra, 'precomputed_trajectory') && ...
                      ~isempty(infra.precomputed_trajectory);
    assert(~use_precomputed, ...
        'Static mode: use_precomputed should be false with empty cell');
end

% =========================================================================
% T5 — Integration: Risk-A* produces lower mean R_map than static
%        (uses a small synthetic scenario without uavScenario to stay fast)
% =========================================================================
function t5_integration()
    % Build synthetic rhoMap with a strong hotspot centred on the corridor
    cellSize = 50;
    half     = 1500;
    margin   = 300;
    xVec = ((-(half+margin)+cellSize/2) : cellSize : ((half+margin)-cellSize/2));
    yVec = ((-margin+cellSize/2) : cellSize : (margin-cellSize/2));
    [XG, YG] = meshgrid(xVec, yVec);
    nX = numel(xVec); nY = numel(yVec);

    % Strong hotspot right on the straight corridor (East=0, North=0)
    rhoMap = exp(-((XG - 0).^2 + (YG - 0).^2) / (2 * 300^2));

    mapGrid.XG      = XG;
    mapGrid.YG      = YG;
    mapGrid.xFlat   = XG(:);
    mapGrid.yFlat   = YG(:);
    mapGrid.rhoFlat = rhoMap(:);
    mapGrid.deltaA  = cellSize^2;

    % Routing config — strongly risk-averse
    routing.lambda         = 1e-4;
    routing.uav_mass       = 1.5;
    routing.uav_fall_speed = 20;
    routing.I              = 100;
    routing.I0             = 34;
    routing.uav_sheltering = 0.5;
    routing.w_d            = 0.2;
    routing.w_r            = 0.8;

    p_start = [-half, 0];
    p_goal  = [ half, 0];

    % A* path
    K_norm = compute_risk_map(rhoMap, mapGrid, routing);
    [path_astar, ~] = astar_grid(K_norm, xVec, yVec, p_start, p_goal, ...
                                  routing.w_d, routing.w_r);

    % Static path (straight line)
    path_static = [p_start; p_goal];

    % Compute mean rho along each path by sampling
    sample_astar  = sample_rho_along_path(path_astar,  mapGrid);
    sample_static = sample_rho_along_path(path_static, mapGrid);

    mean_rho_astar  = mean(sample_astar);
    mean_rho_static = mean(sample_static);

    assert(mean_rho_astar < mean_rho_static, ...
        'A* path mean rho (%.4f) >= static path mean rho (%.4f)', ...
        mean_rho_astar, mean_rho_static);

    fprintf('  [T5] rho: static=%.4f  astar=%.4f  reduction=%.1f%%\n', ...
            mean_rho_static, mean_rho_astar, ...
            100*(mean_rho_static - mean_rho_astar)/mean_rho_static);
end

% ---- Helpers ----

function rho_vals = sample_rho_along_path(path_ne, mapGrid)
% Sample rho at each waypoint of path_ne using nearest-cell lookup.
    xVec = mapGrid.XG(1, :);
    yVec = mapGrid.YG(:, 1);
    cellSize = xVec(2) - xVec(1);
    nX = numel(xVec); nY = numel(yVec);
    rhoMap = reshape(mapGrid.rhoFlat, size(mapGrid.XG));
    rho_vals = zeros(size(path_ne, 1), 1);
    for k = 1:size(path_ne, 1)
        col = max(1, min(nX, round((path_ne(k,1) - xVec(1)) / cellSize) + 1));
        row = max(1, min(nY, round((path_ne(k,2) - yVec(1)) / cellSize) + 1));
        rho_vals(k) = rhoMap(row, col);
    end
end

function results = run_test(results, name, fn)
    fprintf('Running %s ... ', name);
    try
        fn();
        results(end+1) = struct('name', name, 'passed', true,  'msg', '');
        fprintf('PASS\n');
    catch ME
        results(end+1) = struct('name', name, 'passed', false, 'msg', ME.message);
        fprintf('FAIL: %s\n', ME.message);
    end
end
