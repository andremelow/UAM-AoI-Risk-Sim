function traj = route_risk_astar(infra, rhoMap, mapGrid, cfg, drone_id)
% ROUTE_RISK_ASTAR  Pre-computes a risk-aware trajectory for one drone.
%
%   Implements Risk-A* (Hu et al., IEEE Access 2020) as a pre-flight
%   planning step, upstream of and decoupled from the scheduling policy.
%
%   Pipeline:
%     1. compute_risk_map  — builds normalised K_c grid from rhoMap
%     2. astar_grid        — finds minimum-cost path on 8-connected grid
%     3. resample_path_to_slots — interpolates polyline to per-slot positions
%
%   The returned matrix traj is indexed by local slot offset:
%     traj(1,:)   = position at startSlot(drone_id)
%     traj(end,:) = position at endSlot(drone_id)
%   Slots after path arrival repeat the goal position (corridor exit).
%
%   Coordinate convention (matches rest of simulator, NED):
%     traj(:,1) = North  [m]
%     traj(:,2) = East   [m]
%   Altitude is fixed at cfg.hmapZ (default -60 m) and added by the caller
%   (step_read_positions) when assembling the 3-D position vector.
%
%   Inputs
%     infra    : struct from init_scenario (fields: startSlots, endSlots, …)
%     rhoMap   : [nY x nX] density field from init_ground_risk
%     mapGrid  : struct from init_ground_risk (XG, YG, deltaA, …)
%     cfg      : full simulation config; uses cfg.routing, cfg.corridorLength,
%                cfg.droneEastPos, cfg.speedVal, cfg.updateRate
%     drone_id : scalar integer drone index (1..N)
%
%   Output
%     traj : (T x 2) [North, East] matrix, T = endSlots-startSlots+1

routing = cfg.routing;

% Extract xVec / yVec from the meshgrid matrices stored in mapGrid
% (init_ground_risk uses [XG,YG]=meshgrid(xVec,yVec), so XG(1,:)=xVec, YG(:,1)=yVec)
xVec = mapGrid.XG(1, :);   % North cell-centre coords, length nX
yVec = mapGrid.YG(:, 1);   % East  cell-centre coords, length nY

% Normalised ground-risk cost matrix
K_norm = compute_risk_map(rhoMap, mapGrid, routing);

% Corridor start / goal in world coords [North, East]
half    = cfg.corridorLength / 2;
p_start = [-half,  cfg.droneEastPos];
p_goal  = [ half,  cfg.droneEastPos];

% Run A*
t_astar = tic;
[path_ne, cost] = astar_grid(K_norm, xVec, yVec, p_start, p_goal, ...
                              routing.w_d, routing.w_r);
dt_astar = toc(t_astar);

path_len = sum(sqrt(sum(diff(path_ne, 1, 1) .^ 2, 2)));
straight  = norm(p_goal - p_start);
fprintf('[A*] drone %2d: nodes=%4d  cost=%.4f  path=%.0fm  straight=%.0fm  dt=%.2fs\n', ...
        drone_id, size(path_ne, 1), cost, path_len, straight, dt_astar);

% Resample to per-slot positions
T  = infra.endSlots(drone_id) - infra.startSlots(drone_id) + 1;
dt = 1 / cfg.updateRate;
traj = resample_path_to_slots(path_ne, T, cfg.speedVal, dt);
end
