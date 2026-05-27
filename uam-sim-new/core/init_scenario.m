function [scene, drones, infra] = init_scenario(cfg)
% INIT_SCENARIO  Creates uavScenario, base stations, and drone platforms.
%
%  Supports multiple routes via cfg.routes (struct array with .numDrones,
%  .start [North,East], .goal [North,East], .label per entry).
%  validate_uam_config expands legacy corridorLength/droneEastPos to a
%  single-entry cfg.routes when the field is absent, so backward compat
%  is fully preserved.
%
%  Per-drone flight time = norm(goal − start) / cfg.speedVal, so drones
%  on shorter routes land earlier than drones on longer routes.

N = cfg.numDrones;

% --- Build per-drone route lookup tables from cfg.routes ---
droneRoute = zeros(N, 1);
droneStart = zeros(N, 2);   % [North, East]
droneGoal  = zeros(N, 2);
idx = 0;
for r = 1:numel(cfg.routes)
    rt = cfg.routes(r);
    for k = 1:rt.numDrones
        idx = idx + 1;
        droneRoute(idx)    = r;
        droneStart(idx, :) = rt.start(:)';
        droneGoal(idx, :)  = rt.goal(:)';
    end
end

% --- Arrival times (global, covers all routes) ---
[startTimes, arrivalMeta] = generate_arrival_times(cfg);

% --- Per-drone flight time from route length and global speed ---
routeLen    = sqrt(sum((droneGoal - droneStart).^2, 2));
flightTimes = routeLen / cfg.speedVal;
endTimes    = startTimes + flightTimes;
stopTime    = max(endTimes) + 2 / cfg.updateRate;

% --- Scene ---
scene = uavScenario( ...
    UpdateRate   = cfg.updateRate, ...
    StopTime     = stopTime, ...
    MaxNumFrames = 200);

% --- Base stations ---
for i = 1:size(cfg.microBSPos, 1)
    bs = uavPlatform("BS"+i, scene, InitialPosition=cfg.microBSPos(i,:));
    updateMesh(bs,"cuboid",{[30 30 30]},[0 0 1],[0 0 30],[1 0 0 0]);
end

% --- Drone platforms ---
% NOTE: startTimes/endTimes were computed with a seeded RNG inside
% generate_arrival_times — do NOT call it again here (reproducibility).
drones = cell(N, 1);
for k = 1:N
    p1   = [droneStart(k,1), droneStart(k,2), -60];
    p2   = [droneGoal(k,1),  droneGoal(k,2),  -60];
    traj = waypointTrajectory([p1; p2], ...
           TimeOfArrival=[startTimes(k) endTimes(k)]);
    drones{k} = uavPlatform("UAV"+k, scene, Trajectory=traj);
    updateMesh(drones{k},"quadrotor",{4},[.2 .2 .2],[0 0 0],[1 0 0 0]);
end

fprintf('[Arrival] model=%-14s  N=%d  routes=%d  seed=%d\n', ...
        arrivalMeta.model, N, numel(cfg.routes), cfg.rng_seed);

setup(scene);

% --- Infrastructure output struct ---
infra.microBSPos  = cfg.microBSPos;
infra.startTimes  = startTimes;
infra.endTimes    = endTimes;
infra.startSlots  = round(startTimes * cfg.updateRate);
infra.endSlots    = round(endTimes   * cfg.updateRate);
infra.arrivalMeta = arrivalMeta;
infra.droneRoute  = droneRoute;
infra.droneStart  = droneStart;
infra.droneGoal   = droneGoal;

plCfg = nrPathLossConfig;
plCfg.Scenario = "UMi";
infra.plCfg = plCfg;
end
