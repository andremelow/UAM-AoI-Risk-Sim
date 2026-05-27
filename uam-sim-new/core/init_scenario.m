function [scene, drones, infra] = init_scenario(cfg)
% INIT_SCENARIO  Creates uavScenario, base stations, and drone platforms.
%
%  Extracted from monolith sections 1–5.
%
%  Reproducibility: uses cfg.rng_seed (default 42 via validate_uam_config)
%  instead of rng('shuffle'), so different runs of the same config produce
%  identical drone arrival times and trajectories. This is essential for
%  parametric sweeps where you want to isolate the effect of the swept
%  parameter from arrival-time noise.

N = cfg.numDrones;

% --- Scene ---
% StopTime = last drone landing + 2-slot buffer
[startTimes, arrivalMeta] = generate_arrival_times(cfg);
endTimes   = startTimes + cfg.flightTime;
stopTime   = max(endTimes) + 2 / cfg.updateRate;

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
% NOTE: startTimes/endTimes were already computed above using the
% deterministic seeded RNG inside generate_arrival_times. We do NOT
% call rng('shuffle') or generate_arrival_times again — that would
% (a) destroy reproducibility across sweep runs and (b) produce a
% different startTimes vector than the one used to compute stopTime.
drones = cell(N, 1);
for k = 1:N
    half = cfg.corridorLength / 2;
    p1   = [-half  cfg.droneEastPos  -60];
    p2   = [ half  cfg.droneEastPos  -60];
    traj = waypointTrajectory([p1; p2], ...
           TimeOfArrival=[startTimes(k) endTimes(k)]);
    drones{k} = uavPlatform("UAV"+k, scene, Trajectory=traj);
    updateMesh(drones{k},"quadrotor",{4},[.2 .2 .2],[0 0 0],[1 0 0 0]);
end

fprintf('[Arrival] model=%-14s  N=%d  seed=%d\n', ...
        arrivalMeta.model, N, cfg.rng_seed);

setup(scene);

% --- Infrastructure output struct ---
infra.microBSPos  = cfg.microBSPos;
infra.startTimes  = startTimes;
infra.endTimes    = endTimes;
infra.startSlots  = round(startTimes * cfg.updateRate);
infra.endSlots    = round(endTimes   * cfg.updateRate);
infra.arrivalMeta = arrivalMeta;

plCfg = nrPathLossConfig;
plCfg.Scenario = "UMi";
infra.plCfg = plCfg;
end
