function state = step_read_positions(state, drones, infra, time, slot, cfg)
% STEP_READ_POSITIONS  Reads drone positions, updates active set.
%
%   When infra.precomputed_trajectory{i} is non-empty (Risk-A* mode),
%   the position is taken from the precomputed (T x 2) [North, East]
%   matrix instead of calling read(drones{i}).  The uavScenario still
%   advances time normally; only the position fed to the scheduler and
%   risk model is overridden.  Altitude is fixed at cfg.hmapZ (-10 m
%   above ground, matching init_ground_risk) for A* trajectories.
%
%   In static mode (infra.precomputed_trajectory empty), behaviour is
%   identical to the original implementation.

N = cfg.numDrones;
state.activeDrones = [];
state.activePos    = zeros(N, 3);

use_precomputed = isfield(infra, 'precomputed_trajectory') && ...
                  ~isempty(infra.precomputed_trajectory);

% Altitude for A* trajectories: match the original corridor altitude (-60 m)
astar_alt = -60;

for i = 1:N
    % Mark finished drones as invalid — preserve time-series data for collect_results
    if slot > state.endSlots(i)
        state.pendingMsg{i}.valid = false;
        continue;
    end

    % Skip not-yet-active drones
    if slot < state.startSlots(i)
        state.pendingMsg{i}.valid = false;
        continue;
    end

    % --- Position source: precomputed A* trajectory or uavScenario ---
    if use_precomputed && numel(infra.precomputed_trajectory) >= i && ...
            ~isempty(infra.precomputed_trajectory{i})

        traj      = infra.precomputed_trajectory{i};
        t_local   = slot - state.startSlots(i) + 1;          % 1-based offset
        t_local   = max(1, min(size(traj, 1), t_local));      % clamp
        pos = [traj(t_local, 1), traj(t_local, 2), astar_alt, 1, 0, 0, 0];

    else
        [pos, ~] = read(drones{i});
        if i == 1 && mod(slot, 50) == 0
            fprintf('t=%.2f drone1 pos=[%g %g %g %g %g %g %g]\n', time, pos(1:7));
        end
        if any(isnan(pos(1:3))), continue; end
    end

    state.pendingMsg{i}.pos       = pos(1:3);
    state.pendingMsg{i}.slot      = slot;
    state.pendingMsg{i}.valid     = true;
    state.activeDrones(end+1)     = i; %#ok<AGROW>
    state.activePos(i,:)          = pos(1:3);
end
end
