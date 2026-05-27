function pos_slots = resample_path_to_slots(path_ne, T, v, dt)
% RESAMPLE_PATH_TO_SLOTS  Maps a waypoint polyline to per-slot positions.
%
%   The drone moves at constant speed v along the polyline of cell-centre
%   waypoints.  For each slot t in 1..T the position is the point at arc
%   distance s = (t-1)*v*dt from the start.  Once the path is exhausted
%   (s >= total arc length), the final position is repeated.
%
%   Inputs
%     path_ne : (M x 2) [North, East] waypoints, ordered start → goal
%     T       : number of slots to generate  (= endSlot - startSlot + 1)
%     v       : drone speed  [m/s]
%     dt      : slot duration [s]  (= 1 / updateRate)
%
%   Output
%     pos_slots : (T x 2) [North, East] per-slot positions

if size(path_ne, 1) < 2
    % Degenerate path: stay at the single point for all slots
    pos_slots = repmat(path_ne(1, :), T, 1);
    return;
end

% Arc lengths along each segment and cumulative arc length from start
segs    = diff(path_ne, 1, 1);                    % (M-1) x 2 segment vectors
seg_len = sqrt(sum(segs .^ 2, 2));               % (M-1) x 1 segment lengths
cum_len = [0; cumsum(seg_len)];                   % (M)   x 1 cumulative arc
total_len = cum_len(end);
step_m    = v * dt;                               % metres per slot

pos_slots = zeros(T, 2);
goal      = path_ne(end, :);

for t = 1:T
    s = (t - 1) * step_m;

    if s >= total_len
        pos_slots(t, :) = goal;
        continue;
    end

    % Last segment index whose cumulative start is <= s
    seg_idx = find(cum_len <= s, 1, 'last');
    seg_idx = min(seg_idx, numel(seg_len));       % guard: last valid segment

    frac = (s - cum_len(seg_idx)) / max(seg_len(seg_idx), 1e-12);
    pos_slots(t, :) = path_ne(seg_idx, :) + frac * segs(seg_idx, :);
end
end
