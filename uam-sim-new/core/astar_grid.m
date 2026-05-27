function [path_ne, total_cost] = astar_grid(K_norm, xVec, yVec, p_start, p_goal, w_d, w_r)
% ASTAR_GRID  A* on an 8-connected grid with a mixed distance+risk cost.
%
%   Cost of arc c_i → c_j:
%       g_arc = w_d * (dist_ij / d_max) + w_r * K_norm(c_j)
%   where d_max = sqrt(2)*cellSize (diagonal step, the longest arc).
%
%   Heuristic (admissible):
%       h(c) = w_d * euclidean_dist(c, goal) / d_max
%
%   Grid layout (matches init_ground_risk / meshgrid convention):
%       K_norm is [nY x nX]
%       rows  → East  index (yVec, length nY)
%       cols  → North index (xVec, length nX)
%       K_norm(row, col) = risk at North=xVec(col), East=yVec(row)
%
%   Inputs
%     K_norm  : [nY x nX] normalised risk map (0…1)
%     xVec    : North cell-centre coords, length nX  (= mapGrid.XG(1,:))
%     yVec    : East  cell-centre coords, length nY  (= mapGrid.YG(:,1))
%     p_start : [North, East] world start
%     p_goal  : [North, East] world goal
%     w_d     : distance weight  (0…1, must sum to 1 with w_r)
%     w_r     : risk weight      (0…1)
%
%   Outputs
%     path_ne    : (M x 2) [North, East] cell-centre positions, start → goal
%     total_cost : accumulated g-cost of the returned path

[nY, nX] = size(K_norm);
cellSize  = xVec(2) - xVec(1);
d_max     = sqrt(2) * cellSize;

% --- World ↔ grid helpers ---
% w2g: world (north, east) → [row, col], clamped to valid range
w2g = @(north, east) [ ...
    max(1, min(nY, round((east  - yVec(1)) / cellSize) + 1)), ...
    max(1, min(nX, round((north - xVec(1)) / cellSize) + 1))];

% Column-major linear index: row varies fastest
lin  = @(r, c)  r + (c - 1) * nY;
r_of = @(li)    mod(li - 1, nY) + 1;
c_of = @(li)    floor((li - 1) / nY) + 1;

% Start and goal in grid coords
tmp = w2g(p_start(1), p_start(2));  r_s = tmp(1);  c_s = tmp(2);
tmp = w2g(p_goal(1),  p_goal(2));   r_g = tmp(1);  c_g = tmp(2);

% Pre-compute goal world position for heuristic
north_g = xVec(c_g);
east_g  = yVec(r_g);

% --- A* data structures ---
g_cost    = inf(nY, nX);
n_cells   = nY * nX;
came_from = zeros(n_cells, 1, 'int32');   % 0 = no parent (marks start)

g_cost(r_s, c_s) = 0;
h_start = w_d * sqrt((xVec(c_s) - north_g)^2 + (yVec(r_s) - east_g)^2) / d_max;

% Open set: matrix with one entry per row: [f_cost, row, col]
% Lazy-deletion: duplicates are allowed; closed nodes are skipped when popped.
open   = [h_start, r_s, c_s];
closed = false(nY, nX);
found  = false;

% 8-connected moves: [Δrow, Δcol] and normalised arc length (1 = diagonal)
% NOTE: NB must be double, not int8. MATLAB promotes (double+int8) to int8,
% which causes lin(nr,nc) to overflow for nc>=9 with nY=18 (max int8=127),
% writing came_from at the wrong index and silently breaking path recovery.
NB  = [-1,-1; -1,0; -1,1; 0,-1; 0,1; 1,-1; 1,0; 1,1];
ARC = [1; 1/sqrt(2); 1; 1/sqrt(2); 1/sqrt(2); 1; 1/sqrt(2); 1];
%      diag  card  diag  card       card      diag  card   diag

while ~isempty(open)
    % Pop entry with minimum f_cost
    [~, imin] = min(open(:, 1));
    cur_r = open(imin, 2);
    cur_c = open(imin, 3);
    open(imin, :) = [];

    % Lazy deletion: skip if already processed
    if closed(cur_r, cur_c), continue; end
    closed(cur_r, cur_c) = true;

    % Goal test
    if cur_r == r_g && cur_c == c_g
        found = true;
        break;
    end

    g_cur    = g_cost(cur_r, cur_c);
    north_c  = xVec(cur_c);
    east_c   = yVec(cur_r);

    for k = 1:8
        nr = cur_r + NB(k, 1);
        nc = cur_c + NB(k, 2);
        if nr < 1 || nr > nY || nc < 1 || nc > nX, continue; end
        if closed(nr, nc), continue; end

        g_new = g_cur + w_d * ARC(k) + w_r * K_norm(nr, nc);

        if g_new < g_cost(nr, nc)
            g_cost(nr, nc)         = g_new;
            came_from(lin(nr, nc)) = int32(lin(cur_r, cur_c));

            h_val = w_d * sqrt((xVec(nc) - north_g)^2 + (yVec(nr) - east_g)^2) / d_max;
            open(end+1, :) = [g_new + h_val, nr, nc]; %#ok<AGROW>
        end
    end
end

total_cost = g_cost(r_g, c_g);

% --- Fallback: straight line if no path found ---
if ~found || isinf(total_cost)
    warning('astar_grid:nopath', ...
        'A* found no path (%.0fN,%.0fE)→(%.0fN,%.0fE); returning straight line.', ...
        p_start(1), p_start(2), p_goal(1), p_goal(2));
    path_ne    = [p_start(1), p_start(2); p_goal(1), p_goal(2)];
    total_cost = inf;
    return;
end

% --- Reconstruct path by backtracking came_from ---
li_cur  = lin(r_g, c_g);
li_start = lin(r_s, c_s);
chain   = li_cur;
while came_from(li_cur) ~= 0 && li_cur ~= li_start
    li_cur = double(came_from(li_cur));
    chain(end+1) = li_cur; %#ok<AGROW>
end
chain = flip(chain);

% Convert linear indices to [North, East] world coordinates
path_ne = zeros(numel(chain), 2);
for i = 1:numel(chain)
    li = chain(i);
    path_ne(i, :) = [xVec(c_of(li)), yVec(r_of(li))];
end
end
