function [K_norm, K_raw] = compute_risk_map(rhoMap, mapGrid, routing)
% COMPUTE_RISK_MAP  Ground-risk cost per grid cell (Hu et al., IEEE Access 2020).
%
%   K_c = lambda * A_c * P_c * F_c
%
%   P_c  = rhoMap cell value (population/occupancy density from init_ground_risk)
%   A_c  = mapGrid.deltaA  [m^2]
%   F_c  = (1 - Sh_c) * P_fatal(E_impact)
%   E    = 0.5 * m * v_fall^2   [J]
%   P_fatal = clip( (E - I0) / (I - I0), 0, 1 )   linear interpolation
%
%   Inputs
%     rhoMap  : [nY x nX] density field from init_ground_risk
%     mapGrid : struct with deltaA (cell area [m^2])
%     routing : struct with fields:
%                 lambda         — drone-fall probability per flight [1/s or unitless]
%                 uav_mass       — drone mass [kg]
%                 uav_fall_speed — impact speed [m/s]
%                 I              — energy at 50% fatality [J]
%                 I0             — energy at 0%  fatality [J]
%                 uav_sheltering — sheltering fraction Sh in [0,1] (scalar or
%                                  matrix matching rhoMap; default 0.5)
%
%   Outputs
%     K_norm  : [nY x nX] risk cost, normalised to [0, 1]
%     K_raw   : [nY x nX] risk cost in physical units

% Impact energy and fatality probability
E_impact = 0.5 * routing.uav_mass * routing.uav_fall_speed^2;   % [J]
I_range  = routing.I - routing.I0;
if I_range <= 0
    error('compute_risk_map:badParams', ...
        'routing.I (%g) must be > routing.I0 (%g).', routing.I, routing.I0);
end
P_fatal = max(0, min(1, (E_impact - routing.I0) / I_range));

% Sheltering: scalar or matrix (same shape as rhoMap)
if isfield(routing, 'uav_sheltering')
    Sh = routing.uav_sheltering;
else
    Sh = 0.5;
end
F_c = (1 - Sh) * P_fatal;   % scalar or [nY x nX]

% Ground-risk cost per cell
K_raw = routing.lambda * mapGrid.deltaA * rhoMap .* F_c;

% Normalise to [0, 1] so that w_r is dimensionless and directly comparable to w_d
K_max = max(K_raw(:));
if K_max > 0
    K_norm = K_raw / K_max;
else
    K_norm = K_raw;   % all-zero map; A* degenerates to shortest-path
end
end
