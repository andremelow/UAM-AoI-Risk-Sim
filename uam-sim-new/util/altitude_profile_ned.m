function alt_ned = altitude_profile_ned(s, route_len, cruise_alt, takeoff_dist)
% ALTITUDE_PROFILE_NED  Altitude (NED z-component) as a function of route position.
%
%  Returns the NED z-value (negative = above ground) for a drone that has
%  travelled distance s along a route of total length route_len.
%
%  Profile:
%    [0, td]              — linear ramp 0 → cruise_alt  (takeoff)
%    [td, route_len-td]   — constant cruise_alt         (cruise)
%    [route_len-td, end]  — linear ramp cruise_alt → 0  (landing)
%
%  If route_len < 2*takeoff_dist, the ramp is capped at route_len/2 so
%  takeoff and landing meet exactly at the midpoint (no flat cruise phase).
%
%  Inputs:
%    s            — scalar or vector, distance along route [m]
%    route_len    — total route length [m]
%    cruise_alt   — cruise altitude AGL [m]
%    takeoff_dist — horizontal distance for takeoff/landing ramp [m]
%
%  Output:
%    alt_ned  — NED z-component [m] (negative values = above ground)

td = min(takeoff_dist, route_len / 2);

alt = zeros(size(s));
for i = 1:numel(s)
    si = s(i);
    if si <= td
        alt(i) = cruise_alt * si / td;
    elseif si >= route_len - td
        alt(i) = cruise_alt * (route_len - si) / td;
    else
        alt(i) = cruise_alt;
    end
end

alt_ned = -alt;   % NED convention: negative z = above ground
end
