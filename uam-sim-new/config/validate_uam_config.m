function cfg = validate_uam_config(cfg)
% VALIDATE_UAM_CONFIG  Fills defaults and adds derived constants.
%
%  Updated to fully match the formal model (Sec. 3-6 of analysis doc):
%    - Linear risk surrogate coefficients n_s, n_v (Sec. 3)
%    - Max-Weight stabilization params V_s, V_v, kappa (Sec. 4)
%    - Per-source LB throughput targets q_LB and debt targets qbar (Sec. 6)
%    - Optional normalization flag use_normalization (Sec. 2 vs Sec. 3 alignment)

% --- Routing defaults (Risk-A* upstream path planning) ---
defaults.routing = struct( ...
    'mode',           'static', ...   % 'static' | 'risk_astar'
    'w_d',            0.5,      ...   % distance weight
    'w_r',            0.5,      ...   % risk weight (w_d + w_r must == 1)
    'lambda',         1e-4,     ...   % drone-fall probability per flight
    'uav_mass',       1.5,      ...   % kg
    'uav_fall_speed', 20,       ...   % m/s impact speed
    'I',              100,      ...   % J — energy at 50% fatality
    'I0',             34,       ...   % J — energy at 0%  fatality
    'uav_sheltering', 0.5       ...   % fraction of risk reduced by sheltering
);

% --- Default policy and CSV settings ---
defaults.schedulingPolicy = 'round-robin';   % round-robin | pf-classic | aoi-pure | risk-aware | max-weight
defaults.csvExport        = false;
defaults.csvDir           = 'csv_export';
defaults.sched_alpha      = 0.5;

% --- Scenario geometry ---
defaults.numDrones       = 3;
defaults.corridorLength  = 1800;
defaults.flightTime      = 25;
defaults.updateRate      = 600;
defaults.dronesPerSlot   = 1;          % K — channels per slot
defaults.droneEastPos    = 100;
defaults.minStartDelay   = 20;
defaults.maxStartDelay   = 50;

% --- Radio ---
defaults.fc              = 3.5e9;
defaults.pTransmitDrone  = 23;
defaults.gNB_Gain        = 8;
defaults.noiseFigure     = 6;
defaults.bw              = 20e6;
defaults.thresholdSNR    = 10;

defaults.radioDelay      = 5;
defaults.coreDelay       = 10;
defaults.videoDelay      = 11;

% --- Risk weights (NOTE: doc uses w_Nav, w_Ground, w_vid;
%     legacy code uses w_unc/w_map/w_vid — kept as aliases). ---
defaults.w_unc           = 0.33;   % alias for w_Nav
defaults.w_map           = 0.33;   % alias for w_Ground
defaults.w_vid           = 0.34;
defaults.r_min           = 25;
defaults.k_aoi           = 1;
defaults.tau_max         = 500;
defaults.d_crit          = 200;
defaults.R_bar_sys       = 1e5;

defaults.microBSPos      = [0 -1600 -30];
defaults.numHotspots     = 12;
defaults.rng_seed        = 42;
defaults.manualHotspots  = [];

% --- Dual-source / video model ---
defaults.videoModel      = 'atomic';   % {'atomic','gop'}
defaults.fps             = 30;
defaults.GOP             = 15;
defaults.L_I             = 30;
defaults.L_P             = 3;
defaults.L_vid           = 1;          % atomic-frame size [packets/slots]
defaults.L_c2            = 1;
defaults.w_coh           = 5;
defaults.L_nom           = 0.010;      % nominal one-way latency [s]
defaults.p_c2            = 0.8;        % per-slot status success probability
defaults.p_vid           = 0.8;        % per-slot video success probability
defaults.rho_0           = 0.01;       % uniform ground density [people/m^2]

% --- Doc-aligned options (Phase 1-4 additions) ---
%
% use_normalization:
%   false (default, doc-aligned): R_nav and R_gnd reported in raw units,
%                                  matching the surrogate bound R_u <= C + n_s h + n_v z.
%   true: normalize by R_bar_nav, R_bar_gnd (legacy comparative dashboards).
defaults.use_normalization = false;

% Max-Weight stabilization parameters (Sec. 4.3 of doc)
%   V_s, V_v: debt-queue weights in the Lyapunov function
%   kappa:    multi-slot occupation penalty for video starts
defaults.V_s             = 1.0;
defaults.V_v             = 1.0;
defaults.kappa           = 0.0;        % default off; tune for L>1 video

% Priority weights omega_u (per-UAV). Default: uniform.
defaults.omega            = [];

% --- Fill missing fields ---
fields = fieldnames(defaults);
for i = 1:numel(fields)
    f = fields{i};
    if ~isfield(cfg, f)
        cfg.(f) = defaults.(f);
    end
end

% --- Routes expansion ---
% If cfg.routes is absent, synthesise a single route from legacy corridor params.
% If cfg.routes is present, recompute cfg.numDrones as the sum of route counts.
if ~isfield(cfg, 'routes') || isempty(cfg.routes)
    half = cfg.corridorLength / 2;
    cfg.routes(1).numDrones = cfg.numDrones;
    cfg.routes(1).start     = [-half, cfg.droneEastPos];
    cfg.routes(1).goal      = [ half, cfg.droneEastPos];
    cfg.routes(1).label     = 'Route 1';
else
    % Validate and fill defaults for each route
    for r = 1:numel(cfg.routes)
        if ~isfield(cfg.routes(r), 'start') || ~isfield(cfg.routes(r), 'goal')
            error('UAM:config', 'cfg.routes(%d) must have .start and .goal fields.', r);
        end
        if ~isfield(cfg.routes(r), 'label')
            cfg.routes(r).label = sprintf('Route %d', r);
        end
        if ~isfield(cfg.routes(r), 'numDrones') || cfg.routes(r).numDrones < 1
            cfg.routes(r).numDrones = 1;
        end
    end
    cfg.numDrones = sum([cfg.routes.numDrones]);
    % Keep corridorLength consistent with first route for speedVal computation
    cfg.corridorLength = norm(cfg.routes(1).goal(:)' - cfg.routes(1).start(:)');
end

% --- Validate videoModel ---
cfg.videoModel = lower(strtrim(cfg.videoModel));
if ~ismember(cfg.videoModel, {'atomic','gop'})
    error('UAM:config', ...
        'cfg.videoModel must be ''atomic'' or ''gop'' (got: %s).', cfg.videoModel);
end

% --- Derived radio / kinematic fields ---
cfg.speedVal     = cfg.corridorLength / cfg.flightTime;
cfg.latency_base = cfg.radioDelay + cfg.coreDelay + cfg.videoDelay;
cfg.thermalNoise = -174 + 10*log10(cfg.bw);
cfg.v_max        = cfg.speedVal;

% --- Dual-source derived constants ---
cfg.frame_interval  = 1 / cfg.fps;
cfg.slots_per_frame = cfg.updateRate / cfg.fps;

% H_nom in slots (eq. for H_nom in the doc / Sec. 2)
L_nom_slots     = cfg.L_nom * cfg.updateRate;
cfg.H_nom_slots = L_nom_slots + 2*(cfg.w_coh - 1) * cfg.slots_per_frame / 3;

% Coherence weights alpha_i (decreasing, sum to 1)
cfg.alpha_w = zeros(cfg.w_coh, 1);
for ii = 1:cfg.w_coh
    cfg.alpha_w(ii) = 2*(cfg.w_coh + 1 - ii) / (cfg.w_coh*(cfg.w_coh + 1));
end

% =====================================================================
% N_eff: effective concurrent drones for scheduling theory bounds.
% In relay/capacity experiments numDrones = corridorCapacity × relay_laps,
% but the scheduler only ever sees corridorCapacity drones at a time.
% Using N_total would inflate H_max, n_s, qbar by factor relay_laps.
% =====================================================================
if isfield(cfg, 'corridorCapacity') && cfg.corridorCapacity > 0 && ...
        cfg.corridorCapacity < cfg.numDrones
    N_eff = cfg.corridorCapacity;
else
    N_eff = cfg.numDrones;
end

% --- Normalization references R_bar_nav, R_bar_gnd (legacy) ---
n_sources    = 2 * N_eff;
h_star_slots = n_sources / cfg.p_c2;
h_star_s     = h_star_slots / cfg.updateRate;
r_star       = cfg.r_min + cfg.v_max * h_star_s;

d_min          = 2 * cfg.r_min;
cfg.R_bar_nav  = max((N_eff - 1) * max(2*r_star - d_min, 0), 1e-15);
cfg.R_bar_gnd  = max(cfg.rho_0 * pi * r_star^2, 1e-15);

% =====================================================================
% Phase 1 -- Linear risk surrogate coefficients (Sec. 3 of doc)
%
%   R_u(t) <= C_u + n_s * h_{u,s}(t) + n_v * z_u(t)
%
%   n_s = w_Nav * C_Nav,1 + w_Ground * C_Ground,1
%   n_v = w_vid / H_nom
%   C_u = w_Nav * C_Nav,0 + w_Ground * C_Ground,0
%
%   Bounds use H_max as the AoI cap. Under round-robin worst case,
%   each of the 2N sources is served once every 2N/K slots, so
%   E[h_{u,s}] ~ 2N/(K p_c2). We use a conservative multiplicative
%   margin of 4x to account for variance and unreliable channels.
% =====================================================================

% AoI cap H_max [slots] -- conservative analytic estimate
%   (used both for the surrogate bound and for the LB throughput targets)
if ~isfield(cfg, 'H_max') || isempty(cfg.H_max)
    cfg.H_max = 4 * (2*N_eff) / max(cfg.dronesPerSlot * cfg.p_c2, 1e-3);
end

% v_max in m/slot for the radius equation r_u = r_min + v_max h_{u,s}
% (h_{u,s} is in slots, so v_max needs slot-consistent units)
v_max_m_per_slot = cfg.v_max / cfg.updateRate;

% Navigation bound coefficients (Sec. 3 of doc)
cfg.C_Nav_0 = 2*(N_eff-1)*cfg.r_min ...
            + (N_eff-1)*v_max_m_per_slot*cfg.H_max;
cfg.C_Nav_1 = (N_eff-1)*v_max_m_per_slot;

% Ground bound coefficients (Sec. 3 of doc)
cfg.C_Gnd_0 = cfg.rho_0 * pi * cfg.r_min^2;
cfg.C_Gnd_1 = cfg.rho_0 * pi * ( ...
              2*cfg.r_min*v_max_m_per_slot ...
            + (v_max_m_per_slot^2)*cfg.H_max);

% Linear surrogate coefficients (Sec. 3, final boxed form)
cfg.n_s = cfg.w_unc * cfg.C_Nav_1 + cfg.w_map * cfg.C_Gnd_1;
cfg.n_v = cfg.w_vid / cfg.H_nom_slots;

% policy_max_weight_drift parameters (Sec. 2).
% beta_s / beta_v default to the same surrogate coefficients as n_s / n_v.
% V replaces the separate V_s, V_v of the old policy with a single value.
if ~isfield(cfg, 'beta_s') || isempty(cfg.beta_s)
    cfg.beta_s = cfg.n_s;
end
if ~isfield(cfg, 'beta_v') || isempty(cfg.beta_v)
    cfg.beta_v = cfg.n_v;
end
if ~isfield(cfg, 'V') || isempty(cfg.V)
    cfg.V = cfg.V_s;   % V_s default is 1.0
end

% =====================================================================
% Quadratic Lyapunov coefficients (paper eqs. 20-21)
%
%   eta_s = w_Nav*(N-1)*v_max/(2*R_bar_nav) + w_Ground*rho_0*pi*(r_min*v+v^2)/R_bar_gnd
%   eta_v = w_vid / (2*H_nom)
%
%   Used by policy_max_weight_sw (eqs. 29-30).
%   C_Gnd_1_phys uses only the first-order v term (no H_max multiplier)
%   because the quadratic weight already captures h_s growth directly.
% =====================================================================
C_Gnd_1_phys = cfg.rho_0 * pi * (2*cfg.r_min*v_max_m_per_slot + v_max_m_per_slot^2);
cfg.eta_s = cfg.w_unc * cfg.C_Nav_1 / (2 * cfg.R_bar_nav) + ...
            cfg.w_map * C_Gnd_1_phys / cfg.R_bar_gnd;
cfg.eta_v = cfg.w_vid / (2 * cfg.H_nom_slots);

% =====================================================================
% Phase 3 -- LB throughput targets via Cauchy-Schwarz (Sec. 6, Step 2)
%
%   q_{u,s}^LB = K * sqrt(omega_u * n_s * p_{u,s}) / Theta
%   q_{u,v}^LB = K * sqrt(omega_u * n_v * L * p_{u,v}) / Theta
%   Theta = sum_u sqrt(omega_u n_s / p_{u,s})
%         + sum_u sqrt(omega_u n_v L / p_{u,v})
%
% Debt targets: qbar = q_LB - epsilon  (strict feasibility)
% Computed over N_eff sources; replicated to numDrones for per-drone indexing.
% =====================================================================

if isempty(cfg.omega)
    cfg.omega = ones(N_eff, 1);
end
cfg.omega = cfg.omega(:);
if numel(cfg.omega) ~= N_eff
    warning('UAM:config', 'omega resized to N_eff=%d (corridorCapacity).', N_eff);
    cfg.omega = ones(N_eff, 1);
end

p_s_vec = cfg.p_c2  * ones(N_eff, 1);
p_v_vec = cfg.p_vid * ones(N_eff, 1);
L       = cfg.L_vid;
K       = cfg.dronesPerSlot;

Theta = sum(sqrt(cfg.omega * cfg.n_s ./ p_s_vec)) ...
      + sum(sqrt(cfg.omega * cfg.n_v * L ./ p_v_vec));

cfg.q_LB_s = K .* sqrt(cfg.omega * cfg.n_s .* p_s_vec) ./ Theta;
cfg.q_LB_v = K .* sqrt(cfg.omega * cfg.n_v * L .* p_v_vec) ./ Theta;

% Strict-feasibility margin (Step 3 of Sec. 6): epsilon -> 0+
cfg.qbar_eps = 0.01;

% Cap targets at physical throughput limits before applying epsilon.
%
% x_s is updated per slot: x_s += qbar_s - mu_s, with mu_s ∈ {0,1}.
% For x_s to be stable: qbar_s ≤ E[mu_s] ≤ p_c2.
%
% Each drone is served at most ONCE per slot (binary scheduling with
% used_drones enforcement). Therefore the per-slot, per-drone service
% fraction is min(K/N_eff, 1), NOT K — using K here inflates qbar
% well beyond the achievable rate when K >> N_eff, causing x_s → ∞
% and pathological max-weight behaviour (risk increases with K at low C).
svc_frac = min(K / N_eff, 1);              % fraction of slots a drone can be served
max_q_s  = svc_frac .* p_s_vec;           % max C2  throughput per drone per slot
max_q_v  = svc_frac .* p_v_vec ./ L;      % max vid throughput per drone per slot
cfg.q_LB_s = min(cfg.q_LB_s, max_q_s);
cfg.q_LB_v = min(cfg.q_LB_v, max_q_v);

cfg.qbar_s   = max(cfg.q_LB_s - cfg.qbar_eps, 1e-6);
cfg.qbar_v   = max(cfg.q_LB_v - cfg.qbar_eps, 1e-6);

% Switching-mode qbar: in preemptive (switching) policies the one-radio-
% per-drone constraint limits each drone to at most 1 slot per epoch
% regardless of K. qbar_s grows with K via the CS formula, but the
% achievable C2 throughput stays bounded at p_c2 per slot. When
% qbar_s >> achievable rate, x_s diverges and max-weight-sw starves video.
% Fix: compute qbar_sw using K_eff = min(K, N_eff) — extra slots beyond
% N_eff bring no per-drone throughput gain in switching mode.
K_sw          = min(K, N_eff);
q_LB_s_sw    = K_sw .* sqrt(cfg.omega * cfg.n_s .* p_s_vec) ./ Theta;
q_LB_v_sw    = K_sw .* sqrt(cfg.omega * cfg.n_v * L .* p_v_vec) ./ Theta;
q_LB_s_sw    = min(q_LB_s_sw, max_q_s);
q_LB_v_sw    = min(q_LB_v_sw, max_q_v);
cfg.qbar_s_sw = max(q_LB_s_sw - cfg.qbar_eps, 1e-6);
cfg.qbar_v_sw = max(q_LB_v_sw - cfg.qbar_eps, 1e-6);

% Replicate per-drone vectors to numDrones length.
% In the relay model N_eff = corridorCapacity < numDrones, so omega/qbar
% computed over N_eff entries must be extended for per-drone indexing
% in step_transmit (cfg.qbar_s(i)), step_risk (cfg.omega(k)), and policies.
if cfg.numDrones > N_eff
    tile = @(v) v(mod((0:cfg.numDrones-1)', N_eff) + 1);
    cfg.omega     = tile(cfg.omega);
    cfg.q_LB_s    = tile(cfg.q_LB_s);
    cfg.q_LB_v    = tile(cfg.q_LB_v);
    cfg.qbar_s    = tile(cfg.qbar_s);
    cfg.qbar_v    = tile(cfg.qbar_v);
    cfg.qbar_s_sw = tile(cfg.qbar_s_sw);
    cfg.qbar_v_sw = tile(cfg.qbar_v_sw);
end

% =====================================================================
% Routing config: fill sub-fields that may be missing when cfg.routing
% was set partially by the user (the top-level fill-loop above only
% sets cfg.routing if the field is entirely absent).
% =====================================================================
routing_defaults = defaults.routing;
rfields = fieldnames(routing_defaults);
for ri = 1:numel(rfields)
    f = rfields{ri};
    if ~isfield(cfg.routing, f)
        cfg.routing.(f) = routing_defaults.(f);
    end
end

% Validate routing mode
valid_modes = {'static', 'risk_astar'};
if ~ismember(cfg.routing.mode, valid_modes)
    error('UAM:config', ...
        'cfg.routing.mode must be ''static'' or ''risk_astar'' (got: %s).', ...
        cfg.routing.mode);
end

% Validate w_d + w_r == 1 for risk_astar mode
if strcmp(cfg.routing.mode, 'risk_astar')
    w_sum = cfg.routing.w_d + cfg.routing.w_r;
    if abs(w_sum - 1) > 1e-9
        error('UAM:config', ...
            'cfg.routing.w_d + cfg.routing.w_r must equal 1 (got %.6f).', w_sum);
    end
    if cfg.routing.I <= cfg.routing.I0
        error('UAM:config', ...
            'cfg.routing.I (%g J) must be > cfg.routing.I0 (%g J).', ...
            cfg.routing.I, cfg.routing.I0);
    end
end

% Sanity
if any([cfg.w_unc, cfg.w_map, cfg.w_vid] < 0)
    warning('UAM:config', 'Negative risk weight detected.');
end
if abs((cfg.w_unc + cfg.w_map + cfg.w_vid) - 1) > 1e-6
    warning('UAM:config', ...
        'Risk weights w_Nav+w_Ground+w_vid = %.4f (should sum to 1).', ...
        cfg.w_unc + cfg.w_map + cfg.w_vid);
end
end
