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

% --- Normalization references R_bar_nav, R_bar_gnd (legacy) ---
n_sources    = 2 * cfg.numDrones;
h_star_slots = n_sources / cfg.p_c2;
h_star_s     = h_star_slots / cfg.updateRate;
r_star       = cfg.r_min + cfg.v_max * h_star_s;

d_min          = 2 * cfg.r_min;
cfg.R_bar_nav  = max((cfg.numDrones - 1) * max(2*r_star - d_min, 0), 1e-15);
cfg.R_bar_gnd  = cfg.rho_0 * pi * r_star^2;

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
    cfg.H_max = 4 * (2*cfg.numDrones) / max(cfg.dronesPerSlot * cfg.p_c2, 1e-3);
end

% v_max in m/slot for the radius equation r_u = r_min + v_max h_{u,s}
% (h_{u,s} is in slots, so v_max needs slot-consistent units)
v_max_m_per_slot = cfg.v_max / cfg.updateRate;

% Navigation bound coefficients (Sec. 3 of doc)
cfg.C_Nav_0 = 2*(cfg.numDrones-1)*cfg.r_min ...
            + (cfg.numDrones-1)*v_max_m_per_slot*cfg.H_max;
cfg.C_Nav_1 = (cfg.numDrones-1)*v_max_m_per_slot;

% Ground bound coefficients (Sec. 3 of doc)
cfg.C_Gnd_0 = cfg.rho_0 * pi * cfg.r_min^2;
cfg.C_Gnd_1 = cfg.rho_0 * pi * ( ...
              2*cfg.r_min*v_max_m_per_slot ...
            + (v_max_m_per_slot^2)*cfg.H_max);

% Linear surrogate coefficients (Sec. 3, final boxed form)
cfg.n_s = cfg.w_unc * cfg.C_Nav_1 + cfg.w_map * cfg.C_Gnd_1;
cfg.n_v = cfg.w_vid / cfg.H_nom_slots;

% =====================================================================
% Phase 3 -- LB throughput targets via Cauchy-Schwarz (Sec. 6, Step 2)
%
%   q_{u,s}^LB = K * sqrt(omega_u * n_s * p_{u,s}) / Theta
%   q_{u,v}^LB = K * sqrt(omega_u * n_v * L * p_{u,v}) / Theta
%   Theta = sum_u sqrt(omega_u n_s / p_{u,s})
%         + sum_u sqrt(omega_u n_v L / p_{u,v})
%
% Debt targets: qbar = q_LB - epsilon  (strict feasibility)
% =====================================================================

if isempty(cfg.omega)
    cfg.omega = ones(cfg.numDrones, 1);
end
cfg.omega = cfg.omega(:);
if numel(cfg.omega) ~= cfg.numDrones
    error('UAM:config', 'cfg.omega must have numDrones=%d entries.', cfg.numDrones);
end

p_s_vec = cfg.p_c2  * ones(cfg.numDrones, 1);
p_v_vec = cfg.p_vid * ones(cfg.numDrones, 1);
L       = cfg.L_vid;
K       = cfg.dronesPerSlot;

Theta = sum(sqrt(cfg.omega * cfg.n_s ./ p_s_vec)) ...
      + sum(sqrt(cfg.omega * cfg.n_v * L ./ p_v_vec));

cfg.q_LB_s = K .* sqrt(cfg.omega * cfg.n_s .* p_s_vec) ./ Theta;
cfg.q_LB_v = K .* sqrt(cfg.omega * cfg.n_v * L .* p_v_vec) ./ Theta;

% Strict-feasibility margin (Step 3 of Sec. 6): epsilon -> 0+
cfg.qbar_eps = 0.01;
cfg.qbar_s   = max(cfg.q_LB_s - cfg.qbar_eps, 1e-6);
cfg.qbar_v   = max(cfg.q_LB_v - cfg.qbar_eps, 1e-6);

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
