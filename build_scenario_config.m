function cfg = build_scenario_config(varargin)
% BUILD_SCENARIO_CONFIG  Returns a fully-populated UAM scenario config struct.
%
%   cfg = build_scenario_config()               — interactive defaults
%   cfg = build_scenario_config('nhpp')          — NHPP arrival model
%   cfg = build_scenario_config('rush_hour')     — rush-hour composite
%   cfg = build_scenario_config('batch')         — batch arrivals
%   cfg = build_scenario_config('deterministic') — equally-spaced stress test
%
%   Any field can be overridden after the call:
%       cfg = build_scenario_config('nhpp');
%       cfg.numDrones     = 30;
%       cfg.nhpp_horizon  = 400;
%       cfg.lambdaProfile = @(t) 0.3 + 0.9*(t>100 & t<200);

%% ------------------------------------------------------------------ %%
%  0. ARRIVAL MODEL SELECTION
%% ------------------------------------------------------------------ %%

arrModel = 'uniform';  % 'uniform', 'nhpp','rush_hour','batch','deterministic'         — NHPP arrival model


%% ------------------------------------------------------------------ %%
%  1. CORE SIMULATION PARAMETERS  (unchanged from original)
%% ------------------------------------------------------------------ %%
cfg.schedulingPolicy = 'pf-classic';  % 'round-robin' | 'pf-classic' | 'pf-aoi' | 'risk-aware' | 'hybrid'
cfg.sched_alpha      = 0.7;

cfg.numDrones       = 5;
cfg.corridorLength  = 3000;
cfg.flightTime      = 60;
cfg.updateRate      = 7;
cfg.speedVal        = cfg.corridorLength / cfg.flightTime;

cfg.fc              = 3.5e9;
cfg.pTransmitDrone  = 23;
cfg.gNB_Gain        = 15;
cfg.noiseFigure     = 7;
cfg.bw              = 20e6;
cfg.thermalNoise    = -174 + 10*log10(cfg.bw);
cfg.thresholdSNR    = 5;

cfg.r_min           = 10;
cfg.k_aoi           = 0.5;
cfg.dronesPerSlot   = 1;

cfg.droneEastPos    = 0;
cfg.minStartDelay   = 2;
cfg.maxStartDelay   = 45;

cfg.radioDelay      = 1;
cfg.coreDelay       = 2;
cfg.videoDelay      = 10;
cfg.latency_base    = cfg.radioDelay + cfg.coreDelay + cfg.videoDelay;

cfg.w_unc           = 1.0;
cfg.w_map           = 1e-4;
cfg.w_vid           = 10.0;
cfg.v_max           = cfg.speedVal;
cfg.tau_max         = 500;
cfg.d_crit          = 200;

cfg.R_bar_sys       = 500;

cfg.microBSPos      = [-800 0 -30; 0 0 -30; 800 0 -30];
cfg.masterPos       = [0 0 -60];

cfg.rng_seed        = 42;
cfg.numHotspots     = 6;
cfg.manualHotspots  = [];   % Nx4: [x y amp sigma]

%% ------------------------------------------------------------------ %%
%  2. ARRIVAL MODEL FIELDS  (Fase 1)
%% ------------------------------------------------------------------ %%
cfg.arrivalModel = arrModel;

switch arrModel

    % ---- NHPP -------------------------------------------------------
    case 'nhpp'
        % Default: smooth bell curve — most drones arrive mid-window
        cfg.nhpp_horizon   = cfg.maxStartDelay * 2.5;
        cfg.nhpp_lambda_max = [];    % auto (set inside generate_arrival_times)
        cfg.lambdaProfile  = @(t) ...
            (cfg.numDrones / cfg.nhpp_horizon) * ...
            (0.3 + 1.4 * exp(-0.5*((t - cfg.nhpp_horizon*0.45)/(cfg.nhpp_horizon*0.15)).^2));

    % ---- BATCH ------------------------------------------------------
    case 'batch'
        cfg.nhpp_horizon  = cfg.maxStartDelay * 2.5;
        cfg.batchLambda   = 0.25;   % groups / second
        cfg.batchSize     = [2 4];  % [min max] drones per group
        cfg.batchJitter   = 2.0;    % within-group temporal spread [s]

    % ---- DETERMINISTIC ----------------------------------------------
    case 'deterministic'
        cfg.det_spacing = (cfg.maxStartDelay - cfg.minStartDelay) / ...
                          max(cfg.numDrones - 1, 1);

    % ---- RUSH_HOUR --------------------------------------------------
    case 'rush_hour'
        cfg.nhpp_horizon  = cfg.maxStartDelay * 2.5;
        cfg.rh_peak1_t    = cfg.nhpp_horizon * 0.20;
        cfg.rh_peak2_t    = cfg.nhpp_horizon * 0.72;
        cfg.rh_peak_width = cfg.nhpp_horizon * 0.09;
        cfg.rh_base_rate  = 0.04;
        cfg.rh_peak_rate  = 0.55;

    % ---- UNIFORM (original) -----------------------------------------
    otherwise
        % Nothing extra needed — generate_arrival_times handles it.
end

end