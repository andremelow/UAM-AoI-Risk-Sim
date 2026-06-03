function cfg = base_config_capacity()
% BASE_CONFIG_CAPACITY  Base scenario config for the capacity experiment.
%
%  Corridor: single straight route, East -1000 m to +1000 m (2 km).
%  Speed: 5 m/s → flightTime = 400 s per drone.
%  Arrival model: 'relay' — drones spaced flightTime/C apart so exactly
%  corridorCapacity drones are active simultaneously at steady state.
%
%  Call build_capacity_config(C, K, laps) to get a run-ready config for a
%  given capacity C and dronesPerSlot K.

cfg = build_scenario_config();   % start from canonical defaults

% ---- Corredor: 2 km East, 2 lanes paralelos (North ±50 m) ------------
% Lane Norte (+50 m) e Lane Sul (−50 m) têm o mesmo flightTime = 400 s.
% define_capacity_scenarios() selecciona 1 ou 2 lanes por cenário.
cfg.routes      = struct();
cfg.routes(1).numDrones = 1;           % placeholder; set by build_capacity_run_cfg
cfg.routes(1).start     = [ 50, -1000]; % [North, East]  m  — lane Norte
cfg.routes(1).goal      = [ 50,  1000];
cfg.routes(1).label     = 'lane-N';

cfg.corridorLength = 2000;             % m  (rota 1; ambas têm o mesmo comprimento)
cfg.speedVal       = 5;                % m/s
cfg.flightTime     = cfg.corridorLength / cfg.speedVal;   % 400 s

% ---- gNB: centre of corridor at ground level --------------------------
% microBSPos convention: [East, North, Alt]  (step_channel.m line comment)
cfg.microBSPos = [0, 0, -30];

% ---- Arrival model: capacity_drain ------------------------------------
% Fase fill → fase estável (steady_duration s com C ativos) → fase drain.
% N_total calculado em build_capacity_run_cfg: C + floor(T_steady/spacing).
cfg.arrivalModel     = 'capacity_drain';
cfg.corridorCapacity = 1;    % overridden per run
cfg.steady_duration  = 1200; % segundos em regime C drones ativos (20 min)
cfg.minStartDelay    = 2;

% ---- Radio (keep identical to routing experiment) ---------------------
cfg.updateRate    = 7;
cfg.fc            = 3.5e9;
cfg.pTransmitDrone = 23;
cfg.gNB_Gain      = 15;
cfg.noiseFigure   = 7;
cfg.bw            = 20e6;
cfg.thresholdSNR  = 5;

% ---- AoI / scheduling weights -----------------------------------------
w_sum        = 1.0 + 1.9e-3 + 10.0;
cfg.w_unc    = 1.0    / w_sum;
cfg.w_map    = 1.9e-3 / w_sum;
cfg.w_vid    = 10.0   / w_sum;
cfg.r_min    = 10;
cfg.k_aoi    = 0.5;
cfg.tau_max  = 500;
cfg.d_crit   = 200;
cfg.R_bar_sys = 500;

% ---- Dual-source video model ------------------------------------------
cfg.videoModel = 'atomic';
cfg.fps        = 30;
cfg.L_vid      = 100;
cfg.L_c2       = 1;
cfg.w_coh      = 5;
cfg.L_nom      = 0.01;
cfg.p_c2       = 0.80;
cfg.p_vid      = 0.80;
cfg.rho_0      = 0.01;

% ---- Routing: static (no Risk-A* by default) --------------------------
cfg.routing.mode = 'static';

% ---- Ground risk: mantém seed e hotspots originais de build_scenario_config
% init_ground_risk usa os hotspots que caírem dentro do grid do corredor.
% Não sobrescrever — cfg já herdou de build_scenario_config() acima.

% ---- Export -----------------------------------------------------------
cfg.csvExport = true;
cfg.csvDir    = 'csv_export/capacity';
end
