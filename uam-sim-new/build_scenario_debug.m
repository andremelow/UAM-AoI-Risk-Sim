function cfg = build_scenario_debug()
% BUILD_SCENARIO_DEBUG  Cenário de debug para análise de scheduling assimétrico.
%
%  Duas rotas em zonas SNR diferentes:
%    Route A — longa (~8 km), 1 km ao norte da BS: SNR 17-33 dB
%              [N=1000,E=1000]→[N=1000,E=9000]
%    Route B — curta (~2 km), longe da BS + hotspots: SNR ≈ 6-8 dB
%              [N=-500,E=12000]→[N=-600,E=14000]
%
%  Comportamento esperado:
%    PF       — prioriza zona de alto SNR → neglencia Route B → risco sobe
%    RR       — alterna sem considerar urgência → Route B sofre com SNR ruim
%    max-weight-sw — detecta h_s crescendo em Route B → realoca slots

arrModel = 'uniform';
cfg.schedulingPolicy = 'round-robin';

%% ---- Routes --------------------------------------------------------
cfg.routes(1).numDrones = 8;
cfg.routes(1).start     = [1000  1000];   % [North, East] — 1 km ao norte da BS
cfg.routes(1).goal      = [1000  9000];   %                 SNR 17-33 dB
cfg.routes(1).label     = 'RouteA';

cfg.routes(2).numDrones = 5;
cfg.routes(2).start     = [-500  12000];  % [North, East] — longe + hotspots
cfg.routes(2).goal      = [-600  14000];  %                 SNR ≈  6-8 dB
cfg.routes(2).label     = 'RouteB';

cfg.numDrones       = 13;
cfg.corridorLength  = 8000;               % RouteA ≈ 8000 m
cfg.flightTime      = 800;                % s  (≈10 m/s)
cfg.updateRate      = 7;
cfg.speedVal        = cfg.corridorLength / cfg.flightTime;

%% ---- Rádio ---------------------------------------------------------
cfg.fc              = 3.5e9;
cfg.pTransmitDrone  = 23;
cfg.gNB_Gain        = 15;
cfg.noiseFigure     = 7;
cfg.bw              = 20e6;
cfg.thermalNoise    = -174 + 10*log10(cfg.bw);
cfg.thresholdSNR    = 5;

%% ---- Misc ----------------------------------------------------------
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

cfg.w_unc           = 1/3;
cfg.w_map           = 1/3;
cfg.w_vid           = 1/3;
cfg.v_max           = cfg.speedVal;
cfg.tau_max         = 500;
cfg.d_crit          = 200;
cfg.R_bar_sys       = 500;

%% ---- gNB em 1/3 do corredor (mesmo que assimétrico) ----------------
cfg.microBSPos = [
    5000  0  -30 ];   % [East, North, Alt(neg=above ground)]

%% ---- Hotspots: terço final de RouteBlong (E≈11167–14000, sigma=180m) ----
%  24 pontos em 8 posições uniformes ao longo do terço final (t=0.67..1.0),
%  com 3 hotspots por posição: centro + ±150m perpendicular à rota.
%  Rota: [N=0,E=5500]→[N=-700,E=14000];  perpendicular = (-0.997, -0.082).
cfg.rng_seed    = 55;
cfg.numHotspots = 0;
cfg.manualHotspots = [
% [North    East      amp   sigma]     % t        offset
  -466.7   11166.7   0.75   180;       % t=0.667  centro
  -616.2   11154.4   0.75   180;       % t=0.667  +150m perp
  -317.2   11179.0   0.75   180;       % t=0.667  -150m perp
  -500.0   11571.4   0.75   180;       % t=0.714  centro
  -649.5   11559.1   0.75   180;       % t=0.714  +150m perp
  -350.5   11583.7   0.75   180;       % t=0.714  -150m perp
  -533.3   11976.2   0.75   180;       % t=0.762  centro
  -682.8   11963.9   0.75   180;       % t=0.762  +150m perp
  -383.8   11988.5   0.75   180;       % t=0.762  -150m perp
  -566.7   12381.0   0.75   180;       % t=0.810  centro
  -716.2   12368.6   0.75   180;       % t=0.810  +150m perp
  -417.2   12393.3   0.75   180;       % t=0.810  -150m perp
  -600.0   12785.7   0.75   180;       % t=0.857  centro
  -749.5   12773.4   0.75   180;       % t=0.857  +150m perp
  -450.5   12798.0   0.75   180;       % t=0.857  -150m perp
  -633.3   13190.5   0.75   180;       % t=0.905  centro
  -782.8   13178.2   0.75   180;       % t=0.905  +150m perp
  -483.8   13202.8   0.75   180;       % t=0.905  -150m perp
  -666.7   13595.2   0.75   180;       % t=0.952  centro
  -816.2   13582.9   0.75   180;       % t=0.952  +150m perp
  -517.2   13607.5   0.75   180;       % t=0.952  -150m perp
  -700.0   14000.0   0.75   180;       % t=1.000  centro
  -849.5   13987.7   0.75   180;       % t=1.000  +150m perp
  -550.5   14012.3   0.75   180;       % t=1.000  -150m perp
];

%% ---- Routing -------------------------------------------------------
cfg.routing.mode           = 'static';
cfg.routing.w_d            = 0.50;
cfg.routing.w_r            = 0.50;
cfg.routing.lambda         = 0.0001;
cfg.routing.uav_mass       = 1.5;
cfg.routing.uav_fall_speed = 20;
cfg.routing.I              = 100;
cfg.routing.I0             = 34;
cfg.routing.uav_sheltering = 0.5;

cfg.arrivalModel = arrModel;
end
