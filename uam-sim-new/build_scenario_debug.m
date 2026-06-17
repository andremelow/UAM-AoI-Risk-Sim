function cfg = build_scenario_debug()
% BUILD_SCENARIO_DEBUG  Cenário de debug para análise de scheduling assimétrico.
%
%  Duas rotas CURTAS (~2 km) em zonas SNR fixas:
%    Route A — próxima à BS:  [N=200,E=4000]→[N=300,E=6000]   SNR ≈ 35-40 dB
%    Route B — longe da BS:   [N=-500,E=12000]→[N=-600,E=14000] SNR ≈  6-8 dB
%                             (passa pelos hotspots manuais em E≈12800-13200)
%
%  Comportamento esperado:
%    PF       — prioriza Route A (alto SNR) → neglencia Route B → risco sobe
%    RR       — alterna sem considerar urgência → Route B sofre com SNR ruim
%    max-weight-sw — detecta h_s crescendo em Route B → realoca slots

arrModel = 'uniform';
cfg.schedulingPolicy = 'round-robin';

%% ---- Routes --------------------------------------------------------
cfg.routes(1).numDrones = 5;
cfg.routes(1).start     = [200   4000];   % [North, East] — próxima à BS
cfg.routes(1).goal      = [300   6000];   %                 SNR ≈ 35-40 dB
cfg.routes(1).label     = 'RouteA';

cfg.routes(2).numDrones = 5;
cfg.routes(2).start     = [-500  12000];  % [North, East] — longe + hotspots
cfg.routes(2).goal      = [-600  14000];  %                 SNR ≈  6-8 dB
cfg.routes(2).label     = 'RouteB';

cfg.numDrones       = 10;
cfg.corridorLength  = norm([100, 2000]);   % ≈ 2002 m
cfg.flightTime      = 200;                 % s  (≈10 m/s)
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

%% ---- Hotspots sob Route B (East ≈ 12800-13200, North ≈ -700) -------
cfg.rng_seed    = 55;
cfg.numHotspots = 0;   % sem hotspot aleatório — apenas manuais abaixo
cfg.manualHotspots(1,:)  = [-716 13161 0.75 120];
cfg.manualHotspots(2,:)  = [-720 13143 0.75 120];
cfg.manualHotspots(3,:)  = [-702 13073 0.75 120];
cfg.manualHotspots(4,:)  = [-713 13047 0.75 120];
cfg.manualHotspots(5,:)  = [-716 13007 0.75 120];
cfg.manualHotspots(6,:)  = [-702 12951 0.75 120];
cfg.manualHotspots(7,:)  = [-698 12896 0.75 120];
cfg.manualHotspots(8,:)  = [-691 12863 0.75 120];
cfg.manualHotspots(9,:)  = [-676 12856 0.75 120];
cfg.manualHotspots(10,:) = [-676 12852 0.75 120];
cfg.manualHotspots(11,:) = [-676 12852 0.75 120];
cfg.manualHotspots(12,:) = [-676 12852 0.75 120];
cfg.manualHotspots(13,:) = [-738 12830 0.75 120];
cfg.manualHotspots(14,:) = [-783 12823 0.75 120];

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
