function UAM_AoI_Control_Sim_WithRisk(cfg)
% UAM_AoI_Control_Sim_WithRisk  —  accepts a config struct from
%   build_scenario_config() / the Scenario Builder UI.
%
% Usage:
%   UAM_AoI_Control_Sim_WithRisk(build_scenario_config())
%
%   Or with overrides:
%   cfg = build_scenario_config();
%   cfg.numDrones = 30;
%   cfg.w_unc = 0.5;
%   UAM_AoI_Control_Sim_WithRisk(cfg);

clc; close all;

%% =========================================================
% 0. VALIDATE & UNPACK CONFIG
%% =========================================================
if nargin < 1, cfg = build_scenario_config(); end
cfg = validate_uam_config(cfg);          % fills defaults + derived

% Unpack — every downstream line stays identical to your original
schedulingPolicy = cfg.schedulingPolicy;
sched_alpha      = cfg.sched_alpha;

numDrones       = cfg.numDrones;
corridorLength  = cfg.corridorLength;
flightTime      = cfg.flightTime;
updateRate      = cfg.updateRate;
speedVal        = cfg.speedVal;           % derived in validator

fc              = cfg.fc;
pTransmitDrone  = cfg.pTransmitDrone;
gNB_Gain        = cfg.gNB_Gain;
noiseFigure     = cfg.noiseFigure;
bw              = cfg.bw;
thermalNoise    = cfg.thermalNoise;       % derived
thresholdSNR    = cfg.thresholdSNR;

r_min           = cfg.r_min;
k_aoi           = cfg.k_aoi;
dronesPerSlot   = cfg.dronesPerSlot;

droneEastPos    = cfg.droneEastPos;
minStartDelay   = cfg.minStartDelay;
maxStartDelay   = cfg.maxStartDelay;

radioDelay      = cfg.radioDelay;
coreDelay       = cfg.coreDelay;
videoDelay      = cfg.videoDelay;
latency_base    = cfg.latency_base;       % derived

w_unc           = cfg.w_unc;
w_map           = cfg.w_map;
w_vid           = cfg.w_vid;
v_max           = cfg.v_max;              % derived (= speedVal)
tau_max         = cfg.tau_max;
d_crit          = cfg.d_crit;
eps_sep         = 1e-3;                   % constant, not configurable

R_bar_sys       = cfg.R_bar_sys;

microBSPos      = cfg.microBSPos;
masterPos       = cfg.masterPos;

%% =========================================================
% 1. HEATMAP — merges seeded random + manual hotspots
%% =========================================================
mapXlim  = [-1600, 1600];
mapYlim  = [-600,   600];
cellSize = 50;
hmapZ    = -10;

xVec = (mapXlim(1)+cellSize/2) : cellSize : (mapXlim(2)-cellSize/2);
yVec = (mapYlim(1)+cellSize/2) : cellSize : (mapYlim(2)-cellSize/2);
[XG, YG] = meshgrid(xVec, yVec);
deltaA   = cellSize^2;

% Inicializar vetores sempre antes de qualquer append
hotX   = [];
hotY   = [];
hotAmp = [];
hotSig = [];

% Hotspots aleatórios com seed reproduzível
rng(cfg.rng_seed);
for h = 1:cfg.numHotspots
    hotX(end+1)   = mapXlim(1) + rand() * diff(mapXlim); %#ok<AGROW>
    hotY(end+1)   = mapYlim(1) + rand() * diff(mapYlim); %#ok<AGROW>
    hotAmp(end+1) = 0.5 + 0.5 * rand();                  %#ok<AGROW>
    hotSig(end+1) = 80  + 120  * rand();                  %#ok<AGROW>
end

% Append hotspots manuais vindos do config (Scenario Builder)
if ~isempty(cfg.manualHotspots)
    for m = 1:size(cfg.manualHotspots, 1)
        hotX(end+1)   = cfg.manualHotspots(m,1);
        hotY(end+1)   = cfg.manualHotspots(m,2);
        hotAmp(end+1) = cfg.manualHotspots(m,3);
        hotSig(end+1) = cfg.manualHotspots(m,4);
    end
end

numHotspots = numel(hotX);   % total real após merge

% Construir rhoMap
rhoMap = zeros(size(XG));
for h = 1:numHotspots
    rhoMap = rhoMap + hotAmp(h) * ...
        exp(-((XG - hotX(h)).^2 + (YG - hotY(h)).^2) / (2 * hotSig(h)^2));
end
rhoMap = min(rhoMap, 1);

% Flatten para lookup rápido dentro do loop por drone
xFlat   = XG(:);
yFlat   = YG(:);
rhoFlat = rhoMap(:);
%% =========================================================
% 2. CREATE SCENE  ← must exist before any uavPlatform call
%% =========================================================
scene = uavScenario( ...
    UpdateRate   = updateRate, ...
    StopTime     = flightTime * numDrones + 45, ...
    MaxNumFrames = 200);

%% =========================================================
% 3. INFRASTRUCTURE  ← scene now exists, safe to use
%% =========================================================
for i = 1:size(microBSPos, 1)
    bs = uavPlatform("BS"+i, scene, InitialPosition=microBSPos(i,:));
    updateMesh(bs,"cuboid",{[30 30 30]},[0 0 1],[0 0 30],[1 0 0 0]);
end

mBS = uavPlatform("MasterControl", scene, InitialPosition=masterPos);
updateMesh(mBS,"cuboid",{[30 30 50]},[1 0 0],[0 0 25],[1 0 0 0]);
%% =========================================================
% 4. SPATIAL RISK HEATMAP  rho(x)
%% =========================================================
mapXlim  = [-1600, 1600];
mapYlim  = [-600,   600];
cellSize = 50;
hmapZ    = -10;

xVec = (mapXlim(1)+cellSize/2) : cellSize : (mapXlim(2)-cellSize/2);
yVec = (mapYlim(1)+cellSize/2) : cellSize : (mapYlim(2)-cellSize/2);
[XG, YG] = meshgrid(xVec, yVec);
deltaA   = cellSize^2;

% --- Seeded random hotspots ---
rng(cfg.rng_seed);
numHotspots = cfg.numHotspots;

hotX   = mapXlim(1) + rand(1,numHotspots) * diff(mapXlim);
hotY   = mapYlim(1) + rand(1,numHotspots) * diff(mapYlim);
hotAmp = 0.5 + 0.5 * rand(1,numHotspots);
hotSig = 80  + 120 * rand(1,numHotspots);

% --- Append manual hotspots from config ---
if ~isempty(cfg.manualHotspots)
    for m = 1:size(cfg.manualHotspots, 1)
        hotX(end+1)   = cfg.manualHotspots(m,1);
        hotY(end+1)   = cfg.manualHotspots(m,2);
        hotAmp(end+1) = cfg.manualHotspots(m,3);
        hotSig(end+1) = cfg.manualHotspots(m,4);
    end
    numHotspots = numel(hotX);
end

% --- Build rhoMap ---
rhoMap = zeros(size(XG));
for h = 1:numHotspots
    rhoMap = rhoMap + hotAmp(h) * ...
        exp(-((XG-hotX(h)).^2 + (YG-hotY(h)).^2) / (2*hotSig(h)^2));
end
rhoMap = min(rhoMap, 1);

% --- Flatten for fast per-drone lookup inside the loop ---
xFlat   = XG(:);
yFlat   = YG(:);
rhoFlat = rhoMap(:);

%% =========================================================
% 5. DRONES
%% =========================================================
drones = cell(numDrones,1);
rng('shuffle');

startTimes = sort(minStartDelay + rand(numDrones,1)*(maxStartDelay-minStartDelay));
endTimes   = startTimes + flightTime;

for k = 1:numDrones
    p1   = [-1500  droneEastPos  -60];
    p2   = [ 1500  droneEastPos  -60];
    traj = waypointTrajectory([p1; p2], TimeOfArrival=[startTimes(k) endTimes(k)]);
    drones{k} = uavPlatform("UAV"+k, scene, Trajectory=traj);
    updateMesh(drones{k},"quadrotor",{4},[.2 .2 .2],[0 0 0],[1 0 0 0]);
end

setup(scene);

%% =========================================================
% 6. COMMUNICATION STATE
%% =========================================================
pendingMsg            = cell(numDrones,1);
lastReceivedTimestamp = nan(numDrones,1);
rrPointer             = 1;
snrLast = nan(numDrones, 1);   % último SNR calculado por drone
for k = 1:numDrones
    pendingMsg{k} = struct('pos',[NaN NaN NaN],'timestamp',NaN,'valid',false);
end

%% Contadores de transmissão por drone
txSuccess = zeros(numDrones, 1);   % transmissões com SNR > threshold
txFail    = zeros(numDrones, 1);   % tentativas com falha de canal

%% =========================================================
% 7. PAYLOAD & TRAFFIC STATE
%% =========================================================
Payload = struct();
Traffic = struct();
for i = 1:numDrones
    Traffic(i).nextPacketTime = exprnd(0.008);
    Traffic(i).queueMb        = 0;
end
for i = 1:numDrones
    Payload(i).throughput_Mbps = 0;
    Payload(i).latency_ms      = latency_base;
    Payload(i).lastLatency_ms  = latency_base;
    Payload(i).frameDelivery   = 1;
end

%% =========================================================
% 8. RISK HISTORY BUFFERS
%% =========================================================
riskTime = cell(numDrones,1);
riskUnc  = cell(numDrones,1);
riskMap  = cell(numDrones,1);
riskVid  = cell(numDrones,1);
for i = 1:numDrones
    riskTime{i}=[]; riskUnc{i}=[]; riskMap{i}=[]; riskVid{i}=[];
end

rSysTime=[]; rSysData=[];
uncMeanT=[]; uncMeanV=[];
mapMeanT=[]; mapMeanV=[];
vidMeanT=[]; vidMeanV=[];
aoiMeanT=[]; aoiMeanV=[];

%% =========================================================
% 9. FIGURE LAYOUT
%
%   [Row 1]  3D scene — full width (heatmap overlaid inside)
%   [Row 2]  R_unc per drone  |  R_map per drone
%   [Row 3]  R_vid per drone  |  R_sys(t)
%   [Row 4]  AoI              |  Payload latency & FDR
%   [Bottom] Live status bar (indicators panel)
%% =========================================================
hFig = figure('Color','w','Position',[40 30 1200 1130]);

% ---- STATUS BAR PANEL (bottom, fixed pixel height, normalized width) ----
statusBarH = 86;   % fixed pixel height — tiles are pixel-sized inside

% pStatus uses normalized horizontal, pixel vertical via a resize callback
pStatus = uipanel(hFig, ...
    'Units','normalized', ...
    'Position',[0  0  1  0.001], ...          % height set by callback below
    'BackgroundColor',[0.07 0.08 0.10], ...
    'BorderType','line', ...
    'HighlightColor',[0.25 0.60 0.90], ...
    'BorderWidth',1);

% ---- PLOT AREA (normalized, fills everything above the status bar) ----
pPlots = uipanel(hFig, ...
    'Units','normalized', ...
    'Position',[0  0.001  1  0.999], ...      % adjusted by callback below
    'BackgroundColor','w', ...
    'BorderType','none');
%% =========================================================
% 9b. DRONE STATUS PANEL (right sidebar inside pPlots)
%% =========================================================
panelW = 0.17;   % normalized width do painel lateral

pDroneStatus = uipanel(pPlots, ...
    'Units','normalized', ...
    'Position',[1-panelW, 0, panelW, 1], ...
    'BackgroundColor',[0.07 0.08 0.10], ...
    'BorderType','line', ...
    'HighlightColor',[0.22 0.24 0.30], ...
    'Title','Drone Status', ...
    'ForegroundColor',[0.50 0.55 0.65], ...
    'FontSize', 8, 'FontWeight','bold');

hDroneList = uicontrol(pDroneStatus, ...
    'Style','listbox', ...
    'Units','normalized', ...
    'Position',[0.01 0.01 0.98 0.98], ...
    'BackgroundColor',[0.07 0.08 0.10], ...
    'ForegroundColor',[0.75 0.80 0.90], ...
    'FontName','Courier New', ...
    'FontSize', 8, ...
    'String', {'Aguardando dados...'}, ...
    'Enable','inactive', ...
    'Max',2);          % permite seleção múltipla (sem interação do user)


% Resize callback — keeps status bar at a fixed pixel height while the
% plot panel fills the remaining space.  Fires on every window resize.
set(hFig, 'SizeChangedFcn', @(src,~) resizePanels(src, pStatus, pPlots, statusBarH));

% Run once immediately to set correct initial proportions
resizePanels(hFig, pStatus, pPlots, statusBarH);

% ---- Build indicator tiles inside pStatus (pixel coords within the bar) ----
indicatorW = 136;   % tile width  (px)
indicatorH = 68;    % tile height (px)
tileGap    = 6;
tileY      = 9;

indicatorLabels = { ...
    'ACTIVE DRONES',    'SCHEDULED / SLOT',  'SCHED. POLICY', ...
    'MEAN FDR',         'MEAN LATENCY',       'MEAN AoI', ...
    'SYS RISK  R_{sys}','RISK LEVEL',         'SIM TIME' };

numTiles  = numel(indicatorLabels);
% Tiles are centred horizontally; we use normalized x inside pStatus so
% they recentre when the window is widened.
hTileVal  = gobjects(1,numTiles);
hTileBG   = gobjects(1,numTiles);

accentNorm   = [0.25 0.60 0.90];
accentWarn   = [1.00 0.70 0.10];
accentDanger = [0.95 0.25 0.25];

totalTilesW = numTiles*(indicatorW+tileGap) - tileGap;
% Use normalized units for tile X so they stay centred on resize
for ti = 1:numTiles
    % normalized x position within pStatus (width = 1)
    txNorm = (0.5 - totalTilesW/2/1200) + (ti-1)*(indicatorW+tileGap)/1200;

    hTileBG(ti) = uipanel(pStatus, ...
        'Units','pixels', ...
        'Position',[0 tileY indicatorW indicatorH], ...   % X reset in callback
        'BackgroundColor',[0.12 0.13 0.16], ...
        'BorderType','line', ...
        'HighlightColor',[0.22 0.24 0.30], ...
        'BorderWidth',1, ...
        'UserData', ti);   % store index for resize callback

    uicontrol(hTileBG(ti), 'Style','text', ...
        'Units','pixels', 'Position',[4 42 indicatorW-8 18], ...
        'String', indicatorLabels{ti}, ...
        'ForegroundColor',[0.50 0.55 0.65], ...
        'BackgroundColor',[0.12 0.13 0.16], ...
        'FontSize',7, 'FontWeight','bold', ...
        'HorizontalAlignment','center');

    hTileVal(ti) = uicontrol(hTileBG(ti), 'Style','text', ...
        'Units','pixels', 'Position',[4 8 indicatorW-8 34], ...
        'String','—', ...
        'ForegroundColor', accentNorm, ...
        'BackgroundColor',[0.12 0.13 0.16], ...
        'FontSize',14, 'FontWeight','bold', ...
        'HorizontalAlignment','center');
end

% Update tile SizeChangedFcn on pStatus to recentre tiles horizontally
set(pStatus, 'SizeChangedFcn', @(src,~) recentreTiles(src, hTileBG, indicatorW, tileGap, tileY));
recentreTiles(pStatus, hTileBG, indicatorW, tileGap, tileY);   % run once now

% Indices into hTileVal for easy reference
IDX_ACTIVE   = 1;   IDX_SCHED    = 2;   IDX_POLICY  = 3;
IDX_FDR      = 4;   IDX_LAT      = 5;   IDX_AOI     = 6;
IDX_RSYS     = 7;   IDX_LEVEL    = 8;   IDX_TIME    = 9;

% Pre-fill static fields
set(hTileVal(IDX_POLICY), 'String', cfg.schedulingPolicy, ...
    'ForegroundColor', [0.40 0.85 0.55]);
set(hTileVal(IDX_SCHED),  'String',sprintf('%d / slot', dronesPerSlot));

% --- 3D scene (top, spans both columns) ---
ax3D = subplot(4,2,[1 2],'Parent',pPlots);
hold(ax3D,'on'); grid(ax3D,'on');
xlabel(ax3D,"North (m)"); ylabel(ax3D,"East (m)"); zlabel(ax3D,"Down (m)");
title(ax3D, ...
    "UAM Scenario – Ground Risk Heatmap (surface) + AoI Uncertainty Spheres");
set(ax3D,'ZDir','reverse');

% Draw heatmap as a coloured surface at z = hmapZ
surf(ax3D, XG, YG, hmapZ*ones(size(XG)), rhoMap, ...
    'EdgeColor','none','FaceAlpha',0.55,'FaceColor','interp');
colormap(ax3D, hot);
cb3d = colorbar(ax3D,'Location','eastoutside');
cb3d.Label.String = '\rho(x)  ground risk';
caxis(ax3D,[0 1]);

% --- Risk component subplots ---
axRunc = subplot(4,2,3,'Parent',pPlots);
hold(axRunc,'on'); grid(axRunc,'on');
xlabel(axRunc,"Time (s)"); ylabel(axRunc,"R^{unc}_u  (m)");
title(axRunc,"Uncertainty Overlap Risk  R^{unc}_u(t)  — per drone  (red = mean)");

axRmap = subplot(4,2,4,'Parent',pPlots);
hold(axRmap,'on'); grid(axRmap,'on');
xlabel(axRmap,"Time (s)"); ylabel(axRmap,"R^{map}_u  (m^2)");
title(axRmap,"Map Exposure Risk  R^{map}_u(t)  — per drone  (red = mean)");

axRvid = subplot(4,2,5,'Parent',pPlots);
hold(axRvid,'on'); grid(axRvid,'on');
xlabel(axRvid,"Time (s)"); ylabel(axRvid,"R^{vid}_u  [0,1]");
title(axRvid,"Video-Based Risk  R^{vid}_u(t)  — per drone  (red = mean)");
set(axRvid,'YLimMode','auto');

axRsys = subplot(4,2,6,'Parent',pPlots);
hold(axRsys,'on'); grid(axRsys,'on');
xlabel(axRsys,"Time (s)"); ylabel(axRsys,"R_{sys}");
title(axRsys,"System-Wide Risk  R_{sys}(t) = \Sigma_u R_u(t)");
hRsys = plot(axRsys,NaN,NaN,'k-','LineWidth',2);
yline(axRsys, R_bar_sys,'r--', ...
    'Label','Threshold','LabelHorizontalAlignment','left','FontSize',7);

axAoI = subplot(4,2,7,'Parent',pPlots);
hold(axAoI,'on'); grid(axAoI,'on');
xlabel(axAoI,"Time (s)"); ylabel(axAoI,"AoI (ms)");
title(axAoI,"Age of Information  — per drone  (red = mean)");
set(axAoI,'YLimMode','auto','YScale','linear');

axComm = subplot(4,2,8,'Parent',pPlots);
hold(axComm,'on'); grid(axComm,'on');
xlabel(axComm,"Time (s)");
title(axComm,"Payload Latency (ms, solid) & FDR×500 (dashed)  — per drone");
set(axComm,'YLimMode','auto');

%% Encolher subplots para caber o painel lateral
plotAreaW = 1 - panelW - 0.02;   % largura disponível com margem
axList = [ax3D, axRunc, axRmap, axRvid, axRsys, axAoI, axComm];
for ax = axList
    p = ax.Position;
    ax.Position = [p(1)*plotAreaW, p(2), p(3)*plotAreaW, p(4)];
end

%% Apply clean white/black styling to all plot axes
allAxes = [ax3D, axRunc, axRmap, axRvid, axRsys, axAoI, axComm];
for ax = allAxes
    set(ax, ...
        'Color',            'w', ...          % white plot background
        'XColor',           'k', ...          % black axis lines + tick labels
        'YColor',           'k', ...
        'ZColor',           'k', ...
        'GridColor',        [0.15 0.15 0.15], ...
        'MinorGridColor',   [0.25 0.25 0.25], ...
        'GridAlpha',        0.15, ...
        'FontSize',         8, ...
        'TitleFontWeight',  'bold');
    ax.Title.Color  = 'k';
    ax.XLabel.Color = 'k';
    ax.YLabel.Color = 'k';
    ax.ZLabel.Color = 'k';
end
% Colorbar label on 3D ax
cb3d.Color = 'k';

%% Per-drone line handles
colors = lines(numDrones);

hUnc = cell(numDrones,1);   hMap = cell(numDrones,1);
hVid = cell(numDrones,1);   hAoI = cell(numDrones,1);
hLat = cell(numDrones,1);   hFDR = cell(numDrones,1);

aoiTime=cell(numDrones,1); aoiVal=cell(numDrones,1);
latTime=cell(numDrones,1); latVal=cell(numDrones,1);
fdrTime=cell(numDrones,1); fdrVal=cell(numDrones,1);

for k = 1:numDrones
    hUnc{k} = plot(axRunc,NaN,NaN,'Color',colors(k,:),'LineWidth',0.8);
    hMap{k} = plot(axRmap,NaN,NaN,'Color',colors(k,:),'LineWidth',0.8);
    hVid{k} = plot(axRvid,NaN,NaN,'Color',colors(k,:),'LineWidth',0.8);
    hAoI{k} = plot(axAoI, NaN,NaN,'Color',colors(k,:));
    hLat{k} = plot(axComm,NaN,NaN,'-', 'Color',colors(k,:));
    hFDR{k} = plot(axComm,NaN,NaN,'--','Color',colors(k,:));
    aoiTime{k}=[]; aoiVal{k}=[];
    latTime{k}=[]; latVal{k}=[];
    fdrTime{k}=[]; fdrVal{k}=[];
end

% Thick red mean overlays
hUncMean = plot(axRunc,NaN,NaN,'r-','LineWidth',2.2);
hMapMean = plot(axRmap,NaN,NaN,'r-','LineWidth',2.2);
hVidMean = plot(axRvid,NaN,NaN,'r-','LineWidth',2.2);
hAoIMean = plot(axAoI, NaN,NaN,'r-','LineWidth',2.2);

plCfg = nrPathLossConfig;
plCfg.Scenario = "UMi";

%% =========================================================
% 10. MAIN SIMULATION LOOP
%% =========================================================

avgThroughput = ones(numDrones,1) * 1e-3;  % exponential moving average of throughput (Mbps)
rawThroughput = ones(numDrones,1) * 1e-3;   % ← add this
% riskUnc, riskMap, riskVid already exist — R_u per drone is computed each step
while advance(scene)

    time = scene.CurrentTime;
    dt   = 1/updateRate;

    %% Packet generation (Aviator traffic model)
    for i = 1:numDrones
        while time >= Traffic(i).nextPacketTime
            Traffic(i).queueMb        = Traffic(i).queueMb + lognrnd(7.4,0.6)*8/1e6;
            Traffic(i).nextPacketTime = Traffic(i).nextPacketTime + wblrnd(0.008,1.5);
        end
    end

    delete(findall(ax3D,'Tag','uncert'));

    %% Clear finished drones
    for i = 1:numDrones
        if time > endTimes(i) && ~isempty(aoiTime{i})
            set(hUnc{i},'XData',[],'YData',[]); set(hMap{i},'XData',[],'YData',[]);
            set(hVid{i},'XData',[],'YData',[]); set(hAoI{i},'XData',[],'YData',[]);
            set(hLat{i},'XData',[],'YData',[]); set(hFDR{i},'XData',[],'YData',[]);
            aoiTime{i}=[]; aoiVal{i}=[]; latTime{i}=[]; latVal{i}=[];
            fdrTime{i}=[]; fdrVal{i}=[]; riskTime{i}=[]; riskUnc{i}=[];
            riskMap{i}=[]; riskVid{i}=[]; lastReceivedTimestamp(i)=NaN;
            pendingMsg{i}.valid=false;
        end
    end

    %% Read positions
    activeDrones = [];
    activePos    = zeros(numDrones,3);

    for i = 1:numDrones
        if time < startTimes(i) || time > endTimes(i)
            pendingMsg{i}.valid=false; continue
        end
        [pos,~] = read(drones{i});
        if any(isnan(pos)), continue; end
        pendingMsg{i}.pos=pos(1:3); pendingMsg{i}.timestamp=time; pendingMsg{i}.valid=true;
        activeDrones(end+1)=i; %#ok<AGROW>
        activePos(i,:)=pos(1:3);
    end

    %% Scheduling
    [txSlots, rrPointer] = schedule_drones( ...
        activeDrones, dronesPerSlot, ...
        schedulingPolicy, rrPointer, ...
        lastReceivedTimestamp, time, ...
        avgThroughput,rawThroughput, ...
        riskUnc, riskMap, riskVid, ...
        w_unc, w_map, w_vid, cfg.sched_alpha);

    %% Pre-compute AoI radius for all active drones (needed for R_unc pairs)
    aoiRadius=zeros(numDrones,1);
    for i=activeDrones
        if isnan(lastReceivedTimestamp(i)), AoI_ms_i=0;
        else, AoI_ms_i=(time-lastReceivedTimestamp(i))*1000; end
        aoiRadius(i)=max(r_min + k_aoi*v_max*(AoI_ms_i/1000), r_min);
    end

    %% Per-drone processing
    for i = activeDrones

        pos      = pendingMsg{i}.pos;
        isMyTurn = ismember(i,txSlots);

        % Channel model
        if isMyTurn
            dist=sqrt(sum((microBSPos-pos).^2,2));
            [~,idxBS]=min(dist);
            posBS=[microBSPos(idxBS,2);microBSPos(idxBS,1);-microBSPos(idxBS,3)];
            posUE=[pos(2);pos(1);-pos(3)];
            PL   =nrPathLoss(plCfg,fc,true,posBS,posUE);
            SNR  =(pTransmitDrone+gNB_Gain-PL)-(thermalNoise+noiseFigure);
            snrLast(i) = SNR;
            Payload(i).throughput_Mbps=(bw*log2(1+10^(SNR/10)))/1e6;
            
            if SNR > thresholdSNR
                lastReceivedTimestamp(i) = time;
                alphaVal   = 0.4;
                txSuccess(i) = txSuccess(i) + 1;   % ← conta sucesso
            else
                alphaVal   = 0.1;
                txFail(i)  = txFail(i) + 1;        % ← conta falha
            end
        else
            Payload(i).throughput_Mbps=0; alphaVal=0.1;
        end
        rawThroughput(i) = Payload(i).throughput_Mbps;

        if ismember(i, txSlots)
            avgThroughput(i) = 0.9 * avgThroughput(i) + 0.1 * Payload(i).throughput_Mbps;
        else
            avgThroughput(i) = 0.9 * avgThroughput(i); % delivered rate = 0
        end

        % Payload / FDR
        genMb=Traffic(i).queueMb; Traffic(i).queueMb=0;
        capMb=Payload(i).throughput_Mbps*dt;
        if capMb>=genMb
            lat=latency_base+randn()*5;
            Payload(i).latency_ms=lat; Payload(i).lastLatency_ms=lat;
            Payload(i).frameDelivery=1;
        else
            Payload(i).latency_ms   =Payload(i).lastLatency_ms;
            Payload(i).frameDelivery=min(max(capMb/max(genMb,eps),0),1);
        end

        % AoI
        if isnan(lastReceivedTimestamp(i)), AoI_ms=0;
        else, AoI_ms=(time-lastReceivedTimestamp(i))*1000; end
        r_u=aoiRadius(i);

        %% =================================================
        %%  RISK COMPONENTS  (Sections 4.1 – 4.3 in LaTeX)
        %% =================================================

        %% R_unc: Uncertainty Overlap Risk
        %  r_u(t) = v_max * h_u(t)      [AoI radius]
        %  R_u^unc = sum_{v!=u} [r_u + r_v - d_uv]_+
        R_unc = 0;
        for j = activeDrones
            if j==i, continue; end
            d_uv = norm(pos(1:2) - activePos(j,1:2));
            R_unc = R_unc + max(r_u + aoiRadius(j) - d_uv,  0);
        end

        %% R_map: Map Exposure Risk
        %  B_u(t) = { x : ||x - x_hat_u|| <= r_u }
        %  R_u^map = sum_{c in B_u} rho_c * deltaA
        inSphere = sqrt((xFlat-pos(1)).^2 + (yFlat-pos(2)).^2) <= r_u;
        R_map    = sum(rhoFlat(inSphere)) * deltaA;

        %% R_vid: Video-Based Risk
        %  d_u = min_{v!=u} ||x_u - x_v||
        %  chi(d_u) = min(1, d_crit / (d_u + eps))
        %  R_u^vid = max(1 - FDR, tau/tau_max) * chi
        sepVec = arrayfun(@(j) norm(pos(1:2)-activePos(j,1:2)), activeDrones);
        sepVec(activeDrones==i) = Inf;
        d_u  = min(sepVec);
        chi  = min(1, d_crit/(d_u + eps_sep));
        R_vid = max(1 - Payload(i).frameDelivery, ...
                    Payload(i).latency_ms/tau_max) * chi;

        %% Total R_u(t)
        R_u = w_unc*R_unc + w_map*R_map + w_vid*R_vid;

        % Store risk
        riskTime{i}(end+1)=time; riskUnc{i}(end+1)=R_unc;
        riskMap{i}(end+1) =R_map; riskVid{i}(end+1)=R_vid;

        set(hUnc{i},'XData',riskTime{i},'YData',riskUnc{i});
        set(hMap{i},'XData',riskTime{i},'YData',riskMap{i});
        set(hVid{i},'XData',riskTime{i},'YData',riskVid{i});

        % AoI
        aoiTime{i}(end+1)=time; aoiVal{i}(end+1)=AoI_ms;
        set(hAoI{i},'XData',aoiTime{i},'YData',aoiVal{i});

        % Latency & FDR (FDR scaled ×500 to share axis with latency)
        latTime{i}(end+1)=time; latVal{i}(end+1)=Payload(i).latency_ms;
        fdrTime{i}(end+1)=time; fdrVal{i}(end+1)=Payload(i).frameDelivery*500;
        set(hLat{i},'XData',latTime{i},'YData',latVal{i});
        set(hFDR{i},'XData',fdrTime{i},'YData',fdrVal{i});

        % Uncertainty sphere in 3D
        [Xs,Ys,Zs]=sphere(12);
        surf(ax3D, r_u*Xs+pos(1), r_u*Ys+pos(2), r_u*Zs+pos(3), ...
            'FaceAlpha',alphaVal,'EdgeColor','none', ...
            'FaceColor',colors(i,:),'Tag','uncert');

    end  % end per-drone loop

    %% Mean overlays + R_sys
    activeMask = arrayfun(@(i) pendingMsg{i}.valid, 1:numDrones);
    if any(activeMask)
        activeIdx = find(activeMask);

        uncMeanT(end+1)=time; uncMeanV(end+1)=mean(arrayfun(@(i) riskUnc{i}(end),activeIdx));
        mapMeanT(end+1)=time; mapMeanV(end+1)=mean(arrayfun(@(i) riskMap{i}(end),activeIdx));
        vidMeanT(end+1)=time; vidMeanV(end+1)=mean(arrayfun(@(i) riskVid{i}(end),activeIdx));
        aoiMeanT(end+1)=time; aoiMeanV(end+1)=mean(arrayfun(@(i) aoiVal{i}(end),  activeIdx));

        set(hUncMean,'XData',uncMeanT,'YData',uncMeanV);
        set(hMapMean,'XData',mapMeanT,'YData',mapMeanV);
        set(hVidMean,'XData',vidMeanT,'YData',vidMeanV);
        set(hAoIMean,'XData',aoiMeanT,'YData',aoiMeanV);

        % R_sys = sum_u R_u(t)
        R_sys = sum(arrayfun(@(i) ...
            w_unc*riskUnc{i}(end)+w_map*riskMap{i}(end)+w_vid*riskVid{i}(end), activeIdx));
        rSysTime(end+1)=time; rSysData(end+1)=R_sys;
        set(hRsys,'XData',rSysTime,'YData',rSysData);
    end

    % 3D update
    show3D(scene, Parent=ax3D);
    xlim(ax3D,[-2000 2000]); ylim(ax3D,[-700 700]); zlim(ax3D,[-150 50]);
    view(ax3D,-35,25);

    %% -------------------------------------------------------
    %%  STATUS BAR — live indicator update
    %% -------------------------------------------------------
    activeMaskSB = arrayfun(@(i) pendingMsg{i}.valid, 1:numDrones);
    activeIdxSB  = find(activeMaskSB);
    nAct         = numel(activeIdxSB);

    % 1. Active Drones
    set(hTileVal(IDX_ACTIVE), 'String', sprintf('%d / %d', nAct, numDrones));

    % 2. Scheduled / slot (already static but refresh shows current txSlots count)
    set(hTileVal(IDX_SCHED), 'String', ...
        sprintf('%d / slot  (%d tx)', dronesPerSlot, numel(txSlots)));

    % 3. Scheduling policy — static, already set

    if nAct > 0
        % 4. Mean FDR — colour-coded
        meanFDR = mean([Payload(activeIdxSB).frameDelivery]);
        if meanFDR >= 0.90
            fdrColor = [0.30 0.90 0.45];   % green
        elseif meanFDR >= 0.70
            fdrColor = accentWarn;
        else
            fdrColor = accentDanger;
        end
        set(hTileVal(IDX_FDR), 'String', sprintf('%.1f %%', meanFDR*100), ...
            'ForegroundColor', fdrColor);

        % 5. Mean Latency
        meanLat = mean([Payload(activeIdxSB).latency_ms]);
        if meanLat < 200
            latColor = [0.30 0.90 0.45];
        elseif meanLat < 400
            latColor = accentWarn;
        else
            latColor = accentDanger;
        end
        set(hTileVal(IDX_LAT), 'String', sprintf('%.0f ms', meanLat), ...
            'ForegroundColor', latColor);

        % 6. Mean AoI
        if ~isempty(aoiMeanV)
            curAoI = aoiMeanV(end);
        else
            curAoI = 0;
        end
        if curAoI < 500
            aoiColor = [0.30 0.90 0.45];
        elseif curAoI < 1500
            aoiColor = accentWarn;
        else
            aoiColor = accentDanger;
        end
        set(hTileVal(IDX_AOI), 'String', sprintf('%.0f ms', curAoI), ...
            'ForegroundColor', aoiColor);

        % 7. R_sys value
        if ~isempty(rSysData)
            curRsys = rSysData(end);
            set(hTileVal(IDX_RSYS), 'String', sprintf('%.1e', curRsys));
            % 8. Risk Level badge
            ratio = curRsys / R_bar_sys;
            if ratio < 0.5
                lvlStr = 'LOW';      lvlColor = [0.30 0.90 0.45];
                set(hTileBG(IDX_LEVEL),'HighlightColor',[0.20 0.50 0.25]);
            elseif ratio < 0.85
                lvlStr = 'MODERATE'; lvlColor = accentWarn;
                set(hTileBG(IDX_LEVEL),'HighlightColor',[0.60 0.45 0.05]);
            elseif ratio < 1.0
                lvlStr = 'HIGH';     lvlColor = [1.0 0.50 0.10];
                set(hTileBG(IDX_LEVEL),'HighlightColor',[0.70 0.30 0.05]);
            else
                lvlStr = 'CRITICAL'; lvlColor = accentDanger;
                set(hTileBG(IDX_LEVEL),'HighlightColor',[0.80 0.10 0.10]);
            end
            set(hTileVal(IDX_LEVEL),'String',lvlStr,'ForegroundColor',lvlColor);
            % R_sys colour mirrors risk level
            set(hTileVal(IDX_RSYS),'ForegroundColor',lvlColor);
        end
    end

    % 9. Sim Time  mm:ss.ss
    simMins = floor(time/60);
    set(hTileVal(IDX_TIME), 'String', sprintf('%02d:%05.2f', simMins, mod(time,60)));

    %% -------------------------------------------------------
    %%  PAINEL DE STATUS DOS DRONES — atualizar
    %% -------------------------------------------------------
    droneStatusLines = cell(numDrones, 1);
    for k = 1:numDrones
        inTx = ismember(k, txSlots);
        droneStatusLines{k} = droneStatusLine(k, time, startTimes, endTimes, ...
                                       inTx, lastReceivedTimestamp, aoiVal, ...
                                       fdrVal, txSuccess, txFail,snrLast, thresholdSNR);
    end
    set(hDroneList, 'String', droneStatusLines);
    drawnow limitrate;

end  % end while advance(scene)

disp("Simulation complete.");

%% =========================================================
% 11. POST-SIMULATION SUMMARY FIGURE
%% =========================================================
figure('Color','w','Position',[200 80 1000 720]);
sgtitle("Safety Risk Summary","FontSize",14,"FontWeight","bold");

% Heatmap
subplot(2,3,1);
imagesc(xVec, yVec, rhoMap); colormap(hot); colorbar; axis xy;
hold on;
scatter(hotX, hotY, 60, 'w', 'filled', 'MarkerEdgeColor','k');
xlabel("North (m)"); ylabel("East (m)");
title("Ground Risk Heatmap  \rho(x)  (circles = hotspot centres)");

% Mean R_unc
subplot(2,3,2);
plot(uncMeanT, uncMeanV, 'b-', 'LineWidth',1.8); grid on;
xlabel("Time (s)"); ylabel("\langle R^{unc} \rangle  (m)");
title("Mean Uncertainty Overlap Risk");

% Mean R_map
subplot(2,3,3);
plot(mapMeanT, mapMeanV, 'Color',[0.85 0.33 0.1], 'LineWidth',1.8); grid on;
xlabel("Time (s)"); ylabel("\langle R^{map} \rangle  (m^2)");
title("Mean Map Exposure Risk");

% Mean R_vid
subplot(2,3,4);
plot(vidMeanT, vidMeanV, 'm-', 'LineWidth',1.8); grid on;
xlabel("Time (s)"); ylabel("\langle R^{vid} \rangle  [0,1]");
title("Mean Video-Based Risk");

% R_sys
subplot(2,3,5);
plot(rSysTime, rSysData, 'k-', 'LineWidth',2); grid on; hold on;
yline(R_bar_sys,'r--','Threshold','LabelHorizontalAlignment','left');
xlabel("Time (s)"); ylabel("R_{sys}");
title("System-Wide Risk  R_{sys}(t)");

% Stacked weighted breakdown
subplot(2,3,6);
tC = uncMeanT;
area(tC, [w_unc*uncMeanV(:)  w_map*mapMeanV(:)  w_vid*vidMeanV(:)]);
colororder([0 0.45 0.74; 0.85 0.33 0.1; 0.49 0.18 0.56]);
legend("w_{unc}\cdotR^{unc}","w_{map}\cdotR^{map}","w_{vid}\cdotR^{vid}", ...
       'Location','northwest','FontSize',8);
grid on; xlabel("Time (s)"); ylabel("Weighted Risk");
title("Weighted Risk Component Breakdown (mean)");

end   % main function

%% =========================================================
% HELPER: keep status bar at fixed pixel height; plot panel fills the rest
%% =========================================================
function resizePanels(fig, pStatus, pPlots, barHeightPx)
    figPos = fig.Position;          % [x y w h] in pixels
    figH   = figPos(4);
    if figH < 1, figH = 1; end
    barFrac  = barHeightPx / figH;  % fraction of figure height
    barFrac  = min(barFrac, 0.15);  % safety clamp

    pStatus.Units    = 'normalized';
    pStatus.Position = [0,  0,        1,  barFrac];
    pPlots.Units     = 'normalized';
    pPlots.Position  = [0,  barFrac,  1,  1 - barFrac];
end

%% =========================================================
% HELPER: recentre indicator tiles horizontally inside pStatus
%% =========================================================
function recentreTiles(pStatus, hTileBG, tileW, tileGap, tileY)
    pStatus.Units = 'pixels';
    barW  = pStatus.Position(3);
    pStatus.Units = 'normalized';
    n     = numel(hTileBG);
    total = n * tileW + (n-1) * tileGap;
    x0    = max((barW - total) / 2, 4);
    for ti = 1:n
        hTileBG(ti).Units    = 'pixels';
        hTileBG(ti).Position = [x0 + (ti-1)*(tileW+tileGap),  tileY,  tileW,  68];
    end
end

function str = droneStatusLine(k, time, startTimes, endTimes, ...
                                isInTxSlot, lastReceivedTimestamp, aoiVal, ...
                                fdrVal, txSuccess, txFail, snrLast, thresholdSNR)

%% Estado
if time < startTimes(k)
    statusStr = 'AG.';
    pct = 0;
elseif time > endTimes(k)
    statusStr = 'CO';
    pct = 100;
elseif isInTxSlot
    statusStr = 'TR';
    pct = (time - startTimes(k)) / (endTimes(k) - startTimes(k)) * 100;
else
    statusStr = 'Fl';
    pct = (time - startTimes(k)) / (endTimes(k) - startTimes(k)) * 100;
end

%% Barra de progresso ASCII (8 chars)
filled = round(pct / 12.5);
bar    = [repmat(char(9608), 1, filled), repmat(char(9617), 1, 8-filled)];

%% AoI
if isnan(lastReceivedTimestamp(k)) || time < startTimes(k) || time > endTimes(k)
    aoiStr = '    --- ';
elseif ~isempty(aoiVal{k})
    aoi_ms = aoiVal{k}(end);
    if aoi_ms >= 1000
        aoiStr = sprintf('%5.1fs  ', aoi_ms/1000);
    else
        aoiStr = sprintf('%5.0fms ', aoi_ms);
    end
else
    aoiStr = ' --- ';
end

%% FDR
if time < startTimes(k) || isempty(fdrVal{k})
    fdrStr = ' ---';
else
    fdr_pct = fdrVal{k}(end) / 500 * 100;
    fdrStr  = sprintf('%4.0f%%', fdr_pct);
end

%% SNR — só disponível nos slots de TX; mostra último valor conhecido
if isnan(snrLast(k)) || time < startTimes(k)
    snrStr = ' ---';
else
    % Marca com * se abaixo do threshold
    if snrLast(k) >= thresholdSNR
        snrStr = sprintf('%+6.1fdB ', snrLast(k));
    else
        snrStr = sprintf('%+6.1fdB*', snrLast(k));   % * = abaixo do threshold
    end
end

%% Contadores
okStr  = sprintf('%4d', txSuccess(k));
errStr = sprintf('%4d', txFail(k));

str = sprintf('U%02d  %-9s  %s  %s | F:%s  ok:%s  e:%s  S:%s', ...
              k, statusStr, bar, aoiStr, fdrStr, okStr, errStr, snrStr);
end