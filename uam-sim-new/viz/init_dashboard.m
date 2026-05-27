function db = init_dashboard(cfg, rhoMap, mapGrid)
% INIT_DASHBOARD  Dashboard principal (sem 3D, sem sidebar, sem Path Loss).
%
%  Layout 4×3 tiledlayout:
%    Row 1: top-view (cols 1-2)  |  SNR vs North (col 3)
%    Row 2: R_unc (cols 1-2)     |  R_map (col 3) — wait, use [1 2]+[1]
%    Row 3: R_vid (cols 1-2)     |  R_sys (col 3)
%    Row 4: h1 AoI (cols 1-2)   |  h2 AoI (col 3)
%  + Janela separada: Drone Status Panel

N = cfg.numDrones;

% ── Figura principal (sem painel lateral) ────────────────────────────
db.fig = figure('Color','w','Position',[40 30 1200 1000], ...
    'Name',sprintf('UAM Dashboard — %d drones — %s', N, cfg.schedulingPolicy), ...
    'NumberTitle','off');

% Status bar no fundo
statusBarH = 86;
db.pStatus = uipanel(db.fig, ...
    'Units','normalized','Position',[0 0 1 0.001], ...
    'BackgroundColor',[0.07 0.08 0.10], ...
    'BorderType','line','HighlightColor',[0.25 0.60 0.90],'BorderWidth',1);
db.pPlots = uipanel(db.fig, ...
    'Units','normalized','Position',[0 0.001 1 0.999], ...
    'BackgroundColor','w','BorderType','none');
set(db.fig,'SizeChangedFcn',@(src,~) resize_panels(src,db.pStatus,db.pPlots,statusBarH));
resize_panels(db.fig,db.pStatus,db.pPlots,statusBarH);

% ── Tiles da status bar ──────────────────────────────────────────────
indicatorLabels = {'ACTIVE DRONES','SCHED / SLOT','POLICY', ...
    'h1  C2 AoI','h2  Vid AoI','R vid', ...
    'R sys','RISK LEVEL','SIM TIME'};
numTiles = numel(indicatorLabels);
tileW=142; tileGap=5; tileY=9;
accentColor=[0.30 0.85 0.65];
db.hTileBG=gobjects(numTiles,1); db.hTileLbl=gobjects(numTiles,1); db.hTileVal=gobjects(numTiles,1);
for ti=1:numTiles
    db.hTileBG(ti)=uipanel(db.pStatus,'Units','pixels', ...
        'Position',[4+(ti-1)*(tileW+tileGap) tileY tileW 68], ...
        'BackgroundColor',[0.12 0.13 0.16],'BorderType','line', ...
        'HighlightColor',[0.22 0.24 0.30],'BorderWidth',1);
    db.hTileLbl(ti)=uicontrol(db.hTileBG(ti),'Style','text', ...
        'Units','normalized','Position',[0.04 0.62 0.92 0.30], ...
        'String',indicatorLabels{ti},'FontSize',7, ...
        'ForegroundColor',[0.50 0.55 0.65],'BackgroundColor',[0.12 0.13 0.16], ...
        'HorizontalAlignment','left');
    db.hTileVal(ti)=uicontrol(db.hTileBG(ti),'Style','text', ...
        'Units','normalized','Position',[0.04 0.06 0.92 0.52], ...
        'String','---','FontSize',11,'FontWeight','bold', ...
        'ForegroundColor',accentColor,'BackgroundColor',[0.12 0.13 0.16], ...
        'HorizontalAlignment','center');
end
db.IDX_ACTIVE=1; db.IDX_SCHED=2; db.IDX_POLICY=3;
db.IDX_H1=4;     db.IDX_H2=5;    db.IDX_RVID=6;
db.IDX_RSYS=7;   db.IDX_LEVEL=8; db.IDX_TIME=9;
set(db.hTileVal(db.IDX_POLICY),'String',cfg.schedulingPolicy,'ForegroundColor',[0.40 0.85 0.55]);

% ── Layout 4×3 (sem sidebar, ocupa 100% da largura) ─────────────────
tl = tiledlayout(db.pPlots, 4, 3, 'TileSpacing','compact','Padding','compact');

axprop = {'Color',[0.07 0.08 0.10],'XColor','w','YColor','w', ...
          'GridColor',[0.3 0.3 0.3],'GridAlpha',0.5};

% ── Row 1, Col 1-2: Visão aérea ─────────────────────────────────────
db.axMap = nexttile(tl, 1, [1 2]);
hold(db.axMap,'on'); grid(db.axMap,'on');
set(db.axMap, axprop{:});

surf(db.axMap, mapGrid.XG, mapGrid.YG, zeros(size(mapGrid.XG)), rhoMap, ...
     'EdgeColor','none','FaceAlpha',0.50,'FaceColor','interp','Tag','heatmap');
colormap(db.axMap, hot);
cb=colorbar(db.axMap,'Location','eastoutside'); cb.Color='w';
cb.Label.String='\rho(x)  ground risk'; cb.Label.Color='w';
clim(db.axMap,[0 1]);

% micro-BS: builder exports [East, North, Alt] → plot as (North=col2, East=col1)
for b=1:size(cfg.microBSPos,1)
    bN = cfg.microBSPos(b,2);   % North = col2
    bE = cfg.microBSPos(b,1);   % East  = col1
    plot(db.axMap, bN, bE, '^', ...
        'Color',[0.3 0.7 1],'MarkerSize',11,'MarkerFaceColor',[0.3 0.7 1], ...
        'DisplayName',sprintf('BS %d',b));
    text(db.axMap, bN+20, bE+20, sprintf('BS%d',b), 'Color',[0.3 0.7 1],'FontSize',8);
end

db.colors = lines(N);
db.hMapTrail  = gobjects(N,1);
db.hMapAoiCirc= gobjects(N,1);
db.hMapMarker = gobjects(N,1);
for k=1:N
    db.hMapTrail(k)   = plot(db.axMap,NaN,NaN,'-','Color',[db.colors(k,:) 0.30],'LineWidth',1);
    db.hMapAoiCirc(k) = plot(db.axMap,NaN,NaN,'--','Color',db.colors(k,:),'LineWidth',1.4);
    db.hMapMarker(k)  = plot(db.axMap,NaN,NaN,'o','Color',db.colors(k,:), ...
        'MarkerSize',9,'MarkerFaceColor',db.colors(k,:),'DisplayName',sprintf('UAV %d',k));
end
xlabel(db.axMap,'North (m)','Color','w');
ylabel(db.axMap,'East (m)','Color','w');
title(db.axMap,'Aerial View — AoI uncertainty radius','Color','w','FontWeight','bold');
legend(db.axMap,'TextColor','w','Color',[0.10 0.11 0.14],'Location','northwest','FontSize',7);
% Axis limits match the heatmap exactly (no distortion, no equal-axis stretch)
xlim(db.axMap, mapGrid.mapXlim);
ylim(db.axMap, mapGrid.mapYlim);
db.heatmap = struct('XG',mapGrid.XG,'YG',mapGrid.YG, ...
    'Z',zeros(size(mapGrid.XG)),'rhoMap',rhoMap);
db.posNorthHist=cell(N,1); db.posEastHist=cell(N,1);
for k=1:N, db.posNorthHist{k}=[]; db.posEastHist{k}=[]; end

% ── Row 1, Col 3: SNR vs North ──────────────────────────────────────
db.axSNR = nexttile(tl, 3);
hold(db.axSNR,'on'); grid(db.axSNR,'on');
set(db.axSNR, axprop{:});
yline(db.axSNR,cfg.thresholdSNR,'r--','Thresh', ...
    'LabelHorizontalAlignment','left','LabelColor','w','LineWidth',1.2);
db.hSNR = gobjects(N,1);
for k=1:N
    db.hSNR(k)=plot(db.axSNR,NaN,NaN,'.','Color',db.colors(k,:),'MarkerSize',3);
end
xlabel(db.axSNR,'North (m)','Color','w');
ylabel(db.axSNR,'SNR (dB)','Color','w');
title(db.axSNR,'SNR vs Position','Color','w','FontWeight','bold');
db.snrNorthHist=cell(N,1); db.snrValHist=cell(N,1);
for k=1:N, db.snrNorthHist{k}=[]; db.snrValHist{k}=[]; end

% ── Row 2: R_unc (cols 1-2) | R_map (col 3) ────────────────────────
db.axRunc = nexttile(tl, 4, [1 2]);
hold(db.axRunc,'on'); grid(db.axRunc,'on');
xlabel(db.axRunc,'Time (slots)'); ylabel(db.axRunc,'R^{Nav}_u');
title(db.axRunc,'Navigation Overlap Risk');

db.axRmap = nexttile(tl, 6);
hold(db.axRmap,'on'); grid(db.axRmap,'on');
xlabel(db.axRmap,'Time (slots)'); ylabel(db.axRmap,'R^{Ground}_u');
title(db.axRmap,'Ground Exposure Risk');

% ── Row 3: R_vid (cols 1-2) | R_sys (col 3) ────────────────────────
db.axRvid = nexttile(tl, 7, [1 2]);
hold(db.axRvid,'on'); grid(db.axRvid,'on');
xlabel(db.axRvid,'Time (slots)'); ylabel(db.axRvid,'R^{vid}_u');
title(db.axRvid,'Video Coherence Risk');

db.axRsys = nexttile(tl, 9);
hold(db.axRsys,'on'); grid(db.axRsys,'on');
xlabel(db.axRsys,'Time (slots)'); ylabel(db.axRsys,'R_{sys}');
title(db.axRsys,'System-Wide Risk R_{sys}(t)');
db.hRsys = plot(db.axRsys,NaN,NaN,'k-','LineWidth',1.8);

% ── Row 4: h1 AoI (cols 1-2) | h2 AoI (col 3) ─────────────────────
db.axH1 = nexttile(tl, 10, [1 2]);
hold(db.axH1,'on'); grid(db.axH1,'on');
xlabel(db.axH1,'Time (slots)'); ylabel(db.axH1,'h_1 (slots)');
title(db.axH1,'C2 / Telemetry AoI  [slots]');

db.axH2 = nexttile(tl, 12);
hold(db.axH2,'on'); grid(db.axH2,'on');
xlabel(db.axH2,'Time (slots)'); ylabel(db.axH2,'h_2 (slots)');
title(db.axH2,'Video AoI  [slots]');

for ax=[db.axRunc db.axRmap db.axRvid db.axRsys db.axH1 db.axH2]
    set(ax,'FontSize',8,'TitleFontWeight','bold');
end

% Per-drone line handles
db.hUnc=cell(N,1); db.hMap=cell(N,1); db.hVid=cell(N,1);
db.hH1=cell(N,1);  db.hH2=cell(N,1);
for k=1:N
    db.hUnc{k}=plot(db.axRunc,NaN,NaN,'Color',db.colors(k,:),'LineWidth',0.8);
    db.hMap{k}=plot(db.axRmap,NaN,NaN,'Color',db.colors(k,:),'LineWidth',0.8);
    db.hVid{k}=plot(db.axRvid,NaN,NaN,'Color',db.colors(k,:),'LineWidth',0.8);
    db.hH1{k} =plot(db.axH1,  NaN,NaN,'Color',db.colors(k,:),'LineWidth',0.8);
    db.hH2{k} =plot(db.axH2,  NaN,NaN,'Color',db.colors(k,:),'LineWidth',0.8);
end
db.hUncMean=plot(db.axRunc,NaN,NaN,'r-','LineWidth',2.2);
db.hMapMean=plot(db.axRmap,NaN,NaN,'r-','LineWidth',2.2);
db.hVidMean=plot(db.axRvid,NaN,NaN,'r-','LineWidth',2.2);
db.hH1Mean =plot(db.axH1,  NaN,NaN,'r-','LineWidth',2.2);
db.hH2Mean =plot(db.axH2,  NaN,NaN,'r-','LineWidth',2.2);

% ── Janela separada: Drone Status ────────────────────────────────────
db.fig_status = figure('Color',[0.07 0.08 0.10], ...
    'Position',[1260 30 380 700], ...
    'Name','Drone Status','NumberTitle','off', ...
    'MenuBar','none','ToolBar','none');
uicontrol(db.fig_status,'Style','text', ...
    'Units','normalized','Position',[0 0.95 1 0.05], ...
    'String','■ DRONE STATUS — live', ...
    'BackgroundColor',[0.07 0.08 0.10], ...
    'ForegroundColor',[0.30 0.85 0.65], ...
    'FontName','Courier New','FontSize',10,'FontWeight','bold', ...
    'HorizontalAlignment','center');
db.hDroneList = uicontrol(db.fig_status,'Style','listbox', ...
    'Units','normalized','Position',[0.01 0.01 0.98 0.93], ...
    'BackgroundColor',[0.07 0.08 0.10], ...
    'ForegroundColor',[0.75 0.80 0.90], ...
    'FontName','Courier New','FontSize',9, ...
    'String',{'Awaiting data...'},'Enable','inactive','Max',2);

db.statusBarH = statusBarH;
end

function resize_panels(fig,pStatus,pPlots,barHeightPx)
    figPos=fig.Position; figH=max(figPos(4),1);
    barFrac=min(barHeightPx/figH,0.15);
    pStatus.Units='normalized'; pStatus.Position=[0 0 1 barFrac];
    pPlots.Units='normalized';  pPlots.Position=[0 barFrac 1 1-barFrac];
end
