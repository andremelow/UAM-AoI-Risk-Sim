function gnb_density_demo_geo()
%GNB_DENSITY_DEMO_GEO  Interactive validator over a real OSM basemap.
%
%   Same controls as gnb_density_demo, but the ground-truth panel and
%   the density panel are both drawn over an OpenStreetMap basemap
%   centred on 23 deg 42'06.9"S, 46 deg 42'03.4"W (Interlagos area, SP).
%   The figure follows the standard cartographic convention: East on the
%   horizontal axis (X), North on the vertical axis (Y), so the map is
%   not mirrored relative to printed maps. UE positions and the gNB are
%   plotted as (East, North).
%
%   The density colormap goes light-red -> dark-red (no yellow, no
%   white), so that bright cells read as "more people" without the
%   yellow/white tip of the standard 'hot' map.
%
%   Requires osm_basemap.m next to this file.

if ~exist('osm_basemap','file')
    error('gnb_density_demo_geo:missingDep', ...
        'osm_basemap.m must be on the MATLAB path (same folder as this file).');
end

%% =========================================================
%  Geo and scenario constants
%% =========================================================
centerLat = -(23 + 42/60 + 06.9/3600);
centerLon = -(46 + 42/60 + 03.4/3600);

% Axes are in metres relative to (centerLat, centerLon):
%   X axis (horizontal on screen) = East
%   Y axis (vertical on screen)   = North
halfEast  = 1600;       % half-width on the East axis (m)
halfNorth = 600;        % half-width on the North axis (m)
mapXlim   = [-halfEast,  +halfEast];     % East range
mapYlim   = [-halfNorth, +halfNorth];    % North range
zoomLvl   = 16;
cellSize  = 50;

xVec = (mapXlim(1)+cellSize/2):cellSize:(mapXlim(2)-cellSize/2);   % East cells
yVec = (mapYlim(1)+cellSize/2):cellSize:(mapYlim(2)-cellSize/2);   % North cells
[XG, YG] = meshgrid(xVec, yVec);    % XG = East coord, YG = North coord

%% Single serving gNB (placed at the local origin, on the ground plane)
gnb = struct('pos',[0, 0, -40], 'cellId',1);   % .pos = [East, North, Down]

%% Radio + estimator parameters
params = struct( ...
    'n',             2.7,         ...
    'PL0_dB',        38,          ...
    'Pt_dBm',        23,          ...
    'Gt_dB',         8,           ...
    'Gr_dB',         0,           ...
    'sigma_xi_dB',   6,           ...
    'sigma_phi_rad', deg2rad(3),  ...
    'd_min',         10,          ...
    'd_max',         1800,        ...
    'cap_sigma_m',   250,         ...
    'rho_op',        5e-4 );

%% Download / fetch OSM basemap (cached)
fprintf('Loading OSM tiles for (%.5f, %.5f)... ', centerLat, centerLon);
% osm_basemap takes (centerLat, centerLon, halfX_North, halfY_East). The
% returned image rows are indexed by metre-North, columns by metre-East.
[bgRGB, ~, ~] = osm_basemap(centerLat, centerLon, halfNorth, halfEast, zoomLvl);
fprintf('done.\n');

%% =========================================================
%  Figure layout
%% =========================================================
fig = figure('Color','w','Position',[60 60 1500 760], ...
    'Name','gNB Local Density Demo  -  OSM basemap (Interlagos)', ...
    'NumberTitle','off');

axL = axes('Parent',fig,'Position',[0.04 0.22 0.44 0.72]);
axR = axes('Parent',fig,'Position',[0.52 0.22 0.44 0.72]);

drawBasemap(axL, bgRGB, mapXlim, mapYlim);
drawBasemap(axR, bgRGB, mapXlim, mapYlim);

% ---- LEFT panel: ground truth ----
title(axL,'Ground truth  (left-click add, drag move, right-click remove)');
xlabel(axL,'East (m)');  ylabel(axL,'North (m)');

hold(axL,'on');
plot(axL, gnb.pos(1), gnb.pos(2), 'r^','MarkerSize',16, ...
     'MarkerFaceColor','r','LineWidth',1.5);
text(axL, gnb.pos(1)+30, gnb.pos(2)+30, 'gNB', ...
     'FontWeight','bold','Color','r');
hUE = scatter(axL, [], [], 60, 'filled', ...
              'MarkerFaceColor',[0.15 0.55 0.95], ...
              'MarkerEdgeColor','k', 'LineWidth',0.8);

% ---- RIGHT panel: recovered density (semi-transparent heatmap) ----
title(axR,'$\bar{\rho}(x,t)$ recovered from RSRP + AoA', ...
      'Interpreter','latex');
xlabel(axR,'East (m)');  ylabel(axR,'North (m)');

hold(axR,'on');
% imagesc(x, y, C): rows of C are plotted along y, cols along x.
% Our density map has rows = North (yVec) and cols = East (xVec), so
% the call is imagesc(xVec, yVec, rhoMap) -- already aligned with
% North-up display.
hImg = imagesc(axR, xVec, yVec, zeros(size(XG)));
set(hImg,'AlphaData', zeros(size(XG)));     % start fully transparent
caxis(axR,[0 1]);
colormap(axR, red_ramp(256));               % light red -> dark red
cb = colorbar(axR);
cb.Label.String = '$\bar{\rho} \in [0,1]$';
cb.Label.Interpreter = 'latex';

plot(axR, gnb.pos(1), gnb.pos(2), '^','MarkerSize',16, ...
     'MarkerFaceColor','w','MarkerEdgeColor','k','LineWidth',1.2);

linkaxes([axL axR],'xy');

%% =========================================================
%  Bottom UI bar
%% =========================================================
uicontrol(fig,'Style','pushbutton','String','Clear UEs', ...
    'Units','pixels','Position',[40 30 100 30], ...
    'Callback',@(s,e) onClear(fig));

uicontrol(fig,'Style','pushbutton','String','Add 20 random', ...
    'Units','pixels','Position',[150 30 130 30], ...
    'Callback',@(s,e) onAddRandom(fig,20));

uicontrol(fig,'Style','pushbutton','String','Resample noise', ...
    'Units','pixels','Position',[290 30 130 30], ...
    'Callback',@(s,e) onResample(fig));

uicontrol(fig,'Style','text','String','sigma_xi (dB):', ...
    'Position',[450 26 90 22],'BackgroundColor','w', ...
    'HorizontalAlignment','right','FontWeight','bold');
uicontrol(fig,'Style','slider','Min',0.5,'Max',12,'Value',6, ...
    'Position',[545 30 150 22], ...
    'Callback',@(s,e) onSlider(fig,'sigma_xi_dB',s.Value));

uicontrol(fig,'Style','text','String','sigma_phi (deg):', ...
    'Position',[710 26 100 22],'BackgroundColor','w', ...
    'HorizontalAlignment','right','FontWeight','bold');
uicontrol(fig,'Style','slider','Min',0.5,'Max',30,'Value',3, ...
    'Position',[815 30 150 22], ...
    'Callback',@(s,e) onSlider(fig,'sigma_phi_deg',s.Value));

uicontrol(fig,'Style','text','String','heatmap alpha:', ...
    'Position',[980 26 100 22],'BackgroundColor','w', ...
    'HorizontalAlignment','right','FontWeight','bold');
uicontrol(fig,'Style','slider','Min',0.0,'Max',1.0,'Value',0.65, ...
    'Position',[1085 30 110 22], ...
    'Callback',@(s,e) onSlider(fig,'alpha',s.Value));

hStatus = uicontrol(fig,'Style','text', ...
    'String','0 UEs   |   integrated mass: 0.00   (target: 0)', ...
    'Position',[1210 28 270 24], 'BackgroundColor','w', ...
    'HorizontalAlignment','left','FontWeight','bold');

%% =========================================================
%  Stash state and register window callbacks
%% =========================================================
data.gnb       = gnb;
data.params    = params;
data.alpha     = 0.65;
data.XG        = XG;     data.YG    = YG;
data.xVec      = xVec;   data.yVec  = yVec;
data.mapXlim   = mapXlim;  data.mapYlim = mapYlim;
data.cellSize  = cellSize;
data.ueTrue    = zeros(0,2);    % column 1 = East, column 2 = North
data.axL       = axL;    data.axR  = axR;
data.hUE       = hUE;    data.hImg = hImg;
data.hStatus   = hStatus;
data.dragging  = 0;
data.centerLat = centerLat;  data.centerLon = centerLon;
guidata(fig,data);

set(fig,'WindowButtonDownFcn',  @onDown);
set(fig,'WindowButtonMotionFcn',@onMotion);
set(fig,'WindowButtonUpFcn',    @onUp);

redraw(fig);
end


% ====================================================================
%  Basemap and colormap helpers
% ====================================================================
function drawBasemap(ax, bgRGB, mapXlim, mapYlim)
%   bgRGB has rows indexed by metre-North (small index = -North edge,
%   large index = +North edge) and columns by metre-East. To put North
%   pointing UP on the screen we set YDir to 'normal' and rely on
%   image() interpreting bgRGB(row, col, :) at (col, row) coordinates.
image(ax, mapXlim, mapYlim, bgRGB);
set(ax,'YDir','normal');
axis(ax,'equal');  hold(ax,'on');  grid(ax,'on');
xlim(ax, mapXlim);  ylim(ax, mapYlim);
end

function cmap = red_ramp(N)
%RED_RAMP  Light-red -> dark-red monotone colormap.
%   Goes from a pale pink (low density) to a deep brick red (high
%   density). Both endpoints have R > G,B so the ramp stays in the red
%   hue family throughout -- no excursion into yellow or white.
if nargin < 1; N = 256; end
% Start: light red / pink. End: dark blood red.
r0 = [0.98 0.85 0.85];     % near-white pink
r1 = [0.45 0.00 0.00];     % deep dark red
t  = linspace(0,1,N).';
cmap = (1-t).*r0 + t.*r1;
end


% ====================================================================
%  Mouse + UI handlers
% ====================================================================
function onDown(fig, ~)
data = guidata(fig);
cp = get(data.axL,'CurrentPoint');
xE = cp(1,1);   yN = cp(1,2);   % (East, North) in metres

if xE < data.mapXlim(1) || xE > data.mapXlim(2) || ...
   yN < data.mapYlim(1) || yN > data.mapYlim(2)
    data.dragging = 0;  guidata(fig,data);  return;
end

sel = get(fig,'SelectionType');
HIT = 40;

if ~isempty(data.ueTrue)
    d = sqrt((data.ueTrue(:,1)-xE).^2 + (data.ueTrue(:,2)-yN).^2);
    [dmin, idx] = min(d);
else
    dmin = inf;  idx = 0;
end

switch sel
    case 'normal'
        if dmin < HIT
            data.dragging = idx;
        else
            data.ueTrue(end+1,:) = [xE, yN];
            data.dragging = size(data.ueTrue,1);
        end
    case 'alt'
        if dmin < HIT;  data.ueTrue(idx,:) = [];  end
        data.dragging = 0;
    otherwise
        data.dragging = 0;
end

guidata(fig,data);  redraw(fig);
end

function onMotion(fig, ~)
data = guidata(fig);
if data.dragging == 0;  return;  end
cp = get(data.axL,'CurrentPoint');
xE = max(data.mapXlim(1), min(cp(1,1), data.mapXlim(2)));
yN = max(data.mapYlim(1), min(cp(1,2), data.mapYlim(2)));
data.ueTrue(data.dragging,:) = [xE, yN];
guidata(fig,data);  redraw(fig);
end

function onUp(fig, ~)
data = guidata(fig);
data.dragging = 0;  guidata(fig,data);
end

function onClear(fig)
data = guidata(fig);
data.ueTrue = zeros(0,2);
guidata(fig,data);  redraw(fig);
end

function onAddRandom(fig, N)
data = guidata(fig);
xs = data.mapXlim(1) + diff(data.mapXlim)*rand(N,1);   % East
ys = data.mapYlim(1) + diff(data.mapYlim)*rand(N,1);   % North
data.ueTrue = [data.ueTrue; xs, ys];
guidata(fig,data);  redraw(fig);
end

function onResample(fig);  redraw(fig);  end

function onSlider(fig, fieldName, val)
data = guidata(fig);
switch fieldName
    case 'sigma_xi_dB';   data.params.sigma_xi_dB    = val;
    case 'sigma_phi_deg'; data.params.sigma_phi_rad  = deg2rad(val);
    case 'alpha';         data.alpha                 = val;
end
guidata(fig,data);  redraw(fig);
end


% ====================================================================
%  Redraw
% ====================================================================
function redraw(fig)
data = guidata(fig);

ueRaw  = simulate_measurements(data.ueTrue, data.gnb, data.params);
rhoMap = local_gnb_build_density_map(data.gnb, ueRaw, ...
            data.XG, data.YG, data.params);

set(data.hUE,  'XData', data.ueTrue(:,1), 'YData', data.ueTrue(:,2));
set(data.hImg, 'CData', rhoMap, 'AlphaData', data.alpha * rhoMap);

K     = size(data.ueTrue,1);
integ = sum(rhoMap(:)) * data.params.rho_op * data.cellSize^2;
set(data.hStatus,'String', sprintf( ...
    '%d UEs   |   integrated mass: %.2f   (target: %d)', ...
    K, integ, K));
drawnow limitrate;
end


function ueRaw = simulate_measurements(ueTrue, gnb, params)
%   ueTrue: [E N] columns. gnb.pos = [E N D]. AoA is measured CCW from
%   the +East axis (standard math convention), so it lives in the same
%   frame as the (East,North) coordinates the rest of the code uses.
K = size(ueTrue,1);
ueRaw = struct('rsrp_dBm',{},'aoa_rad',{});
if K == 0;  return;  end
ueRaw(K).rsrp_dBm = NaN;

xG = gnb.pos(1);  yG = gnb.pos(2);
EIRP = params.Pt_dBm + params.Gt_dB + params.Gr_dB - params.PL0_dB;

for k = 1:K
    dE = ueTrue(k,1) - xG;
    dN = ueTrue(k,2) - yG;
    d  = max(sqrt(dE^2 + dN^2), params.d_min);
    phi_true = atan2(dN, dE);          % CCW from +East toward +North
    ueRaw(k).rsrp_dBm = EIRP - 10*params.n*log10(d) ...
                       + params.sigma_xi_dB * randn();
    ueRaw(k).aoa_rad  = phi_true + params.sigma_phi_rad * randn();
end
end


% ====================================================================
%  Embedded estimator (matches gnb_build_density_map.m).
%  Now interprets gnb.pos and grid as (East, North) -- the math is
%  identical, only the variable names change to match the cartographic
%  convention used by the rest of this demo.
% ====================================================================
function rhoMap = local_gnb_build_density_map(gnb, ueRaw, XG, YG, params)
[Ny, Nx] = size(XG);
xVec = XG(1,:);  yVec = YG(:,1);
rho  = zeros(Ny, Nx);

xG = gnb.pos(1);   yG = gnb.pos(2);
EIRP_dB       = params.Pt_dBm + params.Gt_dB + params.Gr_dB - params.PL0_dB;
ln10_over_10n = log(10) / (10*params.n);
truncSig      = 4;

for k = 1:numel(ueRaw)
    ue = ueRaw(k);
    d_hat = 10.^( (EIRP_dB - ue.rsrp_dBm) / (10*params.n) );
    d_hat = max(params.d_min, min(d_hat, params.d_max));

    if isfield(ue,'aoa_rad') && ~isnan(ue.aoa_rad)
        phi_hat   = ue.aoa_rad;
        sigma_phi = params.sigma_phi_rad;
    else
        phi_hat   = 2*pi*rand();
        sigma_phi = pi;
    end

    x_k = xG + d_hat * cos(phi_hat);
    y_k = yG + d_hat * sin(phi_hat);

    sigma_r = min(ln10_over_10n * d_hat * params.sigma_xi_dB, params.cap_sigma_m);
    sigma_p = min(d_hat * sigma_phi,                          params.cap_sigma_m);

    c = cos(phi_hat);  s = sin(phi_hat);
    R     = [ c -s ;  s  c ];
    Sigma = R * diag([sigma_r^2, sigma_p^2]) * R.';

    [~, Deig] = eig(Sigma);
    smax = sqrt(max(diag(Deig)));   Rtrunc = truncSig * smax;

    ix = find(xVec >= x_k - Rtrunc & xVec <= x_k + Rtrunc);
    iy = find(yVec >= y_k - Rtrunc & yVec <= y_k + Rtrunc);
    if isempty(ix) || isempty(iy);  continue;  end

    Xs = XG(iy,ix);   Ys = YG(iy,ix);
    invS = inv(Sigma);   detS = det(Sigma);
    dxg  = Xs - x_k;     dyg  = Ys - y_k;
    q    = invS(1,1)*dxg.^2 + 2*invS(1,2)*dxg.*dyg + invS(2,2)*dyg.^2;
    g    = exp(-0.5*q) / (2*pi*sqrt(detS));
    rho(iy,ix) = rho(iy,ix) + g;
end

rhoMap = min(rho / params.rho_op, 1);
end