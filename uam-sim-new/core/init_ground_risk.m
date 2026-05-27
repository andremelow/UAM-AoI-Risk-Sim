function [rhoMap, mapGrid] = init_ground_risk(cfg)
% INIT_GROUND_RISK  Builds spatial risk density field rho(x).
%
%  Extracted from monolith section 4 (SPATIAL RISK HEATMAP).

half     = cfg.corridorLength / 2;
margin   = max(200, cfg.corridorLength * 0.15);   % 15% margin
% Expand limits to include all BS positions
% builder exports microBSPos as [East, North, Alt] → North=col2, East=col1
bsNorth  = cfg.microBSPos(:,2)';   % North coords of BSs
bsEast   = cfg.microBSPos(:,1)';   % East  coords of BSs
xMin     = min([-(half + margin),  bsNorth - margin]);
xMax     = max([ (half + margin),  bsNorth + margin]);
yMin     = min([-margin,           bsEast  - margin]);
yMax     = max([ margin,           bsEast  + margin]);
mapXlim  = [xMin, xMax];
mapYlim  = [yMin, yMax];
cellSize = max(20, round(cfg.corridorLength / 60));
hmapZ    = -10;

xVec = (mapXlim(1)+cellSize/2) : cellSize : (mapXlim(2)-cellSize/2);
yVec = (mapYlim(1)+cellSize/2) : cellSize : (mapYlim(2)-cellSize/2);
[XG, YG] = meshgrid(xVec, yVec);
deltaA   = cellSize^2;

% --- Seeded random hotspots ---
hotX = []; hotY = []; hotAmp = []; hotSig = [];
rng(cfg.rng_seed);
for h = 1:cfg.numHotspots
    hotX(end+1)   = mapXlim(1) + rand() * diff(mapXlim); %#ok<AGROW>
    hotY(end+1)   = mapYlim(1) + rand() * diff(mapYlim); %#ok<AGROW>
    hotAmp(end+1) = 0.5 + 0.5 * rand();                  %#ok<AGROW>
    hotSig(end+1) = 80  + 120  * rand();                  %#ok<AGROW>
end

% --- Append manual hotspots from config ---
if ~isempty(cfg.manualHotspots)
    for m = 1:size(cfg.manualHotspots, 1)
        hotX(end+1)   = cfg.manualHotspots(m,1);   %#ok<AGROW>
        hotY(end+1)   = cfg.manualHotspots(m,2);   %#ok<AGROW>
        hotAmp(end+1) = cfg.manualHotspots(m,3);   %#ok<AGROW>
        hotSig(end+1) = cfg.manualHotspots(m,4);   %#ok<AGROW>
    end
end

% --- Build rhoMap ---
numHotspots = numel(hotX);
rhoMap = zeros(size(XG));
for h = 1:numHotspots
    rhoMap = rhoMap + hotAmp(h) * ...
        exp(-((XG-hotX(h)).^2 + (YG-hotY(h)).^2) / (2*hotSig(h)^2));
end
rhoMap = min(rhoMap, 1);

% --- Grid metadata for downstream use ---
mapGrid.XG      = XG;
mapGrid.YG      = YG;
mapGrid.xFlat   = XG(:);
mapGrid.yFlat   = YG(:);
mapGrid.rhoFlat = rhoMap(:);
mapGrid.deltaA  = deltaA;
mapGrid.hmapZ   = hmapZ;
mapGrid.mapXlim = mapXlim;
mapGrid.mapYlim = mapYlim;
end
