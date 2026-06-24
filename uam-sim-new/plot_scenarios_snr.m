%% PLOT_SCENARIOS_SNR
%  Mapa de SNR + rotas + posicao da gNB para os 4 cenarios de geometria
%  distinta rodados recentemente.  Reproduz fielmente o modelo de canal do
%  engine (core/step_channel.m): nrPathLoss UMa-LOS, drone em cruiseAlt AGL.
%
%    1) 2r_static    RouteA + RouteB     (curta, longe da BS + hotspots)
%    2) 2r_variavel  RouteA + RouteBlong (gradiente de SNR ao longo da rota)
%    3) 2r_a23       RouteA + RouteBhalf (metade final de RouteBlong)
%    4) assimetrico  Route2 <-> Route3   (cruzamento de 15 km)
%
%  Todos compartilham a mesma gNB (E=5000, N=0, alt=30) e os mesmos
%  parametros de radio (build_scenario_variavel).
%
%  Saida: docs/scenarios_snr.png
%
%  Usage (MATLAB Command Window):
%    plot_scenarios_snr

setup_paths();

%% ── Base config (radio + gNB) e perfil de altitude do drone ───────────
cfg = validate_uam_config(build_scenario_variavel());
plCfg          = nrPathLossConfig;
plCfg.Scenario = "UMa";
droneAlt       = cfg.cruiseAlt;                 % m AGL (padrao 60)

bsNED = [cfg.microBSPos(:,2), cfg.microBSPos(:,1), -cfg.microBSPos(:,3)];  % [N E Down]
bs_e  = cfg.microBSPos(1,1);
bs_n  = cfg.microBSPos(1,2);

%% ── Funcao de SNR (identica ao step_channel) ──────────────────────────
%  Aceita vetores de North/East (mesmo tamanho).  BS unica.
snr_fun = @(N, E) snr_uma(N, E, droneAlt, bsNED, plCfg, cfg);

%% ── Definicao dos 4 cenarios (so as rotas mudam) ──────────────────────
rA      = struct('start',[1000  1000],'goal',[1000  9000],'label','RouteA');
rB      = struct('start',[-500 12000],'goal',[-600 14000],'label','RouteB');
rBlong  = struct('start',[0     5500],'goal',[-700 14000],'label','RouteBlong');
rBhalf  = struct('start',[-350  9750],'goal',[-700 14000],'label','RouteBhalf');
r2      = struct('start',[200      0],'goal',[500  15000],'label','Route2');
r3      = struct('start',[-500 15000],'goal',[-600     0],'label','Route3');

scen(1) = struct('name','2r\_static',   'routes',{[rA rB]});
scen(2) = struct('name','2r\_variavel', 'routes',{[rA rBlong]});
scen(3) = struct('name','2r\_a23',      'routes',{[rA rBhalf]});
scen(4) = struct('name','assimetrico',  'routes',{[r2 r3]});

%% ── Grid comum (cobre todas as rotas + BS) ────────────────────────────
allPts = [];
for s = 1:numel(scen)
    for r = scen(s).routes
        allPts = [allPts; r.start; r.goal]; %#ok<AGROW>
    end
end
allPts = [allPts; bs_n bs_e];
margin = 800;
nLim = [min(allPts(:,1))-margin, max(allPts(:,1))+margin];   % North
eLim = [min(allPts(:,2))-margin, max(allPts(:,2))+margin];   % East
res  = 80;                                                    % m por celula
eVec = eLim(1):res:eLim(2);
nVec = nLim(1):res:nLim(2);
[EG, NG] = meshgrid(eVec, nVec);
SNRMAP = snr_fun(NG(:), EG(:));
SNRMAP = reshape(SNRMAP, size(NG));

SNR_LO = 0; SNR_HI = 40;                       % escala de cor [dB]
thr    = cfg.thresholdSNR;

%% ── Figura: 2x2 ───────────────────────────────────────────────────────
fig = figure('Position',[60 60 1300 1150],'Color','k');
tl  = tiledlayout(fig, 4, 1, 'Padding','compact','TileSpacing','compact');

cmapSNR = snr_colormap(256);   % RdYlGn-like (vermelho=baixo, verde=alto)

for s = 1:numel(scen)
    ax = nexttile(tl);
    set(ax,'Color','k','XColor',[.7 .7 .7],'YColor',[.7 .7 .7], ...
           'GridColor',[.3 .3 .3],'GridAlpha',.4);
    hold(ax,'on');

    % Heatmap de SNR
    imagesc(ax, eLim, nLim, SNRMAP);
    set(ax,'YDir','normal');
    colormap(ax, cmapSNR);
    clim(ax, [SNR_LO SNR_HI]);

    % Contorno do threshold de SNR
    [~,hc] = contour(ax, EG, NG, SNRMAP, [thr thr], 'w--', 'LineWidth', 1.0);
    set(hc,'DisplayName',sprintf('SNR = %g dB', thr));

    % Rotas coloridas pelo SNR ao longo do trajeto
    nSeg = 300;
    for r = scen(s).routes
        ns = linspace(r.start(1), r.goal(1), nSeg);
        es = linspace(r.start(2), r.goal(2), nSeg);
        sv = snr_fun(ns(:), es(:));
        % contorno preto por baixo para destacar a rota sobre o heatmap
        plot(ax, es, ns, '-', 'Color','k', 'LineWidth', 5.0);
        % segmentos coloridos pelo SNR local
        for k = 1:nSeg-1
            c = snr_color(sv(k), SNR_LO, SNR_HI, cmapSNR);
            plot(ax, es(k:k+1), ns(k:k+1), '-', 'Color', c, 'LineWidth', 3.0);
        end
        plot(ax, r.start(2), r.start(1), 'o', 'MarkerFaceColor','w', ...
             'MarkerEdgeColor','k', 'MarkerSize',7);
        plot(ax, r.goal(2),  r.goal(1),  's', 'MarkerFaceColor','w', ...
             'MarkerEdgeColor','k', 'MarkerSize',8);
        text(ax, r.goal(2), r.goal(1)+250, r.label, 'Color','w', ...
             'FontSize',9, 'FontWeight','bold', 'HorizontalAlignment','center');
    end

    % gNB
    plot(ax, bs_e, bs_n, '^', 'MarkerFaceColor','c', 'MarkerEdgeColor','k', ...
         'MarkerSize',13, 'LineWidth',1);
    text(ax, bs_e, bs_n-450, 'gNB', 'Color','c', 'FontSize',9, ...
         'FontWeight','bold', 'HorizontalAlignment','center');

    axis(ax,'equal');
    xlim(ax, eLim); ylim(ax, nLim);
    xlabel(ax,'East (m)','Color',[.7 .7 .7]);
    ylabel(ax,'North (m)','Color',[.7 .7 .7]);
    title(ax, scen(s).name, 'Color','w', 'FontSize',12);
end

% Colorbar comum
cb = colorbar(nexttile(tl,1));
cb.Layout.Tile  = 'east';
cb.Color        = [.7 .7 .7];
cb.Label.String = sprintf('SNR (dB)  —  UMa LOS, drone @ %g m AGL', droneAlt);
cb.Label.Color  = [.7 .7 .7];

title(tl, sprintf('Cenarios recentes — rotas, gNB e SNR  (fc=%.1f GHz, BW=%g MHz, thr=%g dB)', ...
      cfg.fc/1e9, cfg.bw/1e6, thr), 'Color','w', 'FontSize',13);

%% ── Salva ─────────────────────────────────────────────────────────────
outDir = fullfile(fileparts(mfilename('fullpath')), '..', 'docs');
if ~exist(outDir,'dir'), mkdir(outDir); end
outFile = fullfile(outDir, 'scenarios_snr.png');
exportgraphics(fig, outFile, 'Resolution', 150, 'BackgroundColor','k');
fprintf('Salvo em %s\n', outFile);

%% ── Helpers ───────────────────────────────────────────────────────────
function snr = snr_uma(Nv, Ev, alt, bsNED, plCfg, cfg)
    % Vetorizado: replica core/step_channel.m para uma BS unica.
    M    = numel(Nv);
    % step_channel: droneNED = [N; E; -pos(3)] com pos(3)=Down=-alt  ->  +alt.
    % (sinal importa: nrPathLoss UMa usa a altura da UE no breakpoint)
    posUE = [Nv(:)'; Ev(:)'; alt*ones(1,M)];     % NED 3xM
    posBS = bsNED(1,:)';                          % NED 3x1 (BS unica)
    % nrPathLoss(posBS 3x1, posUE 3xM) -> PL 1xM (elemento-a-elemento)
    PL   = nrPathLoss(plCfg, cfg.fc, true, posBS, posUE);   % LOS
    snr  = (cfg.pTransmitDrone + cfg.gNB_Gain - PL(:)) ...
           - (cfg.thermalNoise + cfg.noiseFigure);
    snr  = reshape(snr, size(Nv));
end

function c = snr_color(val, lo, hi, cmap)
    t   = max(0, min(1, (val - lo)/(hi - lo)));
    idx = max(1, min(size(cmap,1), round(1 + t*(size(cmap,1)-1))));
    c   = cmap(idx,:);
end

function cmap = snr_colormap(n)
    % Aproximacao de RdYlGn (vermelho=baixo SNR -> verde=alto SNR).
    anchors = [0.647 0.000 0.149;   % vermelho escuro
               0.843 0.188 0.153;
               0.957 0.427 0.263;
               0.992 0.682 0.380;
               0.996 0.878 0.545;
               1.000 1.000 0.749;   % amarelo claro
               0.851 0.937 0.545;
               0.651 0.851 0.416;
               0.400 0.741 0.388;
               0.102 0.596 0.314;
               0.000 0.408 0.216];  % verde escuro
    x  = linspace(0,1,size(anchors,1));
    xi = linspace(0,1,n);
    cmap = [interp1(x,anchors(:,1),xi)', ...
            interp1(x,anchors(:,2),xi)', ...
            interp1(x,anchors(:,3),xi)'];
end
