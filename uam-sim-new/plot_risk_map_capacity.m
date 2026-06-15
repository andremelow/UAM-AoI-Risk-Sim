function plot_risk_map_capacity(outFile)
% PLOT_RISK_MAP_CAPACITY  Mapa de risco e posição da gNB para
%   o experimento experiment_capacity_15min_sw_nsw (2 lanes, L_vid=100).
%
%   outFile (opcional): caminho do PNG de saída.
%                       Default: 'risk_map_capacity.png' no diretório do script.

setup_paths();

if nargin < 1 || isempty(outFile)
    outFile = fullfile(fileparts(mfilename('fullpath')), 'risk_map_capacity.png');
end

% ── Config idêntico ao experimento ────────────────────────────────────────
cfg = base_config_capacity();

% Usa o cenário com 2 lanes (2r) para o mapa ter a geometria real do sweep
cfg.routes(1).numDrones = 1;
cfg.routes(1).start     = [ 50, -1000];
cfg.routes(1).goal      = [ 50,  1000];
cfg.routes(1).label     = 'lane-N';
cfg.routes(2).numDrones = 1;
cfg.routes(2).start     = [-50, -1000];
cfg.routes(2).goal      = [-50,  1000];
cfg.routes(2).label     = 'lane-S';

cfg = validate_uam_config(cfg);

% ── Mapa de densidade e risco ─────────────────────────────────────────────
[rhoMap, mapGrid] = init_ground_risk(cfg);
[K_norm, ~]       = compute_risk_map(rhoMap, mapGrid, cfg.routing);

% Eixos: XG = Norte, YG = Leste  (convenção de init_ground_risk)
%   → plotar com Leste no eixo-X e Norte no eixo-Y
xVec = mapGrid.XG(1,:);   % Norte
yVec = mapGrid.YG(:,1);   % Leste
% K_norm(row=Leste, col=Norte); transpomos para imagesc(x=Leste, y=Norte)
K_plot = K_norm';          % agora K_plot(row=Norte, col=Leste)

% ── Figura ────────────────────────────────────────────────────────────────
fig = figure('Visible','off','Color','k', ...
             'Position',[0 0 1200 480]);

ax = axes('Parent',fig,'Color','k');
hold(ax,'on');

% Mapa de risco (heatmap)
imagesc(ax, yVec, xVec, K_plot);
set(ax,'YDir','normal');

% Colormap: preto → amarelo → vermelho (risco crescente)
colormap(ax, hot(256));
cb = colorbar(ax);
cb.Color        = [0.85 0.85 0.85];
cb.Label.String = 'Risco de solo normalizado  K_c / K_{max}';
cb.Label.Color  = [0.9 0.9 0.9];
cb.Label.FontSize = 10;
caxis(ax,[0 1]);

% Alpha proporcional ao risco (células vazias ficam transparentes)
alpha_data = 0.85 * K_plot.^0.5;
set(ax.Children(end), 'AlphaData', alpha_data);

% Fundo preto para o painel
set(ax,'Color','k','XColor',[0.7 0.7 0.7],'YColor',[0.7 0.7 0.7], ...
       'GridColor',[0.3 0.3 0.3],'GridAlpha',0.4);
grid(ax,'on');

% Linhas do corredor: Lane N (Norte=+50 m) e Lane S (Norte=-50 m)
laneEast = [min(yVec), max(yVec)];
plot(ax, laneEast, [ 50  50], '--','Color',[0.4 0.8 1.0],'LineWidth',1.6, ...
     'DisplayName','Lane N (+50 m)');
plot(ax, laneEast, [-50 -50], '--','Color',[0.9 0.6 0.2],'LineWidth',1.6, ...
     'DisplayName','Lane S (−50 m)');

% Sentido de voo: setas no meio de cada lane
arrowE = [-300 0 300];
for ae = arrowE
    annotation_arrow(ax, ae, ae+200,  50,  50, [0.4 0.8 1.0]);
    annotation_arrow(ax, ae, ae+200, -50, -50, [0.9 0.6 0.2]);
end

% gNB: triângulo vermelho
gnbE = cfg.microBSPos(1,1);   % Leste
gnbN = cfg.microBSPos(1,2);   % Norte
plot(ax, gnbE, gnbN, '^', 'MarkerSize',14, ...
     'MarkerFaceColor',[1.0 0.2 0.2], 'MarkerEdgeColor','w', ...
     'LineWidth',1.2, 'DisplayName','gNB');
text(ax, gnbE+40, gnbN+20, 'gNB', 'Color','w', ...
     'FontSize',10,'FontWeight','bold');

% ── Labels e limites ──────────────────────────────────────────────────────
xlabel(ax,'Leste  (m)','Color',[0.85 0.85 0.85],'FontSize',11);
ylabel(ax,'Norte  (m)','Color',[0.85 0.85 0.85],'FontSize',11);
title(ax, 'Mapa de risco de solo — experimento capacity\_15min\_sw\_nsw', ...
      'Color','w','FontSize',13,'FontWeight','bold');

legend(ax,'Location','northeast','TextColor','w', ...
       'Color',[0.1 0.1 0.1],'EdgeColor',[0.4 0.4 0.4],'FontSize',9);

xlim(ax,[min(yVec) max(yVec)]);
ylim(ax,[min(xVec) max(xVec)]);
axis(ax,'equal');

% ── Salva ─────────────────────────────────────────────────────────────────
exportgraphics(fig, outFile, 'Resolution',180,'BackgroundColor','k');
fprintf('[plot_risk_map] Salvo: %s\n', outFile);
close(fig);
end


function annotation_arrow(ax, x1, x2, y1, y2, clr)
% Seta simples sobre os axes (usa quiver para compatibilidade)
dx = x2 - x1;   dy = y2 - y1;
quiver(ax, x1, y1, dx, dy, 0, 'Color', clr, ...
       'MaxHeadSize',0.8,'LineWidth',1.2,'HandleVisibility','off');
end
