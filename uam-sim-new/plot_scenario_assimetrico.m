%% PLOT_SCENARIO_ASSIMETRICO
%  Visualiza o mapa de risco, rotas e posição da gNB do cenário assimétrico.
%  Painel esquerdo : mapa de risco 2D + rotas + gNB
%  Painel direito  : perfil de risco ρ(x) ao longo de cada rota
%  Salva em docs/scenario_assimetrico.png
%
%  Usage (MATLAB Command Window):
%    plot_scenario_assimetrico

setup_paths();
cfg = build_scenario_assimetrico();

%% ── Mapa de risco ────────────────────────────────────────────────────
[rhoMap, mapGrid] = init_ground_risk(cfg);
XG = mapGrid.XG;   % North (linhas)
YG = mapGrid.YG;   % East  (colunas)

bs_n = cfg.microBSPos(1, 2);   % North
bs_e = cfg.microBSPos(1, 1);   % East

%% ── Perfil de risco ao longo de cada rota ────────────────────────────
nSamp = 500;
rho_routes = cell(1, numel(cfg.routes));
dist_routes = cell(1, numel(cfg.routes));
for ri = 1:numel(cfg.routes)
    rt   = cfg.routes(ri);
    ns   = linspace(rt.start(1), rt.goal(1), nSamp);   % North
    es   = linspace(rt.start(2), rt.goal(2), nSamp);   % East
    rho  = zeros(1, nSamp);
    for k = 1:nSamp
        % bilinear interpolation na grid
        % XG(1,:) = North (col dim), YG(:,1) = East (row dim)
        rho(k) = interp2(XG(1,:), YG(:,1), rhoMap, ns(k), es(k), 'linear', 0);
    end
    % distância acumulada ao longo da rota [km]
    dx   = diff(ns); dy = diff(es);
    d    = [0, cumsum(sqrt(dx.^2 + dy.^2))] / 1000;
    rho_routes{ri}  = rho;
    dist_routes{ri} = d;
end

%% ── Figura ───────────────────────────────────────────────────────────
fig = figure('Position', [100 100 1300 580], 'Color', 'k');

%% ── Painel esquerdo: mapa + cenário ─────────────────────────────────
ax = axes('Position', [0.04 0.10 0.62 0.82], 'Color', 'k', ...
          'XColor', [0.7 0.7 0.7], 'YColor', [0.7 0.7 0.7], ...
          'GridColor', [0.3 0.3 0.3], 'GridAlpha', 0.4);
hold(ax, 'on'); grid(ax, 'on');

% Heatmap de risco
imagesc(ax, mapGrid.mapYlim, mapGrid.mapXlim, rhoMap);
colormap(ax, hot(256));
cb = colorbar(ax);
cb.Color = [0.7 0.7 0.7];
cb.Label.String = 'Risco relativo ρ(x)';
cb.Label.Color  = [0.7 0.7 0.7];
clim(ax, [0 1]);
set(ax, 'YDir', 'normal');

% Rotas
rt_colors = {'#99CCFF', '#FF9999'};
for ri = 1:numel(cfg.routes)
    rt = cfg.routes(ri);
    plot(ax, [rt.start(2) rt.goal(2)], [rt.start(1) rt.goal(1)], ...
         '-', 'Color', rt_colors{ri}, 'LineWidth', 2.5, ...
         'DisplayName', rt.label);
    plot(ax, rt.start(2), rt.start(1), 'o', 'Color', rt_colors{ri}, ...
         'MarkerFaceColor', rt_colors{ri}, 'MarkerSize', 8, 'HandleVisibility', 'off');
    plot(ax, rt.goal(2),  rt.goal(1),  's', 'Color', rt_colors{ri}, ...
         'MarkerFaceColor', rt_colors{ri}, 'MarkerSize', 8, 'HandleVisibility', 'off');
end

% gNB
plot(ax, bs_e, bs_n, '^', 'Color', '#00FFFF', 'MarkerFaceColor', '#00FFFF', ...
     'MarkerSize', 13, 'LineWidth', 1.5, ...
     'DisplayName', sprintf('gNB (E=%d m)', bs_e));

% Hotspots manuais
if ~isempty(cfg.manualHotspots)
    plot(ax, cfg.manualHotspots(:,2), cfg.manualHotspots(:,1), ...
         'x', 'Color', '#FF6600', 'MarkerSize', 9, 'LineWidth', 2, ...
         'DisplayName', 'Hotspots');
end

xlabel(ax, 'East (m)', 'Color', [0.7 0.7 0.7]);
ylabel(ax, 'North (m)', 'Color', [0.7 0.7 0.7]);
title(ax, 'Cenário assimétrico — 15 km, UMa, BS em 1/3', ...
      'Color', 'w', 'FontSize', 12);
lg = legend(ax, 'Location', 'northwest', 'TextColor', [0.7 0.7 0.7], ...
            'Color', [0.08 0.08 0.08], 'EdgeColor', [0.3 0.3 0.3]);
lg.FontSize = 9;
axis(ax, 'equal');
xlim(ax, mapGrid.mapYlim);
ylim(ax, mapGrid.mapXlim);

%% ── Painel direito: perfil de risco ao longo das rotas ───────────────
ax2 = axes('Position', [0.73 0.10 0.25 0.82], 'Color', 'k', ...
           'XColor', [0.7 0.7 0.7], 'YColor', [0.7 0.7 0.7], ...
           'GridColor', [0.3 0.3 0.3], 'GridAlpha', 0.4);
hold(ax2, 'on'); grid(ax2, 'on');

for ri = 1:numel(cfg.routes)
    plot(ax2, rho_routes{ri}, dist_routes{ri}, '-', ...
         'Color', rt_colors{ri}, 'LineWidth', 2, ...
         'DisplayName', cfg.routes(ri).label);
end

xlabel(ax2, 'Densidade de risco ρ(x)', 'Color', [0.7 0.7 0.7]);
ylabel(ax2, 'Distância percorrida (km)', 'Color', [0.7 0.7 0.7]);
title(ax2, 'Perfil de risco ao longo da rota', 'Color', 'w', 'FontSize', 11);
legend(ax2, 'Location', 'northeast', 'TextColor', [0.7 0.7 0.7], ...
       'Color', [0.08 0.08 0.08], 'EdgeColor', [0.3 0.3 0.3]);
ylim(ax2, [0, max(cellfun(@max, dist_routes)) * 1.05]);

%% ── Salva ────────────────────────────────────────────────────────────
outDir = fullfile(fileparts(mfilename('fullpath')), '..', 'docs');
if ~exist(outDir, 'dir'), mkdir(outDir); end
outFile = fullfile(outDir, 'scenario_assimetrico.png');
exportgraphics(fig, outFile, 'Resolution', 150, 'BackgroundColor', 'k');
fprintf('Salvo em %s\n', outFile);
