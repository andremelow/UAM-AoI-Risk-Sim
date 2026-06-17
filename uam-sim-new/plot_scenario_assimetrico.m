%% PLOT_SCENARIO_ASSIMETRICO
%  Visualiza o mapa de risco, rotas e posição da gNB do cenário assimétrico.
%  Salva em docs/scenario_assimetrico.png
%
%  Usage (MATLAB Command Window):
%    plot_scenario_assimetrico

setup_paths();
cfg = build_scenario_assimetrico();

%% ── Mapa de risco ────────────────────────────────────────────────────
[rhoMap, mapGrid] = init_ground_risk(cfg);
XG = mapGrid.XG;   % North
YG = mapGrid.YG;   % East

%% ── SNR — contornos aproximados (UMa LOS, drone@60m, BS@30m) ────────
% SNR = 132 - [28 + 22*log10(d) + 20*log10(3.5)]
% d(SNR) = 10^((132 - SNR - 28 - 10.88) / 22)
snr_levels = [5, 10, 20];   % dB
d_snr = 10 .^ ((132 - snr_levels - 28 - 20*log10(3.5)) / 22);   % m

bs_n = cfg.microBSPos(1, 2);   % North
bs_e = cfg.microBSPos(1, 1);   % East

theta = linspace(0, 2*pi, 360);

%% ── Figura ───────────────────────────────────────────────────────────
fig = figure('Position', [100 100 1200 600], 'Color', 'k');

%% ── Painel esquerdo: mapa de risco + cenário ─────────────────────────
ax = axes('Position', [0.05 0.10 0.60 0.82], 'Color', 'k', ...
          'XColor', [0.7 0.7 0.7], 'YColor', [0.7 0.7 0.7], ...
          'GridColor', [0.3 0.3 0.3], 'GridAlpha', 0.5);
hold(ax, 'on'); grid(ax, 'on');

% Heatmap de risco (eixo x=East, y=North)
imagesc(ax, mapGrid.mapYlim, mapGrid.mapXlim, rhoMap);
colormap(ax, hot(256));
cb = colorbar(ax);
cb.Color = [0.7 0.7 0.7];
cb.Label.String = 'Risco relativo ρ(x)';
cb.Label.Color  = [0.7 0.7 0.7];
clim(ax, [0 1]);
set(ax, 'YDir', 'normal');

% Contornos SNR
snr_colors = {'#00FF41', '#FFD700', '#FF5252'};   % verde/ouro/vermelho
snr_labels = {'SNR = 20 dB', 'SNR = 10 dB', 'SNR = 5 dB'};
for i = 1:numel(snr_levels)
    c_e = bs_e + d_snr(i) * cos(theta);
    c_n = bs_n + d_snr(i) * sin(theta);
    plot(ax, c_e, c_n, '--', 'Color', snr_colors{i}, ...
         'LineWidth', 1.2, 'DisplayName', snr_labels{i});
end

% Rotas
for ri = 1:numel(cfg.routes)
    rt   = cfg.routes(ri);
    cols = {'#99CCFF', '#FF9999'};
    plot(ax, [rt.start(2) rt.goal(2)], [rt.start(1) rt.goal(1)], ...
         '-', 'Color', cols{ri}, 'LineWidth', 2.5, ...
         'DisplayName', rt.label);
    % marcadores início / fim
    plot(ax, rt.start(2), rt.start(1), 'o', 'Color', cols{ri}, ...
         'MarkerFaceColor', cols{ri}, 'MarkerSize', 7, 'HandleVisibility', 'off');
    plot(ax, rt.goal(2),  rt.goal(1),  's', 'Color', cols{ri}, ...
         'MarkerFaceColor', cols{ri}, 'MarkerSize', 7, 'HandleVisibility', 'off');
end

% gNB
plot(ax, bs_e, bs_n, '^', 'Color', '#00FFFF', 'MarkerFaceColor', '#00FFFF', ...
     'MarkerSize', 12, 'LineWidth', 1.5, 'DisplayName', sprintf('gNB (E=%d m)', bs_e));

% Hotspots manuais
if ~isempty(cfg.manualHotspots)
    plot(ax, cfg.manualHotspots(:,2), cfg.manualHotspots(:,1), ...
         'x', 'Color', '#FF6600', 'MarkerSize', 8, 'LineWidth', 1.5, ...
         'DisplayName', 'Hotspots (manual)');
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

%% ── Painel direito: perfil SNR ao longo de Route 3 ──────────────────
ax2 = axes('Position', [0.72 0.10 0.26 0.82], 'Color', 'k', ...
           'XColor', [0.7 0.7 0.7], 'YColor', [0.7 0.7 0.7], ...
           'GridColor', [0.3 0.3 0.3], 'GridAlpha', 0.5);
hold(ax2, 'on'); grid(ax2, 'on');

% Route 3 vai de E=15000 → E=0
e_r3 = linspace(15000, 0, 300);
n_r3 = linspace(-500, -600, 300);   % interpolação linear
d_r3 = sqrt((e_r3 - bs_e).^2 + (n_r3 - bs_n).^2 + 30^2);   % 3D (delta_alt=30m)
PL_r3 = 28 + 22*log10(d_r3) + 20*log10(3.5);
SNR_r3 = 132 - PL_r3;   % link budget: Ptx=23, G=15, noise=-94 dBm

plot(ax2, SNR_r3, e_r3/1000, '-', 'Color', '#FF9999', 'LineWidth', 2, ...
     'DisplayName', 'Route 3');

% Route 2: E=0 → E=15000
e_r2 = linspace(0, 15000, 300);
n_r2 = linspace(200, 500, 300);
d_r2 = sqrt((e_r2 - bs_e).^2 + (n_r2 - bs_n).^2 + 30^2);
PL_r2 = 28 + 22*log10(d_r2) + 20*log10(3.5);
SNR_r2 = 132 - PL_r2;

plot(ax2, SNR_r2, e_r2/1000, '-', 'Color', '#99CCFF', 'LineWidth', 2, ...
     'DisplayName', 'Route 2');

% Linha de threshold
xline(ax2, 5,  '--', 'Color', '#FF5252', 'LineWidth', 1.2, 'Label', 'SNR=5 dB', ...
      'LabelColor', '#FF5252', 'FontSize', 8);
xline(ax2, 10, '--', 'Color', '#FFD700', 'LineWidth', 1.2, 'Label', 'SNR=10 dB', ...
      'LabelColor', '#FFD700', 'FontSize', 8);

xlabel(ax2, 'SNR (dB)', 'Color', [0.7 0.7 0.7]);
ylabel(ax2, 'East (km)', 'Color', [0.7 0.7 0.7]);
title(ax2, 'Perfil SNR ao longo das rotas', 'Color', 'w', 'FontSize', 11);
legend(ax2, 'Location', 'northeast', 'TextColor', [0.7 0.7 0.7], ...
       'Color', [0.08 0.08 0.08], 'EdgeColor', [0.3 0.3 0.3]);

%% ── Salva ────────────────────────────────────────────────────────────
outDir = fullfile(fileparts(mfilename('fullpath')), '..', 'docs');
if ~exist(outDir, 'dir'), mkdir(outDir); end
outFile = fullfile(outDir, 'scenario_assimetrico.png');
exportgraphics(fig, outFile, 'Resolution', 150, 'BackgroundColor', 'k');
fprintf('Salvo em %s\n', outFile);
