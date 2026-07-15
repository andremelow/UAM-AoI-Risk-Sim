function plot_scenarios_map()
%% PLOT_SCENARIOS_MAP
%  Gera um mapa por cenário simulado contendo:
%    - nível de sinal (mapa de SNR à altitude de cruzeiro, modelo UMa LOS)
%    - posição da(s) gNB
%    - rotas, com início (○ verde) e fim (□ vermelho) destacados
%    - posição dos hotspots de risco
%
%  O SNR replica exatamente core/step_channel.m (nrPathLoss UMa, BS mais
%  próxima, posUE com Down = +cruiseAlt — convenção do engine).
%
%  Saída: uam-sim-new/scenario_plots/scenario_<label>.png

setup_paths();
OUT = fullfile(fileparts(mfilename('fullpath')), 'scenario_plots');
if ~exist(OUT, 'dir'), mkdir(OUT); end

% Base: gNB, hotspots e parâmetros de rádio compartilhados por todos os
% cenários (build_scenario_variavel é o cfg_base dos experimentos mw_otimo*).
base      = build_scenario_variavel();
cruiseAlt = 60;   % m AGL — default de modo fixo (validate_uam_config)

%% ── Cenários simulados (rotas; gNB+hotspots são comuns) ───────────────
rA  = mkroute([1000  1000], [1000   9000], 'RouteA');
rBl = mkroute([0     5500], [-700  14000], 'RouteBlong');
rB  = mkroute([-500 12000], [-600  14000], 'RouteB');
rBh = mkroute([-350  9750], [-700  14000], 'RouteBhalf');

S = { ...
    struct('label', 'variavel',  'routes', [rA rBl]), ...   % base mw_otimo
    struct('label', '2r_static', 'routes', [rA rB ]), ...   % a23
    struct('label', '2r_a23',    'routes', [rA rBh]) ...     % a23 (2/3 RouteA)
};

%% ── Mapa de SNR (à altitude de cruzeiro) ──────────────────────────────
Evec = linspace(0, 15000, 200);
Nvec = linspace(-1300, 1400, 90);
[EE, NN] = meshgrid(Evec, Nvec);
M = numel(EE);

plCfg = nrPathLossConfig; plCfg.Scenario = "UMa";
nBS    = size(base.microBSPos, 1);
SNRall = -inf(size(EE));
for b = 1:nBS
    bs    = base.microBSPos(b, :);                  % [East, North, Alt(neg=acima)]
    posBS = [bs(2); bs(1); -bs(3)];                 % NED [N; E; Down], 3x1 (= step_channel)
    posUE = [NN(:)'; EE(:)'; cruiseAlt*ones(1, M)]; % Down = +cruiseAlt (engine), 3xM
    % nrPathLoss(posBS 3x1, posUE 3xM) -> 1xM (uma BS x todos os pontos).
    PL    = nrPathLoss(plCfg, base.fc, true, posBS, posUE);
    PL    = PL(:)';
    SNR   = (base.pTransmitDrone + base.gNB_Gain - PL) ...
            - (base.thermalNoise + base.noiseFigure);
    SNRall = max(SNRall, reshape(SNR, size(EE)));    % BS mais próxima = maior SNR
end

%% ── Plot por cenário ──────────────────────────────────────────────────
for s = 1:numel(S)
    sc = S{s};
    f  = figure('Position', [80 80 1500 460], 'Color', 'w', 'Visible', 'off');

    imagesc(Evec, Nvec, SNRall); set(gca, 'YDir', 'normal'); hold on;
    colormap(turbo);
    cb = colorbar; cb.Label.String = 'SNR [dB]';
    caxis([0 45]);

    % contorno do limiar de SNR (link inutilizável abaixo)
    [~, hc] = contour(EE, NN, SNRall, [base.thresholdSNR base.thresholdSNR], ...
                      'w--', 'LineWidth', 1.6);

    % hotspots de risco  ([North East amp sigma])
    hHot = [];
    if ~isempty(base.manualHotspots)
        hs   = base.manualHotspots;
        hHot = scatter(hs(:,2), hs(:,1), 26, 'filled', ...
                       'MarkerFaceColor', [1 0.25 0.25], ...
                       'MarkerEdgeColor', 'k');
    end

    % rotas + início/fim
    cols = lines(numel(sc.routes));
    hRoute = gobjects(1, numel(sc.routes)); lblRoute = cell(1, numel(sc.routes));
    for r = 1:numel(sc.routes)
        rt = sc.routes(r);
        hRoute(r) = plot([rt.start(2) rt.goal(2)], [rt.start(1) rt.goal(1)], ...
                         '-', 'Color', cols(r,:), 'LineWidth', 2.6);
        lblRoute{r} = rt.label;
        hSt = plot(rt.start(2), rt.start(1), 'o', 'MarkerSize', 11, ...
                   'MarkerFaceColor', [0.10 0.85 0.10], 'MarkerEdgeColor', 'k', 'LineWidth', 1.2);
        hEn = plot(rt.goal(2),  rt.goal(1),  's', 'MarkerSize', 12, ...
                   'MarkerFaceColor', [0.95 0.10 0.10], 'MarkerEdgeColor', 'k', 'LineWidth', 1.2);
    end

    % gNB(s)
    hBS = plot(base.microBSPos(:,1), base.microBSPos(:,2), '^', 'MarkerSize', 14, ...
               'MarkerFaceColor', [1 1 0], 'MarkerEdgeColor', 'k', 'LineWidth', 1.6);

    % legenda (rotas + marcadores genéricos)
    legH = [hRoute, hSt, hEn, hBS, hc];
    legL = [lblRoute, {'Início', 'Fim', 'gNB', sprintf('SNR=%g dB', base.thresholdSNR)}];
    if ~isempty(hHot), legH(end+1) = hHot; legL{end+1} = 'Hotspots'; end
    legend(legH, legL, 'Location', 'eastoutside', 'Box', 'on');

    axis equal; axis([0 15000 -1300 1400]);
    xlabel('East [m]'); ylabel('North [m]');
    title(sprintf('Cenário "%s" — SNR, gNB, rotas e hotspots', sc.label), ...
          'Interpreter', 'none');
    set(gca, 'Layer', 'top', 'FontSize', 11);

    outPng = fullfile(OUT, sprintf('scenario_%s.png', sc.label));
    try
        exportgraphics(f, outPng, 'Resolution', 150);
    catch
        print(f, outPng, '-dpng', '-r150');
    end
    close(f);
    fprintf('[PLOT] %s\n', outPng);
end

fprintf('[PLOT] Concluído: %d cenários em %s\n', numel(S), OUT);
end


function r = mkroute(startNE, goalNE, label)
% mkroute  Rota com início/fim em [North, East].
r.start = startNE;
r.goal  = goalNE;
r.label = label;
end
