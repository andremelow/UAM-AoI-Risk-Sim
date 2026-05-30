%% URBAN_CIRCUIT_SWEEP  Varia a quantidade de drones e compara AoI (RR).
%
%  Executa urban_circuit_sim para cada N em drone_counts e plota
%  curvas comparativas de AoI média, pico e taxa de entrega.
%
%  Uso:
%    urban_circuit_sweep()              % N = [1 2 3 5 8]
%    urban_circuit_sweep([1 3 5 10])    % N personalizados

function urban_circuit_sweep(drone_counts)

if nargin < 1
    drone_counts = [1 2 3 5 8];
end

fprintf('╔══════════════════════════════════════════════════╗\n');
fprintf('║       Urban Circuit — Varredura de N drones      ║\n');
fprintf('╠══════════════════════════════════════════════════╣\n');
fprintf('║  N testados: %-34s  ║\n', mat2str(drone_counts));
fprintf('╚══════════════════════════════════════════════════╝\n\n');

nRuns   = numel(drone_counts);
res_all = cell(nRuns, 1);

% Desabilita dashboards durante a varredura (fecha figuras intermediárias)
set(0, 'DefaultFigureVisible', 'off');

for r = 1:nRuns
    N = drone_counts(r);
    fprintf('\n══ Rodada %d/%d — N = %d drones ══\n', r, nRuns, N);
    res_all{r} = urban_circuit_sim(N);
    close all;  % fecha dashboard intermediário
end

set(0, 'DefaultFigureVisible', 'on');

%% ── Figura comparativa ──────────────────────────────────────────────────
colors = lines(nRuns);

fig = figure('Color', 'w', 'Position', [60 60 1280 760], ...
    'Name', 'Comparação — N drones — Round-Robin', 'NumberTitle', 'off');
sgtitle('Impacto do Número de Drones na AoI — Round-Robin', ...
    'FontSize', 13, 'FontWeight', 'bold');

% 1: AoI C2 média vs tempo (curvas sobrepostas)
ax1 = subplot(2, 3, 1);
hold on;
for r = 1:nRuns
    R = res_all{r};
    if ~isempty(R.timeHist)
        plot(R.timeHist, R.meanAoiHist, '-', 'Color', colors(r,:), ...
             'LineWidth', 1.8, 'DisplayName', sprintf('N=%d', drone_counts(r)));
    end
end
grid on; xlabel('Tempo (s)'); ylabel('AoI C2 média (ms)');
title('AoI C2 média (por cenário)');
legend('Location', 'northeast');

% 2: AoI C2 pico vs tempo
ax2 = subplot(2, 3, 2);
hold on;
for r = 1:nRuns
    R = res_all{r};
    if ~isempty(R.timeHist)
        plot(R.timeHist, R.peakAoiHist, '-', 'Color', colors(r,:), ...
             'LineWidth', 1.8, 'DisplayName', sprintf('N=%d', drone_counts(r)));
    end
end
grid on; xlabel('Tempo (s)'); ylabel('AoI C2 pico (ms)');
title('AoI C2 pico (por cenário)');
legend('Location', 'northeast');

% 3: AoI vídeo média vs tempo
ax3 = subplot(2, 3, 3);
hold on;
for r = 1:nRuns
    R = res_all{r};
    if ~isempty(R.timeHist)
        plot(R.timeHist, R.meanAoiVidHist, '-', 'Color', colors(r,:), ...
             'LineWidth', 1.8, 'DisplayName', sprintf('N=%d', drone_counts(r)));
    end
end
grid on; xlabel('Tempo (s)'); ylabel('AoI Vídeo média (ms)');
title('AoI Vídeo média (por cenário)');
legend('Location', 'northeast');

% 4: AoI C2 média final vs N (barra)
ax4 = subplot(2, 3, 4);
y_mean = arrayfun(@(r) res_all{r}.meanAoI_c2_ms, 1:nRuns);
y_peak = arrayfun(@(r) res_all{r}.peakAoI_c2_ms, 1:nRuns);
bar_h = bar(ax4, drone_counts, [y_mean(:), y_peak(:)]);
bar_h(1).FaceColor = [0.2 0.6 0.9];
bar_h(2).FaceColor = [0.9 0.3 0.3];
grid(ax4, 'on');
xlabel(ax4, 'Número de drones (N)'); ylabel(ax4, 'AoI C2 (ms)');
title(ax4, 'AoI C2 Média e Pico vs N');
legend(ax4, {'Média', 'Pico'}, 'Location', 'northwest');

% 5: Taxa de entrega vs N
ax5 = subplot(2, 3, 5);
y_del = arrayfun(@(r) res_all{r}.deliveryRate * 100, 1:nRuns);
plot(ax5, drone_counts, y_del, 'ko-', 'LineWidth', 2, 'MarkerSize', 8, ...
     'MarkerFaceColor', 'k');
grid(ax5, 'on');
xlabel(ax5, 'Número de drones (N)'); ylabel(ax5, 'Taxa de entrega (%)');
title(ax5, 'Taxa de Entrega vs N');
ylim(ax5, [0 110]);
yline(ax5, 95, 'r--', 'Meta 95%', 'LabelHorizontalAlignment', 'left');

% 6: Tabela de resumo
ax6 = subplot(2, 3, 6);
axis(ax6, 'off');
rows = cell(nRuns + 1, 5);
rows(1,:) = {'N', 'AoI C2 média (ms)', 'AoI C2 pico (ms)', ...
             'AoI Vid (ms)', 'Entrega (%)'};
for r = 1:nRuns
    R = res_all{r};
    rows(r+1, :) = {
        drone_counts(r), ...
        sprintf('%.1f', R.meanAoI_c2_ms), ...
        sprintf('%.1f', R.peakAoI_c2_ms), ...
        sprintf('%.1f', R.meanAoI_vid_ms), ...
        sprintf('%.1f', R.deliveryRate * 100)
    };
end
t = uitable('Parent', fig, ...
    'Data', rows(2:end, :), ...
    'ColumnName', rows(1,:), ...
    'ColumnWidth', {40, 130, 130, 100, 90}, ...
    'Units', 'normalized', ...
    'Position', [0.67 0.04 0.32 0.42], ...
    'FontSize', 9);

%% ── Imprime tabela no terminal ──────────────────────────────────────────
fprintf('\n%s\n', repmat('─', 1, 68));
fprintf('%-5s  %-18s  %-18s  %-14s  %-10s\n', ...
    'N', 'AoI C2 média (ms)', 'AoI C2 pico (ms)', 'AoI Vid (ms)', 'Entrega (%)');
fprintf('%s\n', repmat('─', 1, 68));
for r = 1:nRuns
    R = res_all{r};
    fprintf('%-5d  %-18.1f  %-18.1f  %-14.1f  %-10.1f\n', ...
        drone_counts(r), R.meanAoI_c2_ms, R.peakAoI_c2_ms, ...
        R.meanAoI_vid_ms, R.deliveryRate * 100);
end
fprintf('%s\n\n', repmat('─', 1, 68));

end
