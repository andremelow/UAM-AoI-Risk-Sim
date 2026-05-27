function results = urban_circuit_sim(numDrones)
%% URBAN_CIRCUIT_SIM  Simulador UAM urbano com gNB único, circuito de 2 min
%                     e escalonamento Round-Robin com cálculo de AoI.
%
%  Uso:
%    urban_circuit_sim()          % padrão: 3 drones
%    urban_circuit_sim(5)         % 5 drones
%    urban_circuit_sim(10)        % 10 drones
%
%  Cenário:
%    - 1 gNB (gNodeB 5G NR) em ambiente urbano UMi (3GPP TR 38.901)
%    - Drones voam em circuito fechado de ~2 min (400 m × 400 m)
%    - Cada drone entra separado por 20 s (chegadas determinísticas)
%    - Round-Robin entre todos os drones ativos
%    - AoI calculada por drone e agregada (média e pico)

clc; close all;

if nargin < 1
    numDrones = 3;
end

fprintf('╔══════════════════════════════════════════════╗\n');
fprintf('║      Urban Circuit Simulator — RR + AoI      ║\n');
fprintf('╠══════════════════════════════════════════════╣\n');
fprintf('║  Drones : %-4d                               ║\n', numDrones);
fprintf('║  Política: Round-Robin                       ║\n');
fprintf('║  Circuito: 2 min (~400 m × 400 m)            ║\n');
fprintf('║  Cenário : UMi (5G NR, 3.5 GHz)              ║\n');
fprintf('╚══════════════════════════════════════════════╝\n\n');

%% ── 1. CONFIGURAÇÃO ──────────────────────────────────────────────────────
cfg = build_urban_config(numDrones);

%% ── 2. CENÁRIO E DRONES ──────────────────────────────────────────────────
[scene, drones, infra] = build_urban_scene(cfg);

%% ── 3. ESTADO INICIAL ────────────────────────────────────────────────────
state = init_urban_state(cfg, infra);

%% ── 4. CANAL (path loss UMi) ─────────────────────────────────────────────
plCfg = nrPathLossConfig;
plCfg.Scenario = 'UMi';

%% ── 5. DASHBOARD ─────────────────────────────────────────────────────────
db = create_dashboard(cfg);

%% ── 6. LOOP PRINCIPAL ────────────────────────────────────────────────────
fprintf('[SIM] Iniciando simulação...\n');
dt = 1 / cfg.updateRate;

while advance(scene)
    time = scene.CurrentTime;

    % 6a. Lê posições e atualiza conjunto ativo
    state = read_positions(state, drones, time, cfg);

    % 6b. Gera pacotes C2 (generate-at-will)
    % C2 sempre tem pacote disponível — sem estado adicional necessário.

    % 6c. Escalonamento Round-Robin
    [txDrone, state] = rr_schedule(state, time, cfg);

    % 6d. Canal — SNR para todos os drones ativos
    state = compute_channel(state, infra, plCfg, cfg);

    % 6e. Transmissão e atualização de AoI
    state = transmit_and_aoi(state, txDrone, time, dt, cfg);

    % 6f. Acumula histórico SNR em todo slot (resolução total)
    for i = state.activeDrones
        snr_i = state.snr(i);
        if ~isnan(snr_i)
            db.snrX_hist{i}(end+1) = state.pos(i, 1);
            db.snrY_hist{i}(end+1) = snr_i;
        end
    end

    % 6g. Atualiza dashboard periodicamente
    if mod(round(time * cfg.updateRate), cfg.dashboardRate) == 0
        update_urban_dashboard(db, state, time, cfg);
    end
end

%% ── 7. PÓS-SIMULAÇÃO ─────────────────────────────────────────────────────
results = collect_urban_results(state, cfg);
fprintf('\n[SIM] Completo.\n');
end

%% ═══════════════════════════════════════════════════════════════════════
%%  FUNÇÕES LOCAIS
%% ═══════════════════════════════════════════════════════════════════════

function cfg = build_urban_config(N)
%% BUILD_URBAN_CONFIG  Parâmetros do cenário urbano com circuito de 2 min.

% --- Geral ---
cfg.numDrones        = N;
cfg.schedulingPolicy = 'round-robin';
cfg.dronesPerSlot    = 1;          % RR: 1 drone por slot
cfg.updateRate       = 100;        % 100 slots/s → dt = 10 ms
cfg.dashboardRate    = 50;         % atualiza display a cada 50 slots

% --- Circuito urbano 400m × 400m ---
% Waypoints (retângulo): Sul → Leste → Norte → Oeste → Sul
cfg.circuit = [-200 -200  -50;   % SW (ponto de entrada)
                200 -200  -50;   % SE
                200  200  -50;   % NE
               -200  200  -50;   % NW
               -200 -200  -50];  % SW (fecha o circuito)

cfg.circuitPerim     = 4 * 400;   % 1600 m
cfg.circuitTime      = 120;       % 2 min (s)
cfg.droneSpeed       = cfg.circuitPerim / cfg.circuitTime; % ≈ 13.3 m/s

% --- Chegadas determinísticas: uma a cada 20 s ---
cfg.entryInterval    = 20;        % s entre chegadas
cfg.entrySpread      = 2;         % jitter ±2 s

% Tempo total: último drone entra em (N-1)*20 s + 1 circuito + buffer
cfg.simDuration      = (N - 1) * cfg.entryInterval + cfg.circuitTime + 30;

% Cada drone voa exatamente 2 circuitos (4 min) ou até o fim da sim
cfg.flightDuration   = cfg.circuitTime * 2;  % 4 min por drone

% --- gNB único no centro do cenário urbano ---
cfg.gnbPos           = [0  0  -30];    % [x y z] em metros (z negativo = altura)

% --- Rádio 5G NR UMi ---
cfg.fc               = 3.5e9;      % 3.5 GHz
cfg.bw               = 20e6;       % 20 MHz
cfg.pTransmit        = 23;         % dBm (drone UE)
cfg.gnbGain          = 15;         % dBi
cfg.noiseFigure      = 7;          % dB
cfg.thermalNoise     = -174 + 10*log10(cfg.bw);  % dBm
cfg.thresholdSNR     = 5;          % dB

% --- Parâmetros AoI ---
cfg.r_min            = 10;         % m (raio mínimo de incerteza)
cfg.v_max            = cfg.droneSpeed;

% --- Vídeo (modelo atômico) ---
cfg.videoModel       = 'atomic';
cfg.fps              = 30;
cfg.L_vid            = 1;          % 1 pacote por frame
cfg.L_c2             = 1;          % 1 pacote C2
cfg.p_c2             = 0.9;
cfg.p_vid            = 0.85;
cfg.frame_interval   = 1 / cfg.fps;

fprintf('[CFG] Duração da simulação: %.0f s\n', cfg.simDuration);
fprintf('[CFG] Velocidade dos drones: %.1f m/s\n', cfg.droneSpeed);
fprintf('[CFG] Slots por ciclo RR: %d\n', N);
end

% ─────────────────────────────────────────────────────────────────────────

function [scene, drones, infra] = build_urban_scene(cfg)
%% BUILD_URBAN_SCENE  Cria uavScenario com 1 gNB e N drones em circuito.

N = cfg.numDrones;

% --- Cena ---
scene = uavScenario( ...
    UpdateRate   = cfg.updateRate, ...
    StopTime     = cfg.simDuration, ...
    MaxNumFrames = 500);

% --- gNB (plataforma estática) ---
gnb = uavPlatform('gNB', scene, InitialPosition = cfg.gnbPos);
updateMesh(gnb, 'cuboid', {[10 10 40]}, [0 0.4 0.8], [0 0 20], [1 0 0 0]);
fprintf('[SCENE] gNB posicionado em [%.0f, %.0f, %.0f] m\n', cfg.gnbPos);

% --- Chegadas determinísticas (espaçadas por entryInterval s) ---
rng(42);  % reprodutibilidade
startTimes = zeros(N, 1);
for k = 1:N
    jitter = (rand() - 0.5) * 2 * cfg.entrySpread;
    startTimes(k) = (k - 1) * cfg.entryInterval + jitter;
    startTimes(k) = max(startTimes(k), 1);
end
endTimes = startTimes + cfg.flightDuration;

fprintf('[SCENE] Chegadas dos drones:\n');
for k = 1:N
    fprintf('  Drone %2d: entra em t = %5.1f s, sai em t = %5.1f s\n', ...
            k, startTimes(k), endTimes(k));
end

% --- Cria trajetórias em circuito ---
% Cada drone começa no ponto de entrada (SW) desfasado na fase
% proporcional ao intervalo de entrada.
drones = cell(N, 1);
waypoints = cfg.circuit;

for k = 1:N
    % Fase inicial no circuito: distribui drones uniformemente
    % (cada drone separado por 20 s → deslocamento angular)
    phaseOffset = mod((k - 1) * cfg.entryInterval / cfg.circuitTime, 1);
    toa = build_circuit_toa(startTimes(k), endTimes(k), ...
                            cfg.circuitTime, phaseOffset, waypoints);

    traj = waypointTrajectory(toa.waypoints, ...
           TimeOfArrival = toa.times);
    drones{k} = uavPlatform('UAV' + string(k), scene, Trajectory = traj);
    updateMesh(drones{k}, 'quadrotor', {3}, [0.2 0.2 0.2], [0 0 0], [1 0 0 0]);
end
setup(scene);

% --- Estrutura de infraestrutura ---
infra.gnbPos    = cfg.gnbPos;
infra.startTimes = startTimes;
infra.endTimes   = endTimes;
end

% ─────────────────────────────────────────────────────────────────────────

function toa = build_circuit_toa(tStart, tEnd, circuitTime, phaseOffset, wpts)
%% BUILD_CIRCUIT_TOA  Constrói tabela de waypoints para circuito fechado.
%
%  O drone começa no waypoint correspondente à fase e circula até tEnd.

nWpts   = size(wpts, 1) - 1;  % -1 porque primeiro e último são iguais (fecha)
segLen  = circuitTime / nWpts; % tempo por segmento

% Ponto de entrada na fase dada
startSeg = floor(phaseOffset * nWpts);
startFrac = phaseOffset * nWpts - startSeg;

tList  = tStart;
pList  = wpts(mod(startSeg, nWpts) + 1, :);  % posição inicial

% Primeiro segmento (parcial)
firstSegRemain = segLen * (1 - startFrac);
pNext = wpts(mod(startSeg + 1, nWpts) + 1, :);
tList(end+1) = tStart + firstSegRemain;
pList(end+1, :) = pNext;

% Segmentos completos até tEnd
t_cur = tStart + firstSegRemain;
seg   = mod(startSeg + 1, nWpts);
while t_cur + segLen <= tEnd + 0.5
    seg    = mod(seg + 1, nWpts);
    t_cur  = t_cur + segLen;
    pList(end+1, :) = wpts(mod(seg, nWpts) + 1, :); %#ok<AGROW>
    tList(end+1) = t_cur; %#ok<AGROW>
end

% Garante que o último waypoint é exatamente em tEnd
tList(end)   = tEnd;
pList(end, :) = pList(end, :);

toa.waypoints = pList;
toa.times     = tList(:);
end

% ─────────────────────────────────────────────────────────────────────────

function state = init_urban_state(cfg, infra)
%% INIT_URBAN_STATE  Inicializa estado mutável da simulação.

N = cfg.numDrones;

% --- AoI por drone ---
for u = N:-1:1
    state.aoi(u).h        = 0;      % AoI C2 em slots
    state.aoi(u).h_vid    = 0;      % AoI vídeo em slots
    state.aoi(u).lastRx   = NaN;
    state.aoi(u).lastRx_vid = NaN;
    state.aoi(u).vid_remaining = 0;
    state.aoi(u).vid_gen_time  = 0;
    state.aoi(u).next_frame_t  = infra.startTimes(u);
    state.aoi(u).txCount  = 0;
    state.aoi(u).failCount = 0;
end

% --- Canal ---
state.snr        = nan(N, 1);
state.capacity   = nan(N, 1);  % Mbps

% --- Posições ---
state.pos        = zeros(N, 3);
state.valid      = false(N, 1);
state.activeDrones = [];

% --- RR ---
state.rrPointer  = 1;

% --- Histórico para plots ---
state.timeHist   = [];
for u = 1:N
    state.aoiHist{u}    = [];   % C2 AoI (ms)
    state.aoiVidHist{u} = [];   % Video AoI (ms)
    state.timePerDrone{u} = [];
end
state.meanAoiHist    = [];
state.peakAoiHist    = [];
state.meanAoiVidHist = [];
state.snrHist        = [];  % SNR médio

% --- Timing ---
state.startTimes = infra.startTimes;
state.endTimes   = infra.endTimes;
end

% ─────────────────────────────────────────────────────────────────────────

function state = read_positions(state, drones, time, cfg)
%% READ_POSITIONS  Lê posições, atualiza conjunto ativo.

N = cfg.numDrones;
state.activeDrones = [];

for i = 1:N
    % Drone ainda não chegou ou já saiu
    if time < state.startTimes(i) || time > state.endTimes(i)
        state.valid(i) = false;
        continue;
    end

    [posq, ~] = read(drones{i});
    if any(isnan(posq)), state.valid(i) = false; continue; end

    state.pos(i, :)    = posq(1:3);
    state.valid(i)     = true;
    state.activeDrones(end+1) = i; %#ok<AGROW>
end
end

% ─────────────────────────────────────────────────────────────────────────

function [txDrone, state] = rr_schedule(state, ~, cfg)
%% RR_SCHEDULE  Round-Robin simples entre drones ativos.
%
%  Retorna o índice do drone selecionado (ou 0 se nenhum ativo).

active = state.activeDrones;
if isempty(active)
    txDrone = 0;
    return;
end

n   = numel(active);
ptr = mod(state.rrPointer - 1, n) + 1;
txDrone = active(ptr);
state.rrPointer = mod(ptr, n) + 1;
end

% ─────────────────────────────────────────────────────────────────────────

function state = compute_channel(state, infra, plCfg, cfg)
%% COMPUTE_CHANNEL  SNR e capacidade para todos os drones ativos.

for i = state.activeDrones
    posUE = state.pos(i, :);

    % gNB único
    posBS_ned = [infra.gnbPos(2); infra.gnbPos(1); -infra.gnbPos(3)];
    posUE_ned = [posUE(2); posUE(1); -posUE(3)];

    PL  = nrPathLoss(plCfg, cfg.fc, true, posBS_ned, posUE_ned);
    SNR = (cfg.pTransmit + cfg.gnbGain - PL) - ...
          (cfg.thermalNoise + cfg.noiseFigure);

    state.snr(i)      = SNR;
    state.capacity(i) = (cfg.bw * log2(1 + 10^(SNR/10))) / 1e6;
end
end

% ─────────────────────────────────────────────────────────────────────────

function state = transmit_and_aoi(state, txDrone, time, dt, cfg)
%% TRANSMIT_AND_AOI  Transmissão RR e evolução da AoI (C2 e vídeo).
%
%  AoI é medida em slots e convertida para ms nos históricos.

ms_per_slot = 1000 / cfg.updateRate;

% --- Gera frames de vídeo para drones ativos ---
for i = state.activeDrones
    if time >= state.aoi(i).next_frame_t
        % Novo frame disponível (modo atômico: L=1 pacote)
        state.aoi(i).vid_remaining = cfg.L_vid;
        state.aoi(i).vid_gen_time  = time;
        state.aoi(i).next_frame_t  = time + cfg.frame_interval;
    end
end

% --- Transmissão ---
for i = state.activeDrones
    isMyTurn = (txDrone == i);

    if isMyTurn && ~isnan(state.snr(i)) && state.snr(i) > cfg.thresholdSNR
        % Transmissão bem-sucedida → reset AoI C2
        state.aoi(i).h      = 0;
        state.aoi(i).lastRx = time;
        state.aoi(i).txCount = state.aoi(i).txCount + 1;

        % Tenta transmitir pacote de vídeo se C2 já atualizado
        if state.aoi(i).vid_remaining > 0
            state.aoi(i).vid_remaining = state.aoi(i).vid_remaining - 1;
            if state.aoi(i).vid_remaining == 0
                elapsed = time - state.aoi(i).vid_gen_time;
                state.aoi(i).h_vid = round(elapsed * cfg.updateRate);
                state.aoi(i).lastRx_vid = time;
            end
        end

    elseif isMyTurn
        state.aoi(i).failCount = state.aoi(i).failCount + 1;
    end

    % --- AoI aging ---
    state.aoi(i).h     = state.aoi(i).h     + 1;
    state.aoi(i).h_vid = state.aoi(i).h_vid + 1;

    % --- Histórico por drone ---
    state.timePerDrone{i}(end+1) = time;
    state.aoiHist{i}(end+1)     = state.aoi(i).h     * ms_per_slot;
    state.aoiVidHist{i}(end+1)  = state.aoi(i).h_vid * ms_per_slot;
end

% --- Métricas agregadas ---
if ~isempty(state.activeDrones)
    h_vals = arrayfun(@(u) state.aoi(u).h * ms_per_slot, state.activeDrones);
    state.timeHist(end+1)      = time;
    state.meanAoiHist(end+1)   = mean(h_vals);
    state.peakAoiHist(end+1)   = max(h_vals);
    h_vid_vals = arrayfun(@(u) state.aoi(u).h_vid * ms_per_slot, state.activeDrones);
    state.meanAoiVidHist(end+1) = mean(h_vid_vals);
    snr_vals = state.snr(state.activeDrones);
    state.snrHist(end+1) = mean(snr_vals(~isnan(snr_vals)));
end
end

% ─────────────────────────────────────────────────────────────────────────

function db = create_dashboard(cfg)
%% CREATE_DASHBOARD  Painel de monitoramento em tempo real.

N = cfg.numDrones;
colors = lines(max(N, 1));

db.fig = figure('Color', [0.08 0.09 0.11], ...
    'Position', [40 40 1400 700], ...
    'Name', sprintf('Urban Circuit — %d Drones — Round-Robin', N), ...
    'NumberTitle', 'off');

% Layout: 2 linhas × 3 colunas
% Row 1: top-view (cols 1-2) | SNR vs X (col 3)
% Row 2: AoI C2 por drone (cols 1-3, full width)
tl = tiledlayout(db.fig, 2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf('Simulador UAM Urbano  |  gNB único  |  Round-Robin  |  N = %d drones', N), ...
    'Color', 'w', 'FontSize', 13, 'FontWeight', 'bold');

axprop = {'Color',[0.1 0.11 0.14],'XColor','w','YColor','w', ...
          'GridColor',[0.3 0.3 0.3],'GridAlpha',0.4};

% ── [Row1, Col1-2]: Top-view aérea ───────────────────────────────────
db.ax_scene = nexttile(tl, 1, [1 2]);
hold(db.ax_scene, 'on');
set(db.ax_scene, axprop{:});
grid(db.ax_scene, 'on');

circuit = cfg.circuit;
plot(db.ax_scene, circuit(:,1), circuit(:,2), '--', ...
     'Color', [0.4 0.4 0.4], 'LineWidth', 1.2);

% gNB mestre (estrela amarela)
plot(db.ax_scene, 0, 0, 'p', 'Color', [1 0.9 0.1], ...
     'MarkerSize', 15, 'MarkerFaceColor', [1 0.9 0.1]);
text(db.ax_scene, 12, 12, 'gNB', 'Color', [1 0.9 0.1], 'FontSize', 9);

% micro-BS
if isfield(cfg, 'microBSPos')
    for b = 1:size(cfg.microBSPos,1)
        plot(db.ax_scene, cfg.microBSPos(b,1), cfg.microBSPos(b,2), '^', ...
            'Color',[0.3 0.7 1],'MarkerSize',10,'MarkerFaceColor',[0.3 0.7 1]);
    end
end

db.h_trails   = gobjects(N, 1);
db.h_aoi_circ = gobjects(N, 1);
db.h_drones   = gobjects(N, 1);
for k = 1:N
    db.h_trails(k)   = plot(db.ax_scene, NaN, NaN, '-', ...
        'Color', [colors(k,:) 0.30], 'LineWidth', 1);
    db.h_aoi_circ(k) = plot(db.ax_scene, NaN, NaN, '--', ...
        'Color', colors(k,:), 'LineWidth', 1.4);
    db.h_drones(k)   = plot(db.ax_scene, NaN, NaN, 'o', ...
        'Color', colors(k,:), 'MarkerSize', 10, 'MarkerFaceColor', colors(k,:));
end
db.h_scheduled = plot(db.ax_scene, NaN, NaN, 'o', ...
    'Color', 'y', 'MarkerSize', 16, 'LineWidth', 2);
xlabel(db.ax_scene, 'X (m)', 'Color', 'w');
ylabel(db.ax_scene, 'Y (m)', 'Color', 'w');
title(db.ax_scene, 'Vista aérea — círculo = raio de incerteza AoI', 'Color', 'w');
xlim(db.ax_scene, [-400 400]); ylim(db.ax_scene, [-400 400]);
axis(db.ax_scene, 'equal');
db.trail_x   = cell(N, 1);
db.trail_y   = cell(N, 1);
db.aoi_theta = linspace(0, 2*pi, 40);

% ── [Row1, Col3]: SNR vs posição X ───────────────────────────────────
db.ax_snrX = nexttile(tl, 3);
hold(db.ax_snrX, 'on');
set(db.ax_snrX, axprop{:});
grid(db.ax_snrX, 'on');
yline(db.ax_snrX, cfg.thresholdSNR, 'r--', 'Thresh', ...
    'LabelHorizontalAlignment','left','LabelColor','w','LineWidth',1.2);
db.h_snrX = gobjects(N, 1);
for k = 1:N
    db.h_snrX(k) = plot(db.ax_snrX, NaN, NaN, '.', ...
        'Color', colors(k,:), 'MarkerSize', 3);
end
xlabel(db.ax_snrX, 'X (m)', 'Color', 'w');
ylabel(db.ax_snrX, 'SNR (dB)', 'Color', 'w');
title(db.ax_snrX, 'SNR vs Posição X', 'Color', 'w');
% Buffers preenchidos no loop principal (step 6f), não aqui
db.snrX_hist = cell(N,1);
db.snrY_hist = cell(N,1);
for k = 1:N, db.snrX_hist{k}=[]; db.snrY_hist{k}=[]; end

% ── [Row2, Col1-3]: AoI C2 por drone (full width) ────────────────────
db.ax_aoi = nexttile(tl, 4, [1 3]);
hold(db.ax_aoi, 'on');
set(db.ax_aoi, axprop{:});
grid(db.ax_aoi, 'on');
db.h_aoi = gobjects(N, 1);
for k = 1:N
    db.h_aoi(k) = plot(db.ax_aoi, NaN, NaN, '-', ...
        'Color', colors(k,:), 'LineWidth', 1.5, ...
        'DisplayName', sprintf('Drone %d', k));
end
db.h_aoi_mean = plot(db.ax_aoi, NaN, NaN, 'w--', ...
    'LineWidth', 2.5, 'DisplayName', 'Média');
legend(db.ax_aoi, 'TextColor','w','Color',[0.1 0.11 0.14], ...
    'Location','northeast','FontSize',8);
xlabel(db.ax_aoi, 'Tempo (s)', 'Color', 'w');
ylabel(db.ax_aoi, 'AoI C2 (ms)', 'Color', 'w');
title(db.ax_aoi, 'Age of Information — Controle C2 (por drone)', 'Color', 'w');

% ── Janela separada: painel de status dos drones ──────────────────────
db.fig_status = figure('Color',[0.07 0.08 0.10], ...
    'Position', [1460 40 420 700], ...
    'Name', 'Drone Status Panel', ...
    'NumberTitle', 'off', ...
    'MenuBar', 'none', 'ToolBar', 'none');
db.hDroneList = uicontrol(db.fig_status, ...
    'Style', 'listbox', ...
    'Units', 'normalized', 'Position', [0.01 0.01 0.98 0.94], ...
    'BackgroundColor',[0.07 0.08 0.10], ...
    'ForegroundColor',[0.75 0.80 0.90], ...
    'FontName','Courier New', 'FontSize', 9, ...
    'String', {'Aguardando dados...'}, ...
    'Enable','inactive', 'Max', 2);
uicontrol(db.fig_status, 'Style','text', ...
    'Units','normalized','Position',[0.01 0.95 0.98 0.04], ...
    'String','DRONE STATUS — live', ...
    'BackgroundColor',[0.07 0.08 0.10], ...
    'ForegroundColor',[0.30 0.85 0.65], ...
    'FontName','Courier New','FontSize',10,'FontWeight','bold', ...
    'HorizontalAlignment','center');

db.colors = colors;
db.N = N;
end

% ─────────────────────────────────────────────────────────────────────────

function update_urban_dashboard(db, state, time, cfg)
%% UPDATE_URBAN_DASHBOARD  Atualiza dashboard principal + janela de status.

N = db.N;
active = state.activeDrones;
ms_per_slot = 1000 / cfg.updateRate;

% ── Visão aérea + círculos de AoI ────────────────────────────────────
scheduled_pos = [NaN NaN];
ptr = state.rrPointer;
for k = 1:N
    if state.valid(k)
        x = state.pos(k, 1);
        y = state.pos(k, 2);

        % Marcador
        set(db.h_drones(k), 'XData', x, 'YData', y);

        % Trail (últimos 80 pontos)
        db.trail_x{k}(end+1) = x;
        db.trail_y{k}(end+1) = y;
        if numel(db.trail_x{k}) > 80
            db.trail_x{k} = db.trail_x{k}(end-79:end);
            db.trail_y{k} = db.trail_y{k}(end-79:end);
        end
        set(db.h_trails(k), 'XData', db.trail_x{k}, 'YData', db.trail_y{k});

        % Círculo de incerteza AoI
        h_ms  = state.aoi(k).h * ms_per_slot;
        r_aoi = cfg.r_min + cfg.v_max * (h_ms / 1000);
        cx = x + r_aoi * cos(db.aoi_theta);
        cy = y + r_aoi * sin(db.aoi_theta);
        set(db.h_aoi_circ(k), 'XData', cx, 'YData', cy);

        % Drone escalonado
        if ~isempty(active)
            n_act = numel(active);
            sched_k = active(mod(ptr - 2, n_act) + 1);
            if state.valid(sched_k)
                scheduled_pos = state.pos(sched_k, 1:2);
            end
        end

        % SNR vs X — usa histórico acumulado no loop principal
        if ~isempty(db.snrX_hist{k})
            set(db.h_snrX(k), 'XData', db.snrX_hist{k}, 'YData', db.snrY_hist{k});
        end
    else
        set(db.h_drones(k),   'XData', NaN, 'YData', NaN);
        set(db.h_aoi_circ(k), 'XData', NaN, 'YData', NaN);
    end
end
set(db.h_scheduled, 'XData', scheduled_pos(1), 'YData', scheduled_pos(2));

% ── AoI por drone ─────────────────────────────────────────────────────
for k = 1:N
    if ~isempty(state.timePerDrone{k})
        set(db.h_aoi(k), 'XData', state.timePerDrone{k}, 'YData', state.aoiHist{k});
    end
end
if ~isempty(state.timeHist)
    set(db.h_aoi_mean, 'XData', state.timeHist, 'YData', state.meanAoiHist);
end

% ── Janela separada: painel de status dos drones ──────────────────────
if ishandle(db.fig_status)
    lines = cell(N, 1);
    for k = 1:N
        if state.valid(k)
            h_ms  = state.aoi(k).h * ms_per_slot;
            snr_k = state.snr(k);
            tot = state.aoi(k).txCount + state.aoi(k).failCount;
            if tot > 0, fdr_k = state.aoi(k).txCount / tot; else, fdr_k = 1; end
            if isnan(snr_k), snr_str = '  N/A ';
            else,             snr_str = sprintf('%+5.1fdB', snr_k); end
            lines{k} = sprintf('UAV%2d  AoI:%5.0fms  FDR:%4.0f%%  SNR:%s', ...
                k, h_ms, fdr_k*100, snr_str);
        else
            if time < state.startTimes(k)
                lines{k} = sprintf('UAV%2d  [aguardando  t=%.0fs]', k, state.startTimes(k));
            else
                lines{k} = sprintf('UAV%2d  [pousado]', k);
            end
        end
    end
    set(db.hDroneList, 'String', lines);
end

drawnow limitrate;
end

% ─────────────────────────────────────────────────────────────────────────

function results = collect_urban_results(state, cfg)
%% COLLECT_URBAN_RESULTS  Empacota resultados finais.

N = cfg.numDrones;
ms_per_slot = 1000 / cfg.updateRate;

% AoI C2: concatena todos os históricos
all_aoi = [];
for k = 1:N
    all_aoi = [all_aoi, state.aoiHist{k}]; %#ok<AGROW>
end

if isempty(all_aoi)
    warning('Sem dados de AoI — simulação pode ter terminado cedo.');
    all_aoi = [0];
end

results.timeHist       = state.timeHist;
results.meanAoiHist    = state.meanAoiHist;
results.peakAoiHist    = state.peakAoiHist;
results.meanAoiVidHist = state.meanAoiVidHist;
results.snrHist        = state.snrHist;

results.aoiPerDrone    = state.aoiHist;
results.aoiVidPerDrone = state.aoiVidHist;
results.timePerDrone   = state.timePerDrone;

results.meanAoI_c2_ms  = mean(all_aoi(all_aoi > 0));
results.peakAoI_c2_ms  = max(all_aoi);

all_vid = [];
for k = 1:N
    all_vid = [all_vid, state.aoiVidHist{k}]; %#ok<AGROW>
end
results.meanAoI_vid_ms = mean(all_vid(all_vid > 0));

% Taxa de entrega (tx bem-sucedidas / tentativas)
total_tx   = sum(arrayfun(@(u) state.aoi(u).txCount,   1:N));
total_fail = sum(arrayfun(@(u) state.aoi(u).failCount, 1:N));
total_att  = total_tx + total_fail;
results.deliveryRate = total_tx / max(total_att, 1);

results.txCount  = arrayfun(@(u) state.aoi(u).txCount,   1:N);
results.failCount = arrayfun(@(u) state.aoi(u).failCount, 1:N);
results.numDrones = N;
results.startTimes = state.startTimes;
end

% ─────────────────────────────────────────────────────────────────────────
