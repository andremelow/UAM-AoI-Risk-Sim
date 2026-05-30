function results = urban_linear_sim(numDrones)
%% URBAN_LINEAR_SIM  Simulador UAM — linha reta, 1 gNB, RR + AoI + qualidade de sinal.
%
%  Os drones voam em linha reta de x = -1000 m a x = +1000 m (y = offset,
%  altitude = 50 m). O gNB fica em (0, 0, 0). Isso torna a variação de
%  distância — e portanto de SNR, RSRP, path loss e AoI — completamente
%  observável e intuitiva.
%
%  Uso:
%    urban_linear_sim()       % padrão: 3 drones
%    urban_linear_sim(1)      % 1 drone (mostra todo o percurso)
%    urban_linear_sim(5)      % 5 drones

clc; close all;

if nargin < 1
    numDrones = 3;
end

fprintf('╔══════════════════════════════════════════════════╗\n');
fprintf('║   Urban Linear Sim — Linha Reta + SNR/AoI        ║\n');
fprintf('╠══════════════════════════════════════════════════╣\n');
fprintf('║  Drones  : %-4d                                  ║\n', numDrones);
fprintf('║  Rota    : x = -1000 m → +1000 m                ║\n');
fprintf('║  gNB     : (0, 0, 0) — UMi 3.5 GHz              ║\n');
fprintf('║  Política: Round-Robin                           ║\n');
fprintf('╚══════════════════════════════════════════════════╝\n\n');

%% ── 1. CONFIG ────────────────────────────────────────────────────────────
cfg = build_linear_config(numDrones);

%% ── 2. CENÁRIO ───────────────────────────────────────────────────────────
[scene, drones, infra] = build_linear_scene(cfg);

%% ── 3. ESTADO ────────────────────────────────────────────────────────────
state = init_linear_state(cfg, infra);

%% ── 4. PATH LOSS UMi ─────────────────────────────────────────────────────
plCfg          = nrPathLossConfig;
plCfg.Scenario = 'UMi';

%% ── 5. DASHBOARD ─────────────────────────────────────────────────────────
db = create_linear_dashboard(cfg);

%% ── 6. LOOP ──────────────────────────────────────────────────────────────
fprintf('[SIM] Iniciando...\n');

while advance(scene)
    time = scene.CurrentTime;

    state = lsim_read_positions(state, drones, time, cfg);
    [txDrone, state] = lsim_rr_schedule(state, cfg);
    state = lsim_channel(state, infra, plCfg, cfg);
    state = lsim_transmit_aoi(state, txDrone, time, cfg);

    if mod(round(time * cfg.updateRate), cfg.dashRate) == 0
        lsim_update_dashboard(db, state, time, cfg);
    end
end

%% ── 7. RESULTADOS ────────────────────────────────────────────────────────
results = lsim_collect(state, cfg);
fprintf('\n[SIM] Concluído.\n');
end

%% ═══════════════════════════════════════════════════════════════════════
%%  CONFIG
%% ═══════════════════════════════════════════════════════════════════════
function cfg = build_linear_config(N)

cfg.numDrones   = N;
cfg.updateRate  = 100;          % slots/s  (dt = 10 ms)
cfg.dashRate    = 20;           % atualiza display a cada 20 slots

% ── Rota linear ──────────────────────────────────────────────────────
cfg.xStart      = -3000;        % m
cfg.xEnd        =  3000;        % m
cfg.routeLen    =  6000;        % m
cfg.droneAlt    =  -50;         % m (negativo = acima do solo em NED)
cfg.droneSpeed  =  20;          % m/s  → travessia em 100 s
cfg.flightTime  = cfg.routeLen / cfg.droneSpeed;   % 100 s

% Separação entre drones: 20 s → offset de 20*20 = 400 m entre eles
cfg.entryInterval = 60;         % s
cfg.yOffset       = 20;         % m entre faixas laterais dos drones
cfg.jitter        = 1;          % s de jitter na chegada

% Duração total da simulação
cfg.simDuration = (N - 1) * cfg.entryInterval + cfg.flightTime + 10;

% ── gNB ──────────────────────────────────────────────────────────────
cfg.gnbPos      = [0  0  0];    % centro da rota, ao nível do solo

% ── Rádio 5G NR UMi ─────────────────────────────────────────────────
cfg.fc          = 3.5e9;
cfg.bw          = 20e6;
cfg.pTx         = 23;           % dBm (drone UE)
cfg.gnbGain     = 15;           % dBi
cfg.noiseFig    = 7;            % dB
cfg.kT          = -174 + 10*log10(cfg.bw);  % thermal noise dBm
cfg.snrThresh   = 5;            % dB

% ── AoI / vídeo ─────────────────────────────────────────────────────
cfg.L_c2        = 1;
cfg.L_vid       = 1;
cfg.fps         = 30;
cfg.frameInt    = 1 / cfg.fps;

fprintf('[CFG] Duração da sim: %.0f s  |  Velocidade: %.0f m/s  |  Travessia: %.0f s\n', ...
        cfg.simDuration, cfg.droneSpeed, cfg.flightTime);
end

%% ═══════════════════════════════════════════════════════════════════════
%%  CENÁRIO
%% ═══════════════════════════════════════════════════════════════════════
function [scene, drones, infra] = build_linear_scene(cfg)

N = cfg.numDrones;
rng(42);

scene = uavScenario( ...
    UpdateRate   = cfg.updateRate, ...
    StopTime     = cfg.simDuration, ...
    MaxNumFrames = 500);

% gNB
gnb = uavPlatform('gNB', scene, InitialPosition = cfg.gnbPos);
updateMesh(gnb, 'cuboid', {[8 8 30]}, [0 0.4 0.8], [0 0 15], [1 0 0 0]);

% Chegadas: drone k entra em t = (k-1)*entryInterval + jitter
startTimes = zeros(N, 1);
for k = 1:N
    jitter = (rand()-0.5) * 2 * cfg.jitter;
    startTimes(k) = (k-1) * cfg.entryInterval + jitter;
    startTimes(k) = max(startTimes(k), 0.5);
end
endTimes = startTimes + cfg.flightTime;

fprintf('[SCENE] Chegadas:\n');
for k = 1:N
    fprintf('  Drone %2d: entra t=%5.1f s | sai t=%5.1f s | faixa y=%+.0f m\n', ...
            k, startTimes(k), endTimes(k), (k-1)*cfg.yOffset);
end

% Trajetórias em linha reta — cada drone numa faixa lateral ligeiramente diferente
drones = cell(N, 1);
for k = 1:N
    yPos = (k - 1) * cfg.yOffset;   % faixas laterais: 0, 20, 40, ... m
    p1 = [cfg.xStart  yPos  cfg.droneAlt];
    p2 = [cfg.xEnd    yPos  cfg.droneAlt];
    traj = waypointTrajectory([p1; p2], ...
           TimeOfArrival = [startTimes(k); endTimes(k)]);
    drones{k} = uavPlatform('UAV' + string(k), scene, Trajectory = traj);
    updateMesh(drones{k}, 'quadrotor', {3}, [0.8 0.3 0.1], [0 0 0], [1 0 0 0]);
end
setup(scene);

infra.gnbPos    = cfg.gnbPos;
infra.startTimes = startTimes;
infra.endTimes   = endTimes;
end

%% ═══════════════════════════════════════════════════════════════════════
%%  ESTADO
%% ═══════════════════════════════════════════════════════════════════════
function state = init_linear_state(cfg, infra)
N = cfg.numDrones;

for u = N:-1:1
    state.aoi(u).h          = 0;
    state.aoi(u).h_vid      = 0;
    state.aoi(u).lastRx     = NaN;
    state.aoi(u).vid_rem    = 0;
    state.aoi(u).vid_gen    = 0;
    state.aoi(u).next_frame = infra.startTimes(u);
    state.aoi(u).nTx        = 0;
    state.aoi(u).nFail      = 0;
end

state.snr      = nan(N, 1);
state.rsrp     = nan(N, 1);   % dBm (estimado)
state.pl       = nan(N, 1);   % path loss dB
state.dist     = nan(N, 1);   % distância ao gNB (m)
state.cap      = nan(N, 1);   % capacidade Mbps
state.pos      = zeros(N, 3);
state.valid    = false(N, 1);
state.active   = [];
state.rrPtr    = 1;

state.startTimes = infra.startTimes;
state.endTimes   = infra.endTimes;

% Históricos por drone (indexados pela posição x para o gráfico espacial)
for u = 1:N
    state.hist(u).t    = [];
    state.hist(u).x    = [];    % posição x ao longo da rota
    state.hist(u).snr  = [];
    state.hist(u).rsrp = [];
    state.hist(u).pl   = [];
    state.hist(u).aoi  = [];
    state.hist(u).dist = [];
    state.hist(u).cap  = [];
end

% Série temporal agregada
state.tSeries      = [];
state.snrMean      = [];
state.aoiMean      = [];
state.aoiPeak      = [];
state.nActiveSeries = [];
end

%% ═══════════════════════════════════════════════════════════════════════
%%  LEITURA DE POSIÇÕES
%% ═══════════════════════════════════════════════════════════════════════
function state = lsim_read_positions(state, drones, time, cfg)
N = cfg.numDrones;
state.active = [];
for i = 1:N
    if time < state.startTimes(i) || time > state.endTimes(i)
        state.valid(i) = false;
        continue;
    end
    [posq, ~] = read(drones{i});
    if any(isnan(posq)), state.valid(i) = false; continue; end
    state.pos(i,:) = posq(1:3);
    state.valid(i) = true;
    state.active(end+1) = i; %#ok<AGROW>
end
end

%% ═══════════════════════════════════════════════════════════════════════
%%  ROUND-ROBIN
%% ═══════════════════════════════════════════════════════════════════════
function [txDrone, state] = lsim_rr_schedule(state, ~)
if isempty(state.active)
    txDrone = 0; return;
end
n   = numel(state.active);
ptr = mod(state.rrPtr - 1, n) + 1;
txDrone     = state.active(ptr);
state.rrPtr = mod(ptr, n) + 1;
end

%% ═══════════════════════════════════════════════════════════════════════
%%  CANAL — SNR, RSRP, PATH LOSS, CAPACIDADE
%% ═══════════════════════════════════════════════════════════════════════
function state = lsim_channel(state, infra, plCfg, cfg)
for i = state.active
    pos = state.pos(i,:);

    % Distância 3D ao gNB
    d3d = norm(pos - infra.gnbPos);
    state.dist(i) = d3d;

    % Path loss (UMi, LOS/NLOS automático pela toolbox)
    gnb_ned = [infra.gnbPos(2); infra.gnbPos(1); -infra.gnbPos(3)];
    ue_ned  = [pos(2); pos(1); -pos(3)];
    PL = nrPathLoss(plCfg, cfg.fc, true, gnb_ned, ue_ned);
    state.pl(i) = PL;

    % SNR
    snr_db = (cfg.pTx + cfg.gnbGain - PL) - (cfg.kT + cfg.noiseFig);
    state.snr(i) = snr_db;

    % RSRP estimada (potência recebida no UE, por subportadora de ref.)
    % RSRP ≈ pTx + gnbGain - PL - 10*log10(Nsc)   [Nsc ≈ bw/15e3]
    Nsc = cfg.bw / 15e3;
    state.rsrp(i) = cfg.pTx + cfg.gnbGain - PL - 10*log10(Nsc);

    % Capacidade Shannon
    state.cap(i) = cfg.bw * log2(1 + 10^(snr_db/10)) / 1e6;  % Mbps
end
end

%% ═══════════════════════════════════════════════════════════════════════
%%  TRANSMISSÃO + AoI
%% ═══════════════════════════════════════════════════════════════════════
function state = lsim_transmit_aoi(state, txDrone, time, cfg)
ms = 1000 / cfg.updateRate;

% ── 1. AGING PRIMEIRO (início do slot) ──────────────────────────────
for i = state.active
    state.aoi(i).h     = state.aoi(i).h     + 1;
    state.aoi(i).h_vid = state.aoi(i).h_vid + 1;
end

% ── 2. GERAÇÃO DE FRAMES ────────────────────────────────────────────
for i = state.active
    if time >= state.aoi(i).next_frame
        state.aoi(i).vid_rem    = cfg.L_vid;
        state.aoi(i).vid_gen    = time;
        state.aoi(i).next_frame = time + cfg.frameInt;
    end
end

% ── 3. TRANSMISSÃO → RESET (depois do aging) ────────────────────────
for i = state.active
    myTurn = (txDrone == i);
    snrOk  = (~isnan(state.snr(i))) && (state.snr(i) > cfg.snrThresh);

    if myTurn && snrOk
        state.aoi(i).h      = 0;   % << reseta APÓS aging, grava 0 no hist
        state.aoi(i).lastRx = time;
        state.aoi(i).nTx    = state.aoi(i).nTx + 1;
        if state.aoi(i).vid_rem > 0
            state.aoi(i).vid_rem = state.aoi(i).vid_rem - 1;
            if state.aoi(i).vid_rem == 0
                elapsed = time - state.aoi(i).vid_gen;
                state.aoi(i).h_vid = round(elapsed * cfg.updateRate);
            end
        end
    elseif myTurn
        state.aoi(i).nFail = state.aoi(i).nFail + 1;
    end

    % ── 4. SALVA HISTÓRICO (com o valor pós-reset) ──────────────────
    state.hist(i).t(end+1)    = time;
    state.hist(i).x(end+1)    = state.pos(i,1);
    state.hist(i).snr(end+1)  = state.snr(i);
    state.hist(i).rsrp(end+1) = state.rsrp(i);
    state.hist(i).pl(end+1)   = state.pl(i);
    state.hist(i).aoi(end+1)  = state.aoi(i).h * ms;  % 0 se TX ok
    state.hist(i).dist(end+1) = state.dist(i);
    state.hist(i).cap(end+1)  = state.cap(i);
end

% ── 5. SÉRIE AGREGADA ───────────────────────────────────────────────
if ~isempty(state.active)
    h_ms  = arrayfun(@(u) state.aoi(u).h * ms, state.active);
    snr_v = state.snr(state.active);
    snr_v = snr_v(~isnan(snr_v));
    state.tSeries(end+1)       = time;
    state.snrMean(end+1)       = mean(snr_v);
    state.aoiMean(end+1)       = mean(h_ms);
    state.aoiPeak(end+1)       = max(h_ms);
    state.nActiveSeries(end+1) = numel(state.active);
end
end

%% ═══════════════════════════════════════════════════════════════════════
%%  DASHBOARD EM TEMPO REAL
%% ═══════════════════════════════════════════════════════════════════════
function db = create_linear_dashboard(cfg)
N = cfg.numDrones;
C = lines(max(N,1));

db.fig = figure('Color',[0.07 0.08 0.10], ...
    'Position',[30 30 1400 860], ...
    'Name', sprintf('Linear Flight — %d drones — SNR & AoI', N), ...
    'NumberTitle','off');

tl = tiledlayout(db.fig, 3, 3, 'TileSpacing','compact', 'Padding','compact');
title(tl, sprintf('Voo em Linha Reta  |  gNB em x=0  |  Round-Robin  |  N=%d drones', N), ...
    'Color','w','FontSize',13,'FontWeight','bold');

axprop = {'Color',[0.10 0.11 0.14],'XColor','w','YColor','w', ...
          'GridColor',[0.3 0.3 0.3],'GridAlpha',0.5};

% ── Tile 1-2: Vista 2D da rota (top-view X×Y) ─────────────────────────
db.ax_map = nexttile(tl,[1 2]);
hold(db.ax_map,'on'); grid(db.ax_map,'on');
set(db.ax_map, axprop{:});
% gNB
plot(db.ax_map, 0, 0, '^', 'Color',[0.2 0.7 1], ...
     'MarkerSize',14,'MarkerFaceColor',[0.2 0.7 1]);
text(db.ax_map, 20, 20, 'gNB', 'Color',[0.2 0.7 1],'FontSize',10,'Color','w');
% Linha de rota
for k = 1:N
    yy = (k-1)*cfg.yOffset;
    plot(db.ax_map, [cfg.xStart cfg.xEnd],[yy yy],'--', ...
         'Color',[C(k,:) 0.25],'LineWidth',1);
end
% Raio de cobertura indicativo (UMi ~200 m típico para snr>5 dB)
th = linspace(0,2*pi,120);
plot(db.ax_map, 200*cos(th), 200*sin(th), ':', ...
     'Color',[1 1 0 0.4],'LineWidth',1);
text(db.ax_map,-180,215,'~cobertura','Color',[1 1 0 0.6],'FontSize',8);
db.h_drones = gobjects(N,1);
db.h_snrPatch = gobjects(N,1);
for k = 1:N
    db.h_snrPatch(k) = scatter(db.ax_map, NaN, NaN, 80, NaN, 'filled', ...
        'MarkerEdgeColor','none');
    db.h_drones(k) = plot(db.ax_map, NaN, NaN, 'o', ...
        'Color',C(k,:),'MarkerSize',12,'MarkerFaceColor',C(k,:));
end
cm_snr = parula(256);
colormap(db.ax_map, cm_snr);
cb = colorbar(db.ax_map); cb.Color = 'w';
cb.Label.String = 'SNR (dB)'; cb.Label.Color = 'w';
clim(db.ax_map, [-10 40]);
xlabel(db.ax_map,'X (m)','Color','w');
ylabel(db.ax_map,'Y (m)','Color','w');
title(db.ax_map,'Vista aérea — cor = SNR','Color','w');
xlim(db.ax_map,[cfg.xStart-100, cfg.xEnd+100]);
ylim(db.ax_map,[-150 max(150,(N-1)*cfg.yOffset+150)]);

% ── Tile 3: painel de texto ───────────────────────────────────────────
db.ax_txt = nexttile(tl);
set(db.ax_txt,'Color',[0.10 0.11 0.14],'Visible','off');
db.h_txt = text(db.ax_txt,0.05,0.5,'...','Color','w','FontSize',10, ...
    'FontName','Courier New','Units','normalized','VerticalAlignment','middle');
title(db.ax_txt,'Status','Color','w');
axis(db.ax_txt,[0 1 0 1]);

% ── Tile 4-5: SNR vs posição X ────────────────────────────────────────
db.ax_snrX = nexttile(tl,[1 2]);
hold(db.ax_snrX,'on'); grid(db.ax_snrX,'on');
set(db.ax_snrX, axprop{:});
xline(db.ax_snrX, 0,'--','Color',[0.2 0.7 1],'LineWidth',1.5,'Label','gNB','LabelColor','w');
yline(db.ax_snrX, cfg.snrThresh,'r--','Threshold','LabelHorizontalAlignment','left','LabelColor','w');
db.h_snrX = gobjects(N,1);
for k = 1:N
    db.h_snrX(k) = plot(db.ax_snrX, NaN, NaN, '-', ...
        'Color',C(k,:),'LineWidth',1.8, ...
        'DisplayName',sprintf('Drone %d',k));
end
legend(db.ax_snrX,'TextColor','w','Color',[0.10 0.11 0.14],'Location','southeast','FontSize',8);
xlabel(db.ax_snrX,'Posição X (m)','Color','w');
ylabel(db.ax_snrX,'SNR (dB)','Color','w');
title(db.ax_snrX,'SNR vs Posição na Rota','Color','w');
xlim(db.ax_snrX,[cfg.xStart cfg.xEnd]);

% ── Tile 6: Path Loss vs X ────────────────────────────────────────────
db.ax_plX = nexttile(tl);
hold(db.ax_plX,'on'); grid(db.ax_plX,'on');
set(db.ax_plX, axprop{:});
xline(db.ax_plX, 0,'--','Color',[0.2 0.7 1],'LineWidth',1.5);
db.h_plX = gobjects(N,1);
for k = 1:N
    db.h_plX(k) = plot(db.ax_plX, NaN, NaN, '-', 'Color',C(k,:),'LineWidth',1.5);
end
xlabel(db.ax_plX,'Posição X (m)','Color','w');
ylabel(db.ax_plX,'Path Loss (dB)','Color','w');
title(db.ax_plX,'Path Loss (UMi) vs X','Color','w');
xlim(db.ax_plX,[cfg.xStart cfg.xEnd]);

% ── Tile 7-8: AoI vs TEMPO ───────────────────────────────────────────
db.ax_aoiT = nexttile(tl,[1 2]);
hold(db.ax_aoiT,'on'); grid(db.ax_aoiT,'on');
set(db.ax_aoiT, axprop{:});
db.h_aoiT = gobjects(N,1);
for k = 1:N
    db.h_aoiT(k) = plot(db.ax_aoiT, NaN, NaN, '-', ...
        'Color',C(k,:),'LineWidth',1.8, ...
        'DisplayName',sprintf('Drone %d',k));
end
db.h_aoiMean = plot(db.ax_aoiT, NaN, NaN, 'w--', 'LineWidth',2.2, ...
    'DisplayName','Média');
legend(db.ax_aoiT,'TextColor','w','Color',[0.10 0.11 0.14],'Location','northeast','FontSize',8);
xlabel(db.ax_aoiT,'Tempo (s)','Color','w');
ylabel(db.ax_aoiT,'AoI C2 (ms)','Color','w');
title(db.ax_aoiT,'AoI C2 vs Tempo — por drone','Color','w');

% ── Tile 9: Capacidade vs X ───────────────────────────────────────────
db.ax_capX = nexttile(tl);
hold(db.ax_capX,'on'); grid(db.ax_capX,'on');
set(db.ax_capX, axprop{:});
xline(db.ax_capX, 0,'--','Color',[0.2 0.7 1],'LineWidth',1.5);
db.h_capX = gobjects(N,1);
for k = 1:N
    db.h_capX(k) = plot(db.ax_capX, NaN, NaN, '-', 'Color',C(k,:),'LineWidth',1.5);
end
xlabel(db.ax_capX,'Posição X (m)','Color','w');
ylabel(db.ax_capX,'Cap. Shannon (Mbps)','Color','w');
title(db.ax_capX,'Capacidade vs X','Color','w');
xlim(db.ax_capX,[cfg.xStart cfg.xEnd]);

db.N = N;
db.C = C;
db.cfg = cfg;
end

% ─────────────────────────────────────────────────────────────────────────
function lsim_update_dashboard(db, state, time, cfg)
N = db.N;
ms = 1000 / cfg.updateRate;

for k = 1:N
    if ~state.valid(k), continue; end
    x = state.pos(k,1);
    y = state.pos(k,2);

    % Mapa: posição + cor SNR
    set(db.h_drones(k),   'XData', x, 'YData', y);
    % trilha colorida por SNR
    if ~isempty(state.hist(k).x)
        set(db.h_snrPatch(k), ...
            'XData', state.hist(k).x, ...
            'YData', repmat((k-1)*cfg.yOffset, size(state.hist(k).x)), ...
            'CData', state.hist(k).snr);
    end

    % SNR vs X, path loss vs X, capacidade vs X
    if ~isempty(state.hist(k).x)
        set(db.h_snrX(k), 'XData', state.hist(k).x, 'YData', state.hist(k).snr);
        set(db.h_plX(k),  'XData', state.hist(k).x, 'YData', state.hist(k).pl);
        set(db.h_capX(k), 'XData', state.hist(k).x, 'YData', state.hist(k).cap);
    end
    % AoI vs Tempo
    if ~isempty(state.hist(k).t)
        set(db.h_aoiT(k), 'XData', state.hist(k).t, 'YData', state.hist(k).aoi);
    end
end

% Atualiza média de AoI
if ~isempty(state.tSeries)
    set(db.h_aoiMean, 'XData', state.tSeries, 'YData', state.aoiMean);
end

% Texto de status
active = state.active;
if ~isempty(active)
    snr_v = state.snr(active);
    snr_v = snr_v(~isnan(snr_v));
    h_ms  = arrayfun(@(u) state.aoi(u).h * ms, active);
    txt = sprintf( ...
        ['t = %.1f s\n' ...
         'Ativos : %d / %d\n' ...
         'SNR min: %5.1f dB\n' ...
         'SNR med: %5.1f dB\n' ...
         'SNR max: %5.1f dB\n' ...
         'AoI med: %5.0f ms\n' ...
         'AoI pik: %5.0f ms\n' ...
         'RR ptr : %d'], ...
        time, numel(active), N, ...
        min(snr_v), mean(snr_v), max(snr_v), ...
        mean(h_ms), max(h_ms), state.rrPtr);
else
    txt = sprintf('t = %.1f s\nAtivos: 0/%d\nAguardando...', time, N);
end
set(db.h_txt,'String', txt);

drawnow limitrate;
end

%% ═══════════════════════════════════════════════════════════════════════
%%  COLETA E SUMÁRIO
%% ═══════════════════════════════════════════════════════════════════════
function results = lsim_collect(state, cfg)
N = cfg.numDrones;
ms = 1000 / cfg.updateRate;

all_snr = []; all_aoi = []; all_rsrp = []; all_pl = []; all_cap = [];
for k = 1:N
    all_snr  = [all_snr,  state.hist(k).snr];  %#ok<AGROW>
    all_aoi  = [all_aoi,  state.hist(k).aoi];  %#ok<AGROW>
    all_rsrp = [all_rsrp, state.hist(k).rsrp]; %#ok<AGROW>
    all_pl   = [all_pl,   state.hist(k).pl];   %#ok<AGROW>
    all_cap  = [all_cap,  state.hist(k).cap];  %#ok<AGROW>
end

results.histPerDrone = state.hist;
results.tSeries      = state.tSeries;
results.snrMean      = state.snrMean;
results.aoiMean      = state.aoiMean;
results.aoiPeak      = state.aoiPeak;
results.nActive      = state.nActiveSeries;

results.snr_min  = min(all_snr(~isnan(all_snr)));
results.snr_max  = max(all_snr(~isnan(all_snr)));
results.snr_mean = mean(all_snr(~isnan(all_snr)));
results.rsrp_min = min(all_rsrp(~isnan(all_rsrp)));
results.rsrp_max = max(all_rsrp(~isnan(all_rsrp)));
results.pl_max   = max(all_pl(~isnan(all_pl)));
results.pl_min   = min(all_pl(~isnan(all_pl)));

results.meanAoI_ms = mean(all_aoi(all_aoi > 0));
results.peakAoI_ms = max(all_aoi);

tot_tx   = sum(arrayfun(@(u) state.aoi(u).nTx,   1:N));
tot_fail = sum(arrayfun(@(u) state.aoi(u).nFail, 1:N));
results.deliveryRate = tot_tx / max(tot_tx + tot_fail, 1);
results.numDrones = N;
end

% ─────────────────────────────────────────────────────────────────────────
