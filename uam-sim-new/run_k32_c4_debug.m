%% RUN_K32_C4_DEBUG
%  Roda K=32, C=4, 2r_static com dashboard gráfico aberto.
%  Mude 'policy' abaixo para comparar as políticas.
%
%  Usage (MATLAB Command Window):
%    run_k32_c4_debug
%
%  Políticas disponíveis:
%    'round-robin'      'round-robin-sw'
%    'pf-classic'       'pf-classic-sw'
%    'max-weight-drift' 'max-weight-sw'

%% ── Parâmetros que você pode mudar ───────────────────────────────────
policy    = 'max-weight-drift';   % política a testar
K         = 32;                   % slots por época
C         = 4;                    % drones simultâneos no corredor
T_steady  = 900;                  % duração regime estável (s)

%% ── Setup ────────────────────────────────────────────────────────────
setup_paths();

%% ── Base config (geometria assimétrica) ──────────────────────────────
cfg = build_scenario_assimetrico();

% Pesos iguais
cfg.w_unc = 1/3;
cfg.w_map = 1/3;
cfg.w_vid = 1/3;

% Modelo de chegada
cfg.arrivalModel    = 'capacity_drain';
cfg.minStartDelay   = 2;
cfg.steady_duration = T_steady;

% Vídeo
cfg.videoModel = 'atomic';
cfg.fps        = 30;
cfg.L_vid      = 100;
cfg.L_c2       = 1;
cfg.w_coh      = 5;
cfg.L_nom      = 0.01;
cfg.p_c2       = 0.80;
cfg.p_vid      = 0.80;
cfg.rho_0      = 0.01;

%% ── Rotas do experimento assimétrico 3 km ────────────────────────────
r1.numDrones = 1;   % será recalculado abaixo
r1.start     = [200,  -1500];   % [North, East]
r1.goal      = [500,   1500];
r1.label     = 'Route2';

r2.numDrones = 1;
r2.start     = [-500,  1500];
r2.goal      = [-600, -1500];
r2.label     = 'Route3';

routes   = [r1, r2];
nR       = numel(routes);

%% ── Número de drones por rota (lógica de build_capacity_run_cfg) ─────
flightTime = norm(routes(1).goal - routes(1).start) / cfg.speedVal;
C_per      = floor(C / nR);
C_rem      = C - C_per * nR;
for ri = 1:nR
    C_r               = C_per + (ri <= C_rem);
    spacing           = flightTime / max(C_r, 1);
    routes(ri).numDrones = C_r + floor(T_steady / spacing);
end
N_total = sum([routes.numDrones]);

%% ── Finaliza cfg ─────────────────────────────────────────────────────
cfg.routes           = routes;
cfg.corridorLength   = norm(routes(1).goal - routes(1).start);
cfg.flightTime       = flightTime;
cfg.corridorCapacity = C;
cfg.dronesPerSlot    = K;
cfg.routing.mode     = 'static';

spacing_min       = flightTime / max(C, 1);
cfg.maxStartDelay = cfg.minStartDelay + (N_total - 1) * spacing_min + 10;

% Deixa validate_uam_config calcular H_max, omega, beta_s, beta_v
cfg.H_max  = [];
cfg.omega  = [];
cfg.beta_s = [];
cfg.beta_v = [];

cfg.schedulingPolicy = policy;
cfg.headless         = false;   % abre o dashboard gráfico
cfg.csvExport        = false;   % não grava CSV (modo debug)

%% ── Info ─────────────────────────────────────────────────────────────
fprintf('\n');
fprintf('══════════════════════════════════════════\n');
fprintf('  DEBUG RUN — K=%d  C=%d  política: %s\n', K, C, policy);
fprintf('  N total drones = %d  (flightTime=%.0f s)\n', N_total, flightTime);
fprintf('  T_steady = %d s\n', T_steady);
fprintf('══════════════════════════════════════════\n\n');

%% ── Roda simulação ───────────────────────────────────────────────────
results = run_sim(cfg);

%% ── Resumo pós-simulação ─────────────────────────────────────────────
fprintf('\n══════════════════════════════════════════\n');
fprintf('  RESULTADO\n');
fprintf('  qbar_s mean  = %.4f  (máx físico = %.4f)\n', ...
        mean(results.cfg.qbar_s), min(K/N_total*2, 1) * cfg.p_c2);
fprintf('  x_s final    = %.2f\n', mean(arrayfun(@(u) ...
        ifempty_v(results.xsVal{u}, 0), 1:results.numDrones)));
fprintf('  Risco médio  = %.4g\n', results.Rsys_emp);
fprintf('  Jhat_emp     = %.4g\n', results.Jhat_emp);
fprintf('══════════════════════════════════════════\n\n');

function v = ifempty_v(x, d)
if isempty(x), v = d; else, v = x(end); end
end
