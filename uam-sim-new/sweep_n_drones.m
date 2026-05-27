%% SWEEP_N_DRONES  Sweep numDrones ∈ {1,5,10,15,20} × 5 políticas.
%
%  Baseado no cenário de 30km com 3 BSs equidistantes.
%  Cada run usa o tempo natural do corredor (flightTime = corridorLength/speedVal).
%
%  Políticas avaliadas:
%    round-robin           — baseline pessimista (cego ao SNR)
%    round-robin-aware     — baseline forte (pula drones em zona morta)
%    aoi-pure              — Max-Weight só com AoI
%    risk-aware            — heurística R_u × AoI (legado)
%    max-weight            — política do doc Sec. 4.5 (contribuição teórica)
%
%  Saída: csv_export/sweep_N/
%    sweep_numDrones_summary.csv         → para o dashboard, com Jhat_emp,
%                                           J_LB_doc, rho_emp, debt queues
%    <policy>_N<N>_<ts>_slot_series.csv  → série temporal densa
%    <policy>_N<N>_<ts>_drone_summary.csv
%    <policy>_N<N>_<ts>_manifest.csv
%
%  Uso:
%    sweep_n_drones()                    % roda tudo
%    sweep_n_drones('aoi-pure')          % roda só uma política (debug)
%
%  Estimativa de tempo (corridorLength=30000, speedVal=30, maxStartDelay=450):
%    flightTime = 1000s, last_land ≈ 1450s = 21750 slots/run
%    25 runs (5N × 5pol) × 21750 slots / 15 slots_s = ~10h wall-clock
%    Reduzir corridorLength ou speedVal para testes rápidos.

function sweep_n_drones(varargin)

setup_paths();

%% ── Parâmetros do sweep ──────────────────────────────────────────────────────
N_VALUES  = [1, 5, 10, 15, 20];
POLICIES  = {'round-robin', 'round-robin-aware', ...
             'aoi-pure', 'risk-aware', 'max-weight'};
OUT_DIR   = fullfile('csv_export', 'sweep_N_5pol');

% Filtra política se passado como argumento (útil para debug/paralelo)
if nargin >= 1
    POLICIES = varargin(1);
    fprintf('[SWEEP] Single-policy mode: %s\n', POLICIES{1});
end

%% ── Config base (cenário de 30km + 3 BSs) ───────────────────────────────────
cfg_base = base_config_30km();

%% ── Monta sweep_cfg e executa ───────────────────────────────────────────────
sc.policies     = POLICIES;
sc.param_name   = 'numDrones';
sc.param_values = N_VALUES;
sc.base_cfg     = cfg_base;
% T_slots não é necessário — flightTime vem de corridorLength/speedVal

sweep_results = run_sweep(sc, OUT_DIR);

%% ── Resumo no terminal ──────────────────────────────────────────────────────
fprintf('\n══════════════════════════════════════════\n');
fprintf('  SWEEP CONCLUÍDO\n');
fprintf('  Resultados em: %s\n', OUT_DIR);
fprintf('  Para visualizar: abrir uam_dashboard.Rmd no RStudio\n');
fprintf('  CSV mestre: sweep_numDrones_summary.csv\n');
fprintf('  Colunas-chave do paper: Jhat_emp, J_LB_doc, rho_emp\n');
fprintf('══════════════════════════════════════════\n\n');

end  % sweep_n_drones

%% ═══════════════════════════════════════════════════════════════════════════
%%  Config base — herda de build_scenario_config (geometria A2 validada)
%%
%%  Igual ao sweep_paper.base_config_paper(): só adiciona os parâmetros
%%  doc-aligned (Phase 1-4) sem tocar a geometria do cenário, que é a
%%  única fonte de verdade para o A2.
%% ═══════════════════════════════════════════════════════════════════════════
function cfg = base_config_30km()

cfg = build_scenario_config();         % HERDA cenário A2 completo

%% Apenas parâmetros doc-aligned (Phase 1-4)
cfg.csvExport         = true;
cfg.csvDir            = fullfile('csv_export', 'sweep_N_5pol');
cfg.use_normalization = false;
cfg.V_s    = 1.0;
cfg.V_v    = 1.0;
cfg.kappa  = 0.0;
% omega: deixar ausente para que validate_uam_config use default uniforme

end
