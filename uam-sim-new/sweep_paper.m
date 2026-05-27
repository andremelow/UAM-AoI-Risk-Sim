function sweep_paper(varargin)
%% SWEEP_PAPER  Disparo completo dos sweeps do paper (3 dimensões).
%
%  Roda em sequência:
%    1. Smoke test           (~5 min)   — verifica se o pipeline está sadio
%    2. Sweep numDrones      (~10h)     — escalabilidade
%    3. Sweep p_c2           (~12h)     — robustez à confiabilidade do canal
%    4. Sweep L_vid          (~10h)     — sensibilidade ao tamanho de frame
%
%  Total estimado: ~32h wall-clock sequencial.
%
%  Cada sweep gera:
%    csv_export/paper/sweep_<param>/sweep_<param>_summary.csv
%    csv_export/paper/sweep_<param>/<policy>_<...>_slot_series.csv
%
%  Uso:
%    sweep_paper()            % roda tudo (com confirmação)
%    sweep_paper('skip-smoke') % pula smoke test (já validado)
%    sweep_paper('--yes')      % não pede confirmação (background)
%    sweep_paper('only', 'N')  % só o sweep de numDrones
%    sweep_paper('only', 'p')  % só o sweep de p_c2
%    sweep_paper('only', 'L')  % só o sweep de L_vid

setup_paths();

% --- Argument parsing ---
skip_smoke   = false;
auto_confirm = false;
only_dim     = '';
ai = 1;
while ai <= numel(varargin)
    a = varargin{ai};
    switch a
        case 'skip-smoke'
            skip_smoke = true;
        case '--yes'
            auto_confirm = true;
        case 'only'
            only_dim = varargin{ai+1};
            ai = ai + 1;
    end
    ai = ai + 1;
end

% --- Output root ---
OUT_ROOT = fullfile('csv_export', 'paper');
if ~exist(OUT_ROOT, 'dir'), mkdir(OUT_ROOT); end

% --- Logfile ---
ts = datestr(now, 'yyyymmdd_HHMMSS'); %#ok<TNOW1,DATST>
logFile = fullfile(OUT_ROOT, sprintf('sweep_paper_%s.log', ts));
diary(logFile); diary on;
fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  SWEEP_PAPER — full doc-aligned simulation campaign\n');
fprintf('  Started: %s\n', datestr(now)); %#ok<TNOW1,DATST>
fprintf('  Log: %s\n', logFile);
fprintf('═══════════════════════════════════════════════════════════════\n\n');

% --- Confirmation ---
if ~auto_confirm
    fprintf('This will run 3 sweeps sequentially:\n');
    fprintf('  1. Smoke test          — ~5 min  (skip with ''skip-smoke'')\n');
    fprintf('  2. Sweep numDrones     — ~10h    (5 values × 5 policies)\n');
    fprintf('  3. Sweep p_c2          — ~12h    (6 values × 5 policies)\n');
    fprintf('  4. Sweep L_vid         — ~10h    (5 values × 5 policies)\n');
    fprintf('  Total: ~32h wall-clock.\n\n');
    fprintf('Press ENTER to continue, or Ctrl-C to abort.\n');
    pause;
end

% --- Common base config (paper-grade) ---
cfg_base = base_config_paper();

% =====================================================================
% SMOKE TEST (5 min) — verifies pipeline before committing to 32h
% =====================================================================
if ~skip_smoke
    fprintf('\n──────────────────────────────────────────────────────────\n');
    fprintf('  STEP 1/4: SMOKE TEST\n');
    fprintf('──────────────────────────────────────────────────────────\n\n');
    smoke_t0 = tic;
    smoke_ok = run_smoke(OUT_ROOT);
    smoke_dt = toc(smoke_t0);
    fprintf('\n[SMOKE] Completed in %.1f min, status: %s\n', ...
        smoke_dt/60, ternary(smoke_ok, 'PASS', 'FAIL'));
    if ~smoke_ok
        fprintf('[SMOKE] FAILED — aborting paper sweep. Inspect output above.\n');
        diary off;
        return;
    end
end

% =====================================================================
% SWEEP 1: numDrones (scalability)
% =====================================================================
if isempty(only_dim) || strcmp(only_dim, 'N')
    fprintf('\n──────────────────────────────────────────────────────────\n');
    fprintf('  STEP 2/4: SWEEP numDrones\n');
    fprintf('──────────────────────────────────────────────────────────\n\n');
    sw_t0 = tic;
    sc1.policies     = {'round-robin','round-robin-aware','aoi-pure','risk-aware','max-weight'};
    sc1.param_name   = 'numDrones';
    sc1.param_values = [1 5 10 15 20];
    sc1.base_cfg     = cfg_base;
    run_sweep(sc1, fullfile(OUT_ROOT, 'sweep_N'));
    fprintf('[SWEEP-N] Completed in %.2f h\n', toc(sw_t0)/3600);
end

% =====================================================================
% SWEEP 2: p_c2 (channel reliability)
% =====================================================================
if isempty(only_dim) || strcmp(only_dim, 'p')
    fprintf('\n──────────────────────────────────────────────────────────\n');
    fprintf('  STEP 3/4: SWEEP p_c2\n');
    fprintf('──────────────────────────────────────────────────────────\n\n');
    sw_t0 = tic;
    sc2.policies     = {'round-robin','round-robin-aware','aoi-pure','risk-aware','max-weight'};
    sc2.param_name   = 'p_c2';
    sc2.param_values = [0.5 0.6 0.7 0.8 0.9 0.95];
    sc2.base_cfg     = cfg_base;
    sc2.base_cfg.numDrones = 10;          % fixar N=10 (mediano da grade anterior)
    run_sweep(sc2, fullfile(OUT_ROOT, 'sweep_p'));
    fprintf('[SWEEP-p] Completed in %.2f h\n', toc(sw_t0)/3600);
end

% =====================================================================
% SWEEP 3: L_vid (video frame size)
% =====================================================================
if isempty(only_dim) || strcmp(only_dim, 'L')
    fprintf('\n──────────────────────────────────────────────────────────\n');
    fprintf('  STEP 4/4: SWEEP L_vid\n');
    fprintf('──────────────────────────────────────────────────────────\n\n');
    sw_t0 = tic;
    sc3.policies     = {'round-robin','round-robin-aware','aoi-pure','risk-aware','max-weight'};
    sc3.param_name   = 'L_vid';
    sc3.param_values = [1 5 10 20 30];
    sc3.base_cfg     = cfg_base;
    sc3.base_cfg.numDrones = 10;
    run_sweep(sc3, fullfile(OUT_ROOT, 'sweep_L'));
    fprintf('[SWEEP-L] Completed in %.2f h\n', toc(sw_t0)/3600);
end

% --- Final consolidation ---
fprintf('\n═══════════════════════════════════════════════════════════════\n');
fprintf('  SWEEP_PAPER COMPLETED\n');
fprintf('  Finished: %s\n', datestr(now)); %#ok<TNOW1,DATST>
fprintf('  Output dir: %s\n', OUT_ROOT);
fprintf('  Summary CSVs:\n');
fprintf('    %s/sweep_N/sweep_numDrones_summary.csv\n', OUT_ROOT);
fprintf('    %s/sweep_p/sweep_p_c2_summary.csv\n',       OUT_ROOT);
fprintf('    %s/sweep_L/sweep_L_vid_summary.csv\n',      OUT_ROOT);
fprintf('  Log: %s\n', logFile);
fprintf('═══════════════════════════════════════════════════════════════\n\n');
diary off;
end


%% ═══════════════════════════════════════════════════════════════════
%%  Smoke test — small reduced config, must complete in ~5 min
%% ═══════════════════════════════════════════════════════════════════
function ok = run_smoke(outRoot)
ok = true;
sc.policies     = {'round-robin', 'max-weight'};
sc.param_name   = 'numDrones';
sc.param_values = [3];
sc.base_cfg     = base_config_smoke();

try
    res = run_sweep(sc, fullfile(outRoot, 'smoke'));
    % Quick sanity checks on results
    for k = 1:numel(res)
        if isfield(res(k), 'error') && ~isempty(res(k).error)
            fprintf('[SMOKE] Run failed: policy=%s, error=%s\n', ...
                    res(k).policy, res(k).error);
            ok = false;
        elseif isfield(res(k), 'rho_emp') && ~isempty(res(k).rho_emp)
            if isnan(res(k).rho_emp) || res(k).rho_emp < 0.99
                fprintf('[SMOKE] Suspicious rho_emp=%.3f for policy=%s (expected >= 1)\n', ...
                        res(k).rho_emp, res(k).policy);
                ok = false;
            else
                fprintf('[SMOKE] %s: rho_emp=%.3f (OK)\n', ...
                        res(k).policy, res(k).rho_emp);
            end
        end
    end
catch ME
    fprintf('[SMOKE] EXCEPTION: %s\n', ME.message);
    fprintf('[SMOKE] Stack: %s:%d\n', ME.stack(1).name, ME.stack(1).line);
    ok = false;
end
end


%% ═══════════════════════════════════════════════════════════════════
%%  Smoke test config — herda paper config mas reduz tudo proporcionalmente
%%
%%  Mantém a geometria A2 (BSs alinhadas ao corredor, dois buracos de
%%  cobertura) mas em escala 1:10 para um run rápido.
%% ═══════════════════════════════════════════════════════════════════
function cfg = base_config_smoke()
cfg = base_config_paper();

scale = 0.1;                              % 1:10
cfg.corridorLength = cfg.corridorLength * scale;     % 30000 → 3000
cfg.flightTime     = cfg.corridorLength / cfg.speedVal;
cfg.maxStartDelay  = 50;
cfg.numDrones      = 3;

% Reescalar BSs proporcionalmente para preservar a geometria A2
cfg.microBSPos(:, 1:2) = cfg.microBSPos(:, 1:2) * scale;
end


%% ═══════════════════════════════════════════════════════════════════
%%  Paper config — herda de build_scenario_config (geometria validada)
%%
%%  IMPORTANTE: tudo que diz respeito à GEOMETRIA do cenário
%%  (microBSPos, droneEastPos, corridorLength, flightTime, BSs, etc.)
%%  vem DIRETAMENTE de build_scenario_config, que é a única fonte de
%%  verdade para o cenário A2. Aqui só adicionamos os parâmetros
%%  doc-aligned (V_s, V_v, kappa, use_normalization) que NÃO existem
%%  no build_scenario_config.
%% ═══════════════════════════════════════════════════════════════════
function cfg = base_config_paper()

cfg = build_scenario_config();         % HERDA cenário A2 completo

%% Apenas parâmetros doc-aligned (Phase 1-4) — não tocar geometria
cfg.csvExport         = true;
cfg.use_normalization = false;
cfg.V_s    = 1.0;
cfg.V_v    = 1.0;
cfg.kappa  = 0.0;

end


%% ═══════════════════════════════════════════════════════════════════
%%  Helper
%% ═══════════════════════════════════════════════════════════════════
function r = ternary(cond, a, b)
if cond, r = a; else, r = b; end
end
