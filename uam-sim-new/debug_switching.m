%% DEBUG_SWITCHING  Teste visual para políticas switching vs no-switching.
%
%  Cenário mínimo: 3 drones, 1 rota curta, K=2, L_vid=8, sucesso determinístico.
%  Cada par (no-switching / switching) roda com dashboard visível e imprime
%  no console um trace por slot com o estado de vid_remaining e preempções.
%
%  Uso:
%    debug_switching               % compara pf-classic vs pf-classic-sw
%    debug_switching('rr')         % round-robin
%    debug_switching('pf')         % proportional fair (padrão)
%    debug_switching('mw')         % max-weight
%    debug_switching('all')        % todos os três pares (headless)

function debug_switching(which_policy)

setup_paths();

if nargin < 1, which_policy = 'pf'; end

switch lower(which_policy)
    case 'rr',  pairs = {{'round-robin',  'round-robin-sw'}};
    case 'pf',  pairs = {{'pf-classic',   'pf-classic-sw'}};
    case 'mw',  pairs = {{'max-weight',   'max-weight-sw'}};
    case 'all'
        pairs = {{'round-robin',  'round-robin-sw'}, ...
                 {'pf-classic',   'pf-classic-sw'}, ...
                 {'max-weight',   'max-weight-sw'}};
    otherwise
        error('debug_switching: opção inválida. Use rr | pf | mw | all.');
end

% -------------------------------------------------------------------------
%  Configuração mínima para debug
% -------------------------------------------------------------------------
cfg = build_scenario_config();

cfg.routes(1).numDrones = 3;
cfg.routes(1).start     = [0  -200];
cfg.routes(1).goal      = [0   200];
cfg.routes = cfg.routes(1);            % apenas 1 rota

cfg.corridorLength  = norm(cfg.routes(1).goal - cfg.routes(1).start);
cfg.flightTime      = 80;              % 80 s de voo
cfg.speedVal        = cfg.corridorLength / cfg.flightTime;
cfg.updateRate      = 4;               % 4 slots/s → slots mais lentos, fácil de ler
cfg.dronesPerSlot   = 2;               % K=2

cfg.L_vid           = 8;              % 8 pacotes/frame → ~8 slots para transmitir
cfg.fps             = 0.5;            % 1 frame a cada 2 s = 8 slots entre frames
cfg.p_c2            = 1.0;            % sucesso determinístico
cfg.p_vid           = 1.0;            % para isolar lógica de preempção
cfg.thresholdSNR    = -30;            % desativa gatilho SNR (sempre OK)

cfg.routing.mode    = 'static';
cfg.manualHotspots  = [];
cfg.numHotspots     = 0;
cfg.minStartDelay   = 2;
cfg.maxStartDelay   = 20;

% -------------------------------------------------------------------------
%  Roda cada par e coleta resultados
% -------------------------------------------------------------------------
all_results = {};

for pi = 1:numel(pairs)
    pair = pairs{pi};
    for vi = 1:2
        policy = pair{vi};
        fprintf('\n');
        fprintf('╔══════════════════════════════════════════════════════╗\n');
        fprintf('║  Política: %-42s║\n', policy);
        fprintf('╚══════════════════════════════════════════════════════╝\n');

        c = cfg;
        c.schedulingPolicy = policy;
        c.headless         = strcmp(which_policy, 'all');  % visual apenas no modo single

        r = run_debug_loop(c);
        r.policy = policy;
        all_results{end+1} = r; %#ok<AGROW>
    end
end

% -------------------------------------------------------------------------
%  Tabela comparativa final
% -------------------------------------------------------------------------
print_comparison(all_results);
end


%% ========================================================================
%%  Loop de simulação com trace detalhado por slot
%% ========================================================================
function results = run_debug_loop(cfg)

cfg = validate_uam_config(cfg);

[scene, drones, infra] = init_scenario(cfg);
state                  = init_state(cfg, infra);
[rhoMap, mapGrid]      = init_ground_risk(cfg);
infra.precomputed_trajectory = {};

if ~cfg.headless
    dashboard = init_dashboard(cfg, rhoMap, mapGrid);
end

N             = cfg.numDrones;
txSlots       = [];
preempt_count = zeros(N, 1);   % preempções por drone
resume_count  = zeros(N, 1);   % retomadas de frame parcial

fprintf('\n  Slot  | Drones ativos | Escalado (src)  | vid_rem antes→depois | Evento\n');
fprintf('  ------+---------------+-----------------+----------------------+---------\n');

while advance(scene)
    time = scene.CurrentTime;
    slot = round(time * cfg.updateRate);

    state = step_read_positions(state, drones, infra, time, slot, cfg);
    if slot > max(state.endSlots) && isempty(state.activeDrones), break; end

    state = step_traffic(state, slot, cfg);

    % Captura vid_remaining ANTES do schedule (para detectar preempção)
    vrem_before = arrayfun(@(u) state.dual(u).vid_remaining, 1:N);
    vif_before  = arrayfun(@(u) state.dual(u).vid_in_flight,  1:N);

    [txSlots, state] = schedule_sources(state, slot, cfg);

    state = step_channel(state, infra, time, cfg);
    state = step_transmit(state, txSlots, slot, cfg);

    % Captura vid_remaining DEPOIS
    vrem_after = arrayfun(@(u) state.dual(u).vid_remaining, 1:N);

    % --- Trace por slot (apenas quando há drones ativos) ---
    if ~isempty(state.activeDrones)
        active_str = sprintf('%d ', state.activeDrones);
        sched_str  = format_scheduled(txSlots, N);

        for u = state.activeDrones
            src_vid = 2*(u-1) + 2;
            vid_scheduled = any(txSlots == src_vid);
            was_inflight  = vif_before(u);

            if was_inflight && ~vid_scheduled
                % Frame em progresso e NÃO foi continuado → preempção
                preempt_count(u) = preempt_count(u) + 1;
                evt = sprintf('[PREEMPT drone %d]', u);
            elseif vrem_before(u) > 0 && vid_scheduled && ~vif_before(u)
                % Frame parcial retomado após preempção prévia
                resume_count(u) = resume_count(u) + 1;
                evt = sprintf('[RESUME  drone %d]', u);
            elseif vrem_after(u) == 0 && vrem_before(u) > 0 && vid_scheduled
                evt = sprintf('[COMPLETE drone %d]', u);
            else
                evt = '';
            end

            if ~isempty(evt)
                fprintf('  %5d | %-13s | %-15s | d%d: %2d → %2d              | %s\n', ...
                        slot, strtrim(active_str), sched_str, ...
                        u, vrem_before(u), vrem_after(u), evt);
            end
        end
    end

    % Logging obrigatório para collect_results
    for i = state.activeDrones
        src_c2  = 2*(i-1)+1;  src_vid = 2*(i-1)+2;
        if     any(txSlots == src_c2),  sched_val = 1;
        elseif any(txSlots == src_vid), sched_val = 2;
        else,                           sched_val = 0;
        end
        state.schedSeries{i}(end+1) = sched_val;
        state.schedSlots{i}(end+1)  = slot;
        snr_ok = (~isnan(state.snrLast(i))) && (state.snrLast(i) >= cfg.thresholdSNR);
        state.txSuccSeries{i}(end+1) = double(sched_val > 0 && snr_ok);
        state.txSuccSlots{i}(end+1)  = slot;
        state.snrSeries{i}(end+1)    = state.snrLast(i);
        state.snrSlots{i}(end+1)     = slot;
    end

    state = step_risk(state, rhoMap, mapGrid, slot, cfg);

    if ~cfg.headless
        update_dashboard(dashboard, state, scene, txSlots, slot, cfg);
    end
end

if ~cfg.headless && exist('slot','var')
    update_dashboard(dashboard, state, scene, txSlots, slot, cfg);
end

results = collect_results(state, infra, cfg);
results.preempt_count = preempt_count;
results.resume_count  = resume_count;
end


%% ========================================================================
%%  Tabela comparativa
%% ========================================================================
function print_comparison(res)
if isempty(res), return; end

fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════════════════╗\n');
fprintf('║                    COMPARAÇÃO DE POLÍTICAS                          ║\n');
fprintf('╠════════════════════╦══════════╦══════════╦══════════╦═══════════════╣\n');
fprintf('║ Política           ║ n_deliv  ║ slotsVid ║ Jhat_emp ║ Preempções   ║\n');
fprintf('╠════════════════════╬══════════╬══════════╬══════════╬═══════════════╣\n');

for k = 1:numel(res)
    r  = res{k};
    nd = sum(r.nDelivered);
    sv = sum(r.slotsVid);
    jh = r.Jhat_emp;
    pr = sum(r.preempt_count);
    fprintf('║ %-18s ║ %8d ║ %8d ║ %8.4f ║ %13d ║\n', ...
            r.policy, nd, sv, jh, pr);
end
fprintf('╚════════════════════╩══════════╩══════════╩══════════╩═══════════════╝\n\n');
end


%% ========================================================================
%%  Helpers
%% ========================================================================
function s = format_scheduled(txSlots, N)
% Compacta lista de fontes escaladas: "d1:C2 d2:Vid"
parts = {};
for u = 1:N
    src_c2  = 2*(u-1) + 1;
    src_vid = 2*(u-1) + 2;
    if any(txSlots == src_c2),  parts{end+1} = sprintf('d%d:C2',  u); end %#ok<AGROW>
    if any(txSlots == src_vid), parts{end+1} = sprintf('d%d:Vid', u); end %#ok<AGROW>
end
if isempty(parts), s = '(none)'; else, s = strjoin(parts, ' '); end
end
