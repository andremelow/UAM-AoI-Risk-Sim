function [txSlots, state] = policy_reserve_c2_sw(eligible, K_free, state, cfg)
% POLICY_RESERVE_C2_SW  Switching policy with reserved C2 slots.
%
%  Reserva um número fixo de slots exclusivamente para C2 a cada época,
%  usando Round-Robin para escolher qual drone serve em cada slot reservado.
%  Os slots restantes são abertos (qualquer fonte) com RR geral.
%
%  Regras:
%    K = 1 : 1 slot → C2 apenas
%    K = 2 : 1 slot → C2 (RR),  1 slot → vídeo apenas (RR)
%    K ≥ 3 : K_c2 = ceil(K/3) slots → C2 (RR),  K-K_c2 slots → RR geral
%
%  Dois ponteiros RR independentes são mantidos no estado:
%    state.rrPtr_c2  — aponta para o próximo drone a servir C2 (1..N)
%    state.rrPtr_all — aponta para a próxima fonte a servir nos slots abertos (1..2N)

N       = cfg.numDrones;
K       = min(cfg.dronesPerSlot, K_free);
n_elig  = numel(eligible);

% ── Partition K into reserved C2 + open slots ────────────────────────────────
if K <= 1
    K_c2   = 1;  K_vid  = 0;  K_open = 0;
elseif K == 2
    K_c2   = 1;  K_vid  = 1;  K_open = 0;
else
    K_c2   = ceil(K / 3);
    K_vid  = 0;
    K_open = K - K_c2;
end

% ── Initialize RR pointers ────────────────────────────────────────────────────
if ~isfield(state, 'rrPtr_c2'),  state.rrPtr_c2  = 1; end
if ~isfield(state, 'rrPtr_all'), state.rrPtr_all = 1; end

txSlots     = [];
used_drones = [];

% ── Phase 1: Reserved C2 slots — RR over eligible drones (C2 source) ─────────
ptr     = state.rrPtr_c2;
chosen  = 0;
scanned = 0;
while chosen < K_c2 && chosen < n_elig && scanned < N
    u = ptr;
    if ismember(u, eligible) && ~ismember(u, used_drones)
        txSlots(end+1)     = 2*(u-1) + 1;   %#ok<AGROW>  C2 source (odd)
        used_drones(end+1) = u;              %#ok<AGROW>
        chosen = chosen + 1;
    end
    ptr     = mod(ptr, N) + 1;
    scanned = scanned + 1;
end
state.rrPtr_c2 = ptr;

% ── Phase 2: Video-only slot (K=2 case) — RR over eligible drones (vid) ──────
if K_vid > 0
    ptr     = state.rrPtr_all;
    chosen  = 0;
    scanned = 0;
    while chosen < K_vid && scanned < N
        u = ptr;
        if ismember(u, eligible) && ~ismember(u, used_drones)
            txSlots(end+1)     = 2*(u-1) + 2;   %#ok<AGROW>  vid source (even)
            used_drones(end+1) = u;              %#ok<AGROW>
            chosen = chosen + 1;
        end
        ptr     = mod(ptr, N) + 1;
        scanned = scanned + 1;
    end
    state.rrPtr_all = ptr;
end

% ── Phase 3: Open slots (K≥3) — RR over all remaining sources ────────────────
if K_open > 0
    ptr     = state.rrPtr_all;
    chosen  = 0;
    scanned = 0;
    N2      = 2 * N;
    n_rem   = n_elig - numel(used_drones);
    while chosen < K_open && chosen < n_rem && scanned < N2
        src = ptr;
        u   = ceil(src / 2);
        if ismember(u, eligible) && ~ismember(u, used_drones)
            txSlots(end+1)     = src;   %#ok<AGROW>
            used_drones(end+1) = u;     %#ok<AGROW>
            chosen = chosen + 1;
        end
        ptr     = mod(ptr, N2) + 1;
        scanned = scanned + 1;
    end
    state.rrPtr_all = ptr;
end
end
