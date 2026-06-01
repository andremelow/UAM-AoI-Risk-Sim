function txSlots = policy_aoi_normalized(activeDrones, K, state, ~, cfg)
% POLICY_AOI_NORMALIZED  AoI scheduling normalised by service cost per source.
%
%  Corrige o viés do aoi-pure: em vez de comparar h1 e h2 em slots brutos,
%  normaliza cada score pelo número de slots necessários para entregar uma
%  actualização, tornando as fontes comparáveis em termos de melhoria de
%  AoI por slot gasto.
%
%    score_C2(u)  = h1(u) / L_c2         L_c2 = 1  (1 slot por pacote C2)
%    score_Vid(u) = h2(u) / L_vid        L_vid = cfg.L_vid  (100 por defeito)
%
%  Interpretação: score = "quanto de AoI esta fonte eliminaria por slot
%  investido, assumindo sucesso". Fontes com melhor retorno por slot têm
%  prioridade — evita que o vídeo domine só por acumular AoI mais depressa.
%
%  Relação com outras políticas:
%    aoi-pure          — score = h  (sem normalização)
%    aoi-normalized    — score = h / L  (esta política)
%    max-weight-drift  — score = w·h + w·drift  (pesos + componente dinâmica)

L_c2  = cfg.L_c2;    % slots por pacote C2  (tipicamente 1)
L_vid = cfg.L_vid;   % slots por frame de vídeo (tipicamente 100)

n       = numel(activeDrones);
sources = zeros(1, 2*n);
scores  = zeros(1, 2*n);

for idx = 1:n
    u = activeDrones(idx);
    % C2 — normalizado por L_c2
    sources(2*idx-1) = 2*(u-1) + 1;
    scores(2*idx-1)  = state.dual(u).h1 / L_c2;
    % Vídeo — normalizado por L_vid
    sources(2*idx)   = 2*(u-1) + 2;
    scores(2*idx)    = state.dual(u).h2 / L_vid;
end

[~, order] = sort(scores, 'descend');
txSlots = sources(order(1:min(K, 2*n)));
end
