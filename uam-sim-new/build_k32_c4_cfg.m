function cfg = build_k32_c4_cfg(policy, K, C, T_steady)
%BUILD_K32_C4_CFG  Monta cfg para debug de K=32 C=4 (geometria assimétrica).
%
%  Uso:
%    cfg = build_k32_c4_cfg()                        % defaults
%    cfg = build_k32_c4_cfg('max-weight-drift')
%    cfg = build_k32_c4_cfg('round-robin', 32, 4)
%    run_sim(build_k32_c4_cfg())

if nargin < 1 || isempty(policy),  policy   = 'max-weight-drift'; end
if nargin < 2 || isempty(K),       K        = 32;  end
if nargin < 3 || isempty(C),       C        = 4;   end
if nargin < 4 || isempty(T_steady),T_steady = 900; end

setup_paths();

%% Base config
cfg = build_scenario_assimetrico();
cfg.w_unc = 1/3; cfg.w_map = 1/3; cfg.w_vid = 1/3;

cfg.arrivalModel    = 'capacity_drain';
cfg.minStartDelay   = 2;
cfg.steady_duration = T_steady;

cfg.videoModel = 'atomic'; cfg.fps = 30;
cfg.L_vid = 100; cfg.L_c2 = 1; cfg.w_coh = 5; cfg.L_nom = 0.01;
cfg.p_c2 = 0.80; cfg.p_vid = 0.80; cfg.rho_0 = 0.01;

%% Rotas 3 km (East ±1500 m)
r1.numDrones = 1; r1.start = [200, -1500]; r1.goal = [500,  1500]; r1.label = 'Route2';
r2.numDrones = 1; r2.start = [-500, 1500]; r2.goal = [-600,-1500]; r2.label = 'Route3';
routes = [r1, r2];
nR     = numel(routes);

flightTime = norm(routes(1).goal - routes(1).start) / cfg.speedVal;
C_per = floor(C / nR);
C_rem = C - C_per * nR;
for ri = 1:nR
    C_r = C_per + (ri <= C_rem);
    spacing = flightTime / max(C_r, 1);
    routes(ri).numDrones = C_r + floor(T_steady / spacing);
end
N_total = sum([routes.numDrones]);

cfg.routes           = routes;
cfg.corridorLength   = norm(routes(1).goal - routes(1).start);
cfg.flightTime       = flightTime;
cfg.corridorCapacity = C;
cfg.dronesPerSlot    = K;
cfg.routing.mode     = 'static';
cfg.maxStartDelay    = cfg.minStartDelay + (N_total - 1) * flightTime/max(C,1) + 10;

cfg.H_max  = []; cfg.omega  = []; cfg.beta_s = []; cfg.beta_v = [];

cfg.schedulingPolicy = policy;
cfg.headless         = false;
cfg.csvExport        = false;

fprintf('[build_k32_c4_cfg] K=%d  C=%d  N=%d  política=%s\n', K, C, N_total, policy);
end
