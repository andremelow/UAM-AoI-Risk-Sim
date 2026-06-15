function results = run_sim(cfg)
% RUN_SIM  UAM-AoI-Risk Simulator — modular entry point.
%
%  Usage:
%    run_sim()                              % default config
%    run_sim(build_scenario_config())       % explicit config
%    cfg = build_scenario_config(); cfg.numDrones = 30; run_sim(cfg);
%
%  The monolithic UAM_AoI_Control_Sim_WithRisk.m is replaced by this
%  orchestrator, which delegates to focused modules in core/, viz/, policies/.

if nargin < 1, cfg = build_scenario_config(); end

% Ensure all subdirectories are on the path
setup_paths();

cfg = validate_uam_config(cfg);

% Switching policies: one radio per drone per slot, so extra slots beyond
% N_eff bring no per-drone throughput gain. Use qbar_sw (computed with
% K_eff=N_eff) to prevent x_s divergence that starves video in max-weight-sw.
SW_POLICIES = {'round-robin-sw', 'pf-classic-sw', 'max-weight-sw'};
if isfield(cfg, 'qbar_s_sw') && ismember(cfg.schedulingPolicy, SW_POLICIES)
    cfg.qbar_s = cfg.qbar_s_sw;
    cfg.qbar_v = cfg.qbar_v_sw;
end

headless = isfield(cfg, 'headless') && cfg.headless;

if ~headless, clc; close all; end

% --- Initialisation ---
[scene, drones, infra] = init_scenario(cfg);
state                  = init_state(cfg, infra);
[rhoMap, mapGrid]      = init_ground_risk(cfg);
% --- Risk-A* pre-flight trajectory planning (upstream of scheduler) ---
% mapGrid must exist before this block; init_ground_risk runs above.
% Trajectories are stored as (T x 2) [North, East] matrices in
% infra.precomputed_trajectory{u}, indexed by local slot offset.
% step_read_positions uses them instead of read(drones{u}) when present.
if isfield(cfg, 'routing') && strcmp(cfg.routing.mode, 'risk_astar')
    infra.precomputed_trajectory = cell(cfg.numDrones, 1);
    fprintf('[Routing] Risk-A* enabled (w_d=%.2f, w_r=%.2f) — planning %d drones...\n', ...
            cfg.routing.w_d, cfg.routing.w_r, cfg.numDrones);
    for u = 1:cfg.numDrones
        infra.precomputed_trajectory{u} = ...
            route_risk_astar(infra, rhoMap, mapGrid, cfg, u);
    end
    fprintf('[Routing] Done.\n');
else
    infra.precomputed_trajectory = {};   % empty → static mode, no override
end

if ~headless
    dashboard = init_dashboard(cfg, rhoMap, mapGrid);
end

% --- Main loop ---
txSlots = [];   % defined here so the post-loop dashboard refresh always has it
while advance(scene)
    time = scene.CurrentTime;              % seconds — uavScenario API only
    slot = round(time * cfg.updateRate);   % integer slot index

    state = step_read_positions(state, drones, infra, time, slot, cfg);

    % Early exit: all drones have landed
    if slot > max(state.endSlots) && isempty(state.activeDrones)
        break;
    end

    state = step_traffic(state, slot, cfg);

    [txSlots, state] = schedule_sources(state, slot, cfg);

    state = step_channel(state, infra, time, cfg);

    state = step_transmit(state, txSlots, slot, cfg);

    % Record scheduled source and tx outcome per drone
    for i = state.activeDrones
        % Decode which source (if any) was scheduled for this drone
        src_c2  = 2*(i-1)+1;  src_vid = 2*(i-1)+2;
        if any(txSlots == src_c2)
            sched_val = 1;
        elseif any(txSlots == src_vid)
            sched_val = 2;
        else
            sched_val = 0;
        end
        state.schedSeries{i}(end+1) = sched_val;
        state.schedSlots{i}(end+1)  = slot;
        % tx success: 1 if scheduled and SNR ok, 0 if scheduled and failed, -1 not scheduled
        if sched_val > 0
            snr_ok = (~isnan(state.snrLast(i))) && (state.snrLast(i) >= cfg.thresholdSNR);
            state.txSuccSeries{i}(end+1) = double(snr_ok);
        else
            state.txSuccSeries{i}(end+1) = -1;
        end
        state.txSuccSlots{i}(end+1) = slot;
    end

    state = step_risk(state, rhoMap, mapGrid, slot, cfg);

    % Accumulate per-slot series for CSV export and SNR spatial display
    for i = state.activeDrones
        snr_i = state.snrLast(i);
        % SNR spatial trail (dashboard)
        if ~isnan(snr_i) && ~headless
            dashboard.snrNorthHist{i}(end+1) = state.pendingMsg{i}.pos(1);
            dashboard.snrValHist{i}(end+1)   = snr_i;
        end
        % Dense series for CSV export
        state.snrSeries{i}(end+1)  = state.snrLast(i);
        state.snrSlots{i}(end+1)   = slot;
    end

    if ~headless
        update_dashboard(dashboard, state, scene, txSlots, slot, cfg);
    end
end

% Final dashboard refresh: the early-exit break fires after step_read_positions
% marks the last drone invalid but before update_dashboard runs, so the status
% panel would otherwise show the last drone as still active after landing.
if ~headless && exist('slot','var')
    update_dashboard(dashboard, state, scene, txSlots, slot, cfg);
end

% --- Post-simulation ---
results = collect_results(state, infra, cfg);
if isfield(cfg, 'csvExport') && cfg.csvExport
    outDir = 'csv_export';
    if isfield(cfg, 'csvDir'), outDir = cfg.csvDir; end
    export_csv(results, cfg, outDir);
end

fprintf('\nSimulation complete.\n');
end
