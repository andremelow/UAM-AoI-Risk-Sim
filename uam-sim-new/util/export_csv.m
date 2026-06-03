function export_csv(results, cfg, outDir)
% EXPORT_CSV  Exports simulation results as CSV files for analysis/dashboards.
%
%  Generates three files in outDir:
%
%  1. <runId>_slot_series.csv   — per-slot time series (dense, one row per slot)
%     Columns: slot, drone_id, h1_slots, h2_slots, snr_db, r_unc, r_map, r_vid,
%              r_tot, source_scheduled (0=none,1=C2,2=vid), tx_success
%
%  2. <runId>_drone_summary.csv — one row per drone, aggregated over flight
%     Columns: drone_id, entry_slot, exit_slot, slots_c2_ok, slots_c2_fail,
%              slots_vid_ok, slots_vid_fail, frames_delivered, frame_counter,
%              mean_h1, mean_h2, peak_h1, peak_h2, mean_r_vid, mean_r_tot
%
%  3. <runId>_run_manifest.csv  — single-row scenario descriptor
%     Columns: run_id, policy, num_drones, corridor_length, update_rate,
%              fps, L_vid, w_coh, slots_per_frame, fc_ghz, threshold_snr,
%              w_unc, w_map, w_vid, sim_slots, timestamp
%
%  runId is auto-generated as: <policy>_N<numDrones>_<timestamp>

if nargin < 3
    outDir = fullfile(pwd, 'csv_export');
end
if ~exist(outDir, 'dir'), mkdir(outDir); end

% Helper: open file with mkdir fallback if directory disappeared mid-run.
    function fid = safe_fopen(filepath)
        fid = fopen(filepath, 'w');
        if fid == -1
            mkdir(fileparts(filepath));
            fid = fopen(filepath, 'w');
        end
        if fid == -1
            error('export_csv:fopen', 'Cannot open for writing: %s', filepath);
        end
    end

% --- Run ID ---
ts = datestr(now, 'yyyymmdd_HHMMSS');
runId = sprintf('%s_N%d_%s', cfg.schedulingPolicy, cfg.numDrones, ts);

fprintf('[CSV] Exporting run: %s\n', runId);

N         = cfg.numDrones;
totalSlots = results.rSysTime(end);

% =========================================================
%  1. SLOT SERIES
% =========================================================
slotFile = fullfile(outDir, [runId '_slot_series.csv']);
fid = safe_fopen(slotFile);
fprintf(fid, 'slot,drone_id,h1_slots,h2_slots,snr_db,r_unc,r_map,r_vid,r_tot,source_sched,tx_success\n');

for u = 1:N
    if isempty(results.h1Time{u}), continue; end

    slots  = results.h1Time{u};
    h1     = results.h1Val{u};
    h2     = results.h2Val{u};
    r_unc  = interp_series(results.riskTime{u}, results.riskUnc{u}, slots);
    r_map  = interp_series(results.riskTime{u}, results.riskMap{u}, slots);
    r_vid  = interp_series(results.riskTime{u}, results.riskVid{u}, slots);
    r_tot  = cfg.w_unc*r_unc + cfg.w_map*r_map + cfg.w_vid*r_vid;

    % SNR series — stored in results if available, else NaN
    if isfield(results, 'snrSeries') && ~isempty(results.snrSeries{u})
        snr = interp_series(results.snrSlots{u}, results.snrSeries{u}, slots);
    else
        snr = nan(size(slots));
    end

    % Source scheduled series
    if isfield(results, 'schedSeries') && ~isempty(results.schedSeries{u})
        sched = interp_series(results.schedSlots{u}, results.schedSeries{u}, slots);
    else
        sched = nan(size(slots));
    end

    % tx success (1 if C2 or vid delivered that slot, 0 otherwise)
    if isfield(results, 'txSuccessSeries') && ~isempty(results.txSuccessSeries{u})
        txs = interp_series(results.txSuccSlots{u}, results.txSuccSeries{u}, slots);
    else
        txs = nan(size(slots));
    end

    for k = 1:numel(slots)
        fprintf(fid, '%d,%d,%.1f,%.1f,%.4f,%.6f,%.6f,%.6f,%.6f,%d,%d\n', ...
            slots(k), u, h1(k), h2(k), ...
            safe_val(snr, k), safe_val(r_unc, k), safe_val(r_map, k), ...
            safe_val(r_vid, k), safe_val(r_tot, k), ...
            safe_int(sched, k), safe_int(txs, k));
    end
end
fclose(fid);
fprintf('[CSV]   slot_series   → %s\n', slotFile);

% =========================================================
%  2. DRONE SUMMARY
% =========================================================
sumFile = fullfile(outDir, [runId '_drone_summary.csv']);
fid = safe_fopen(sumFile);
fprintf(fid, ['drone_id,entry_slot,exit_slot,' ...
    'slots_c2_ok,slots_c2_fail,slots_vid_ok,slots_vid_fail,' ...
    'frames_delivered,frame_counter,' ...
    'mean_h1,mean_h2,peak_h1,peak_h2,mean_r_vid,mean_r_tot\n']);

for u = 1:N
    entrySlot = results.startSlots(u);
    exitSlot  = results.endSlots(u);

    h1v   = results.h1Val{u};
    h2v   = results.h2Val{u};
    rvidv = results.riskVid{u};
    runcv = results.riskUnc{u};
    rmapv = results.riskMap{u};

    if isempty(h1v)
        mean_h1=0; mean_h2=0; peak_h1=0; peak_h2=0; mean_rvid=0; mean_rtot=0;
    else
        mean_h1 = mean(h1v);
        mean_h2 = mean(h2v);
        peak_h1 = max(h1v);
        peak_h2 = max(h2v);
        if ~isempty(rvidv)
            n_common = min([numel(runcv), numel(rmapv), numel(rvidv)]);
            r_tot_v = cfg.w_unc*runcv(1:n_common) + cfg.w_map*rmapv(1:n_common) + cfg.w_vid*rvidv(1:n_common);
            mean_rvid = mean(rvidv);
            mean_rtot = mean(r_tot_v);
        else
            mean_rvid = 0; mean_rtot = 0;
        end
    end

    fprintf(fid, '%d,%d,%d,%d,%d,%d,%d,%d,%d,%.2f,%.2f,%.0f,%.0f,%.4f,%.4f\n', ...
        u, entrySlot, exitSlot, ...
        results.slotsC2(u),    results.slotsC2Fail(u), ...
        results.slotsVid(u),   results.slotsVidFail(u), ...
        results.nDelivered(u), results.frameCounter(u), ...
        mean_h1, mean_h2, peak_h1, peak_h2, mean_rvid, mean_rtot);
end
fclose(fid);
fprintf('[CSV]   drone_summary → %s\n', sumFile);

% =========================================================
%  3. RUN MANIFEST
% =========================================================
manFile = fullfile(outDir, [runId '_manifest.csv']);
fid = safe_fopen(manFile);
cap = 0;
if isfield(cfg, 'corridorCapacity') && cfg.corridorCapacity > 0
    cap = cfg.corridorCapacity;
end
routing_mode = 'static';
if isfield(cfg, 'routing') && isfield(cfg.routing, 'mode')
    routing_mode = cfg.routing.mode;
end
num_routes = numel(cfg.routes);
fprintf(fid, ['run_id,policy,num_drones,corridor_length,update_rate,' ...
    'fps,L_vid,w_coh,slots_per_frame,fc_ghz,threshold_snr,' ...
    'w_unc,w_map,w_vid,sim_slots,timestamp,corridor_capacity,' ...
    'routing_mode,num_routes\n']);
fprintf(fid, '%s,%s,%d,%g,%d,%g,%d,%d,%.3f,%.2f,%g,%.4f,%.4f,%.4f,%d,%s,%d,%s,%d\n', ...
    runId, cfg.schedulingPolicy, cfg.numDrones, cfg.corridorLength, ...
    cfg.updateRate, cfg.fps, cfg.L_vid, cfg.w_coh, cfg.slots_per_frame, ...
    cfg.fc/1e9, cfg.thresholdSNR, cfg.w_unc, cfg.w_map, cfg.w_vid, ...
    totalSlots, ts, cap, routing_mode, num_routes);
fclose(fid);
fprintf('[CSV]   manifest      → %s\n', manFile);

fprintf('[CSV] Done. Run ID: %s\n', runId);
end

%% ── helpers ──────────────────────────────────────────────────────────────

function v = interp_series(t_src, v_src, t_dst)
% Nearest-neighbour alignment: for each slot in t_dst, find closest in t_src.
if isempty(t_src) || isempty(v_src)
    v = nan(size(t_dst)); return;
end
v = zeros(size(t_dst));
for k = 1:numel(t_dst)
    [~, idx] = min(abs(t_src - t_dst(k)));
    v(k) = v_src(idx);
end
end

function v = safe_val(arr, k)
if k <= numel(arr), v = arr(k); else, v = NaN; end
end

function v = safe_int(arr, k)
if k <= numel(arr) && ~isnan(arr(k)), v = round(arr(k)); else, v = -1; end
end
