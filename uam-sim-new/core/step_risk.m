function state = step_risk(state, ~, mapGrid, slot, cfg)
% STEP_RISK  Computes risk components per UAV.
%
%  Doc alignment (Sec. 2-3):
%    R_nav   — eq. (Sec. 2): pairwise overlap (raw or /R_bar_nav)
%    R_gnd   — eq. (Sec. 2): closed-form for uniform rho (raw or /R_bar_gnd)
%    R_vid   — eq. (Sec. 2): coherence-window weighted AoI (already dimensionless)
%    R_tot   — eq. (Sec. 2): weighted sum
%
%  Surrogate (Sec. 3):
%    Rhat_u  = n_s * h_{u,s}(t) + n_v * z_u(t)
%    z_u(t)  = sum_i alpha_i * h_{u,v}^{(i)}(t)
%
%  cfg.use_normalization (default false) selects between:
%    false (doc-aligned): R_nav and R_gnd reported in raw units, so the
%                          surrogate bound R_u <= C + n_s h + n_v z holds.
%    true (legacy):       both normalized to [0,1] for visualization parity
%                          with old experiments.

useNorm = isfield(cfg, 'use_normalization') && cfg.use_normalization;

for i = state.activeDrones
    pos = state.pendingMsg{i}.pos;
    r_u = state.aoiRadius(i);

    % ---- R_nav: pairwise overlap (Sec. 2) ----
    R_nav_raw = 0;
    for j = state.activeDrones
        if j == i, continue; end
        d_uv = norm(pos(1:2) - state.activePos(j,1:2));
        overlap = r_u + state.aoiRadius(j) - d_uv;
        if overlap > 0
            R_nav_raw = R_nav_raw + overlap;
        end
    end
    if useNorm
        R_nav = R_nav_raw / cfg.R_bar_nav;
    else
        R_nav = R_nav_raw;          % raw units (matches Sec. 3 bound)
    end

    % ---- R_ground: integral of rho over disk B_u (Sec. 2) ----
    if isfield(cfg, 'uniform_ground') && cfg.uniform_ground
        R_gnd_raw = cfg.rho_0 * pi * r_u^2;
    else
        inDisk = sqrt((mapGrid.xFlat - pos(1)).^2 + ...
                      (mapGrid.yFlat - pos(2)).^2) <= r_u;
        R_gnd_raw = sum(mapGrid.rhoFlat(inDisk)) * mapGrid.deltaA;
    end
    if useNorm
        R_gnd = R_gnd_raw / cfg.R_bar_gnd;
    else
        R_gnd = R_gnd_raw;
    end

    % ---- R_vid: coherence-window AoI (Sec. 2, eq. R_vid) ----
    nd = state.dual(i).n_delivered;
    if nd >= cfg.w_coh
        % Full window: weighted mean over last w_coh frames
        z_u = 0;
        for ii = 1:cfg.w_coh
            h_frame_slots = slot - state.dual(i).frame_gen_hist(ii);
            z_u = z_u + cfg.alpha_w(ii) * h_frame_slots;
        end
    elseif nd > 0
        % Partial window (warmup): renormalised weights for continuity
        z_u = 0;
        w_sum = sum(cfg.alpha_w(1:nd));
        for ii = 1:nd
            h_frame_slots = slot - state.dual(i).frame_gen_hist(ii);
            z_u = z_u + (cfg.alpha_w(ii) / w_sum) * h_frame_slots;
        end
    else
        % No frames delivered yet — use h2 as single-frame proxy
        z_u = state.dual(i).h2;
    end
    state.dual(i).z_u = z_u;                          % stored for scheduler
    R_vid = max(z_u / cfg.H_nom_slots - 1, 0);

    % ---- Total risk R_u (Sec. 2, eq. R_u) ----
    R_tot = cfg.w_unc * R_nav + cfg.w_map * R_gnd + cfg.w_vid * R_vid;

    % ---- Linear surrogate Rhat_u (Sec. 3, final boxed) ----
    Rhat_u = cfg.n_s * state.dual(i).h1 + cfg.n_v * z_u;

    % Store
    state.riskTime{i}(end+1) = slot;
    state.riskUnc{i}(end+1)  = R_nav;
    state.riskMap{i}(end+1)  = R_gnd;
    state.riskVid{i}(end+1)  = R_vid;
    state.riskTot{i}(end+1)  = R_tot;
    state.riskHat{i}(end+1)  = Rhat_u;
end

% --- Aggregate metrics ---
activeMask = arrayfun(@(k) state.pendingMsg{k}.valid, 1:cfg.numDrones);
if any(activeMask)
    idx = find(activeMask);
    state.uncMeanT(end+1) = slot;
    state.uncMeanV(end+1) = mean(arrayfun(@(k) state.riskUnc{k}(end), idx));
    state.mapMeanT(end+1) = slot;
    state.mapMeanV(end+1) = mean(arrayfun(@(k) state.riskMap{k}(end), idx));
    state.vidMeanT(end+1) = slot;
    state.vidMeanV(end+1) = mean(arrayfun(@(k) state.riskVid{k}(end), idx));
    state.h1MeanT(end+1) = slot;
    state.h1MeanV(end+1) = mean(arrayfun(@(k) state.dual(k).h1, idx));
    state.h2MeanT(end+1) = slot;
    state.h2MeanV(end+1) = mean(arrayfun(@(k) state.dual(k).h2, idx));

    % weighted system risk R_sys = sum_u omega_u R_u
    R_sys = sum(arrayfun(@(k) cfg.omega(k) * state.riskTot{k}(end), idx));
    state.rSysTime(end+1) = slot;
    state.rSysData(end+1) = R_sys;

    % Linear-surrogate aggregate Jhat(t) = sum_u omega_u Rhat_u
    Jhat_inst = sum(arrayfun(@(k) cfg.omega(k) * state.riskHat{k}(end), idx));
    state.JhatTime(end+1) = slot;
    state.JhatData(end+1) = Jhat_inst;
end
end
