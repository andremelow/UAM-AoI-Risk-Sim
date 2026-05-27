%% dual_source_rr_risk_5drones.m
% =========================================================================
%  Discrete-time AoI & Safety-Risk Simulator — 5 Drones
%  -----------------------------------------------------
%  Scenario : 5 drones, 1 BS, 2 uplink sources per drone (C2 + video)
%  Policy   : Round-Robin across all 10 sources (packet-level)
%  Risk     : 3-component model from the draft (Sec. II-A)
%             R_nav    — navigation overlap   (now active: pairwise)
%             R_ground — ground-exposure       (uniform density, closed-form)
%             R_vid    — video coherence        (weighted-window AoI)
%
%  Channel: 10 sources share 1 slot → each source gets ~1/10 of capacity.
%           Under RR the C2 AoI grows ≈ 10× compared to the 1-drone case,
%           making the bottleneck visible in ALL three risk components.
%
%  Usage:  >> dual_source_rr_risk_5drones
% =========================================================================
clear; close all; clc;

bgGray = [0.9 0.9 0.9];
set(groot, 'defaultAxesColor', bgGray);
set(groot, 'defaultFigureColor', bgGray);

%% =====================  PARAMETERS  =====================================
rng(42);

N       = 5;                    % number of drones
T       = 10000;                % simulation horizon [slots]
dt      = 1e-3;                 % slot duration = 1 ms

% --- Channel reliabilities (per drone, identical for now) ---
p_c2  = 0.8;                   % C2 channel reliability
p_vid = 0.8;                   % video channel reliability

% --- Source 1: C2 / telemetry ---
L_c2 = 1;                      % single-packet update

% --- Source 2: Video (GOP structure) ---
fps            = 30;
frame_interval = round(1/(fps*dt));   % ≈ 33 slots
GOP            = 15;
L_I            = 30;                  % I-frame packets
L_P            = 3;                   % P-frame packets

% --- Physical / corridor ---
v_max          = 15;                  % max UAV speed [m/s]
r_min          = 25;                  % minimum separation radius [m]
corridor_len   = 1800;               % corridor length [m]
drone_spacing  = 120;                % initial spacing between drones [m]
drone_y        = 250;                % y-position (straight line corridor)

% --- Ground risk (uniform) ---
rho_0  = 0.01;                       % risk density [people/m²]

% --- Video coherence window ---
w_coh = 5;
L_nom = 0.010;                       % nominal one-way latency [s]
H_nom = L_nom + 2*(w_coh - 1)/(3*fps);

% Linearly decreasing weights (eq. 14)
alpha_w = zeros(w_coh, 1);
for ii = 1:w_coh
    alpha_w(ii) = 2*(w_coh + 1 - ii) / (w_coh*(w_coh + 1));
end

% --- Normalisation constants ---
%   With N drones and 2 sources each, RR period = 2N.
%   Nominal C2 AoI under RR:  h* ≈ 2N / p_c2  slots.
h_star_slots = 2*N / p_c2;
h_star_s     = h_star_slots * dt;
r_star       = r_min + v_max * h_star_s;

% R_bar_nav (eq. 7):  worst-case with N-1 neighbours at d_min
d_min        = 2 * r_min;            % minimum designed separation
R_bar_nav    = max((N - 1) * max(2*r_star - d_min, 0), 1e-15);

% R_bar_ground (eq. 11)
R_bar_gnd    = rho_0 * pi * r_star^2;

% --- Risk weights (must sum to 1) ---
w_N = 0.33;   w_G = 0.33;   w_V = 0.33;

fprintf('=== Simulation: %d drones, Round-Robin ===\n', N);
fprintf('  T = %d slots (%.1f s),  dt = %.0f ms\n', T, T*dt, dt*1e3);
fprintf('  Sources: %d (= %d drones × 2)\n', 2*N, N);
fprintf('  Channel: p_c2=%.2f, p_vid=%.2f\n', p_c2, p_vid);
fprintf('  Video: %d fps, GOP=%d, L_I=%d, L_P=%d\n', fps, GOP, L_I, L_P);
fprintf('  Nominal: h*=%.1f slots (%.1f ms), r*=%.2f m\n', ...
        h_star_slots, h_star_s*1e3, r_star);
fprintf('  R_bar_nav=%.4f, R_bar_gnd=%.6f\n', R_bar_nav, R_bar_gnd);
fprintf('  Weights: w_N=%.2f  w_G=%.2f  w_V=%.2f\n\n', w_N, w_G, w_V);

%% =====================  STATE VARIABLES  ================================

% Per-drone AoI (in slots)
h1 = ones(N, 1);               % C2 AoI
h2 = ones(N, 1);               % Video AoI

% Per-drone video source state
vid_remaining   = zeros(N, 1);
vid_gen_slot    = zeros(N, 1);
frame_counter   = zeros(N, 1);
next_frame_slot = ones(N, 1);  % first frame at t=1 for all

% Per-drone video coherence window
frame_gen_hist  = nan(w_coh, N);
n_delivered     = zeros(N, 1);

% Drone initial x-positions (staggered along corridor)
drone_x0 = 50 + (0:N-1)' * drone_spacing;

% Round-robin pointer: cycles through 2N sources
%   Source ordering:  [C2_1, vid_1, C2_2, vid_2, ..., C2_N, vid_N]
rr_ptr = 1;
n_sources = 2 * N;

% Logging (per-drone)
h1_log    = zeros(T, N);
h2_log    = zeros(T, N);
R_nav_log = zeros(T, N);
R_gnd_log = zeros(T, N);
R_vid_log = zeros(T, N);
R_tot_log = zeros(T, N);
R_sys_log = zeros(T, 1);       % system-wide risk
r_u_log   = zeros(T, N);

%% =====================  MAIN LOOP  ======================================

for t = 1:T

    % --- 1) Log current AoI ---
    h1_log(t,:) = h1';
    h2_log(t,:) = h2';

    % --- 2) Video frame arrival (per drone) ---
    for u = 1:N
        if t >= next_frame_slot(u)
            frame_counter(u) = frame_counter(u) + 1;
            if mod(frame_counter(u) - 1, GOP) == 0
                L_new = L_I;
            else
                L_new = L_P;
            end
            vid_remaining(u)   = L_new;
            vid_gen_slot(u)    = t;
            next_frame_slot(u) = t + frame_interval;
        end
    end

    % --- 3) Round-Robin scheduling: pick 1 of 2N sources ---
    src = rr_ptr;
    rr_ptr = mod(rr_ptr, n_sources) + 1;

    % Decode source → drone index u and type (1=C2, 2=video)
    u_sched    = ceil(src / 2);
    is_video   = (mod(src, 2) == 0);

    % --- 4) Transmission & AoI evolution ---
    h1_next = h1 + 1;          % default: all ages grow
    h2_next = h2 + 1;

    if ~is_video
        % C2 packet for drone u_sched
        if rand() < p_c2
            h1_next(u_sched) = 1;
        end
    else
        % Video packet for drone u_sched
        if vid_remaining(u_sched) > 0
            if rand() < p_vid
                vid_remaining(u_sched) = vid_remaining(u_sched) - 1;
                if vid_remaining(u_sched) == 0
                    h2_next(u_sched) = (t + 1) - vid_gen_slot(u_sched);
                    n_delivered(u_sched) = n_delivered(u_sched) + 1;
                    frame_gen_hist(:, u_sched) = ...
                        [vid_gen_slot(u_sched); frame_gen_hist(1:end-1, u_sched)];
                end
            end
        end
    end

    h1 = h1_next;
    h2 = h2_next;

    % --- 5) Compute risk components ---

    % Drone positions (straight line, wrap around corridor)
    pos_x = mod(drone_x0 + v_max * t * dt, corridor_len);
    pos_y = drone_y * ones(N, 1);

    % Uncertainty radii (from C2 AoI, eq. 2)
    r_u = r_min + v_max * h1_log(t,:)' * dt;
    r_u_log(t,:) = r_u';

    for u = 1:N

        % ---- R_nav (eq. 6): pairwise overlap ----
        R_nav_u = 0;
        for v = 1:N
            if v == u, continue; end
            d_uv = sqrt((pos_x(u) - pos_x(v))^2 + (pos_y(u) - pos_y(v))^2);
            overlap = r_u(u) + r_u(v) - d_uv;
            if overlap > 0
                R_nav_u = R_nav_u + overlap;
            end
        end
        R_nav_norm = R_nav_u / R_bar_nav;

        % ---- R_ground (eq. 9, closed-form uniform density) ----
        R_gnd_raw  = rho_0 * pi * r_u(u)^2;
        R_gnd_norm = R_gnd_raw / max(R_bar_gnd, 1e-15);

        % ---- R_vid (eq. 17) ----
        if n_delivered(u) >= w_coh
            H_W = 0;
            for ii = 1:w_coh
                h_frame = (t - frame_gen_hist(ii, u)) * dt;
                H_W = H_W + alpha_w(ii) * h_frame;
            end
            R_vid_u = max(H_W / H_nom - 1, 0);
        else
            R_vid_u = max(h2_log(t, u) * dt / H_nom - 1, 0);
        end

        % ---- Total risk (eq. 18) ----
        R_tot_u = w_N * R_nav_norm + w_G * R_gnd_norm + w_V * R_vid_u;

        % Store
        R_nav_log(t, u) = R_nav_norm;
        R_gnd_log(t, u) = R_gnd_norm;
        R_vid_log(t, u) = R_vid_u;
        R_tot_log(t, u) = R_tot_u;
    end

    % System-wide risk (eq. 20)
    R_sys_log(t) = mean(R_tot_log(t, :));
end

%% =====================  SUMMARY STATISTICS  =============================

fprintf('=== Round-Robin Results (%d drones, T=%d slots = %.1f s) ===\n', N, T, T*dt);
for u = 1:N
    fprintf('  Drone %d:  <h_C2>=%5.1f ms   <h_vid>=%6.1f ms   frames=%d   <R_u>=%.4f\n', ...
        u, mean(h1_log(:,u))*dt*1e3, mean(h2_log(:,u))*dt*1e3, ...
        n_delivered(u), mean(R_tot_log(:,u)));
end
fprintf('  ---\n');
fprintf('  System <R_nav>   = %.6f\n', mean(R_nav_log(:)));
fprintf('  System <R_ground>= %.6f\n', mean(R_gnd_log(:)));
fprintf('  System <R_vid>   = %.6f\n', mean(R_vid_log(:)));
fprintf('  System <R_sys>   = %.6f\n', mean(R_sys_log));
fprintf('  Peak   R_sys     = %.6f\n', max(R_sys_log));

%% =====================  PLOTS  ==========================================

time_s = (1:T)' * dt;
win    = 200;

% ---- Colour map for drones ----
cmap = lines(N);

% =========================================================================
%  FIGURE 1 — Per-drone AoI  (C2 and Video, stacked)
% =========================================================================
figure('Name','AoI per drone', 'Position',[60 80 1200 700], 'Color', bgGray);

ax_a1 = subplot(2,1,1);
set(ax_a1, 'Color', bgGray);
hold on;
for u = 1:N
    plot(time_s, h1_log(:,u)*dt*1e3, 'Color', [cmap(u,:) 0.6], 'LineWidth', 0.3);
end
ylabel('$h_1$ (C2) [ms]', 'Interpreter','latex');
title(sprintf('C2 / Telemetry AoI — Round-Robin  (%d drones, %d sources)', N, 2*N));
lg = legend(arrayfun(@(u) sprintf('drone %d',u), 1:N, 'Uni',0), ...
            'Location','northeast','FontSize',7);
grid on; xlim([0 T*dt]); set(ax_a1,'FontSize',9);

ax_a2 = subplot(2,1,2);
set(ax_a2, 'Color', bgGray);
hold on;
for u = 1:N
    plot(time_s, h2_log(:,u)*dt*1e3, 'Color', [cmap(u,:) 0.6], 'LineWidth', 0.3);
end
ylabel('$h_2$ (video) [ms]', 'Interpreter','latex');
xlabel('Time [s]');
title('Video AoI');
grid on; xlim([0 T*dt]); set(ax_a2,'FontSize',9);
linkaxes([ax_a1 ax_a2], 'x');

% =========================================================================
%  FIGURE 2 — Risk components + R_sys  (4 panels)
% =========================================================================
figure('Name','Risk components', 'Position',[80 40 1200 850], 'Color', bgGray);

% ---- Panel 1: R_nav ----
ax1 = subplot(4,1,1);
set(ax1, 'Color', bgGray); hold on;
for u = 1:N
    plot(time_s, movmean(R_nav_log(:,u), win), 'Color', cmap(u,:), 'LineWidth', 1);
end
ylabel('$\tilde{R}^{\mathrm{Nav}}_u$', 'Interpreter','latex');
title(sprintf('Navigation Overlap Risk — %d drones', N));
legend(arrayfun(@(u) sprintf('drone %d',u), 1:N, 'Uni',0), ...
       'Location','northeast','FontSize',7);
grid on; xlim([0 T*dt]); set(ax1,'FontSize',9);

% ---- Panel 2: R_ground ----
ax2 = subplot(4,1,2);
set(ax2, 'Color', bgGray); hold on;
for u = 1:N
    plot(time_s, movmean(R_gnd_log(:,u), win), 'Color', cmap(u,:), 'LineWidth', 1);
end
ylabel('$\tilde{R}^{\mathrm{Ground}}_u$', 'Interpreter','latex');
title('Ground-Exposure Risk  (uniform density, closed-form)');
grid on; xlim([0 T*dt]); set(ax2,'FontSize',9);

% ---- Panel 3: R_vid ----
ax3 = subplot(4,1,3);
set(ax3, 'Color', bgGray); hold on;
for u = 1:N
    plot(time_s, movmean(R_vid_log(:,u), win), 'Color', cmap(u,:), 'LineWidth', 1);
end
ylabel('$R^{\mathrm{vid}}_u$', 'Interpreter','latex');
title('Video-Coherence Risk');
grid on; xlim([0 T*dt]); set(ax3,'FontSize',9);

% ---- Panel 4: R_sys (system-wide) ----
ax4 = subplot(4,1,4);
set(ax4, 'Color', bgGray); hold on;
plot(time_s, R_sys_log, 'Color', [0.7 0.7 0.7], 'LineWidth', 0.3);
plot(time_s, movmean(R_sys_log, win), 'k', 'LineWidth', 1.5);
yline(1, '--r', '$R_{\mathrm{sys}}=1$', 'Interpreter','latex', ...
      'LineWidth', 1, 'LabelHorizontalAlignment','left','FontSize',8);
ylabel('$R_{\mathrm{sys}}(t)$', 'Interpreter','latex');
xlabel('Time [s]');
title(sprintf('System-Wide Risk   (w_N=%.2f, w_G=%.2f, w_V=%.2f)', w_N, w_G, w_V));
legend('instantaneous','moving avg','Location','northeast','FontSize',7);
grid on; xlim([0 T*dt]); set(ax4,'FontSize',9);

linkaxes([ax1 ax2 ax3 ax4], 'x');

% =========================================================================
%  FIGURE 3 — Per-drone total risk R_u(t)
% =========================================================================
figure('Name','Per-drone total risk', 'Position',[100 60 1200 400], 'Color', bgGray);
ax5 = axes; set(ax5, 'Color', bgGray); hold on;
for u = 1:N
    plot(time_s, movmean(R_tot_log(:,u), win), 'Color', cmap(u,:), 'LineWidth', 1.2);
end
yline(1, '--r', 'LineWidth', 1);
ylabel('$R_u(t)$', 'Interpreter','latex');
xlabel('Time [s]');
title(sprintf('Total Risk per Drone — Round-Robin (%d drones)', N));
legend([arrayfun(@(u) sprintf('drone %d',u), 1:N, 'Uni',0), {'nominal'}], ...
       'Location','northeast','FontSize',8);
grid on; xlim([0 T*dt]); set(ax5,'FontSize',9);

fprintf('\nDone.  Three figures generated.\n');