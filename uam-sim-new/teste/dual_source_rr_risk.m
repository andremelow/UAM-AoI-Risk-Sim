%% dual_source_rr_risk.m
% =========================================================================
%  Discrete-time AoI & Safety-Risk Simulator
%  -----------------------------------------
%  Scenario : 1 drone, 1 BS, 2 uplink sources (C2 + video)
%  Policy   : Round-Robin (packet-level alternation)
%  Risk     : 3-component model from the draft (Sec. II-A)
%             R_nav  — navigation overlap  (= 0 for single drone)
%             R_ground — ground-exposure    (uniform density grid)
%             R_vid  — video coherence      (weighted-window AoI)
%
%  Mapping to "Optimizing AoI in Networks with Large and Small Updates"
%  (Zhao, Tripathi, Kadota):
%     Source 1  ↔  C2/telemetry   : L_1 = 1 packet   (small update)
%     Source 2  ↔  H.265 video    : L_2 variable      (large update)
%     Channel   ↔  unreliable, p_i ∈ (0,1]
%     K = 1     ↔  one transmission per slot
%
%  Traffic model inspired by AVIATOR (Baltaci et al., INFOCOM 2021):
%     Video frames arrive at fps rate following a GOP structure
%     I-frames → large   (L_I packets)
%     P-frames → small   (L_P packets)
%     Single-update buffer: new frame replaces any unfinished frame.
%
%  Usage:  >> dual_source_rr_risk
% =========================================================================
clear; close all; clc;
bgGray = [0.9 0.9 0.9];


%% =====================  PARAMETERS  =====================================
rng(42);                        % reproducibility

T       = 6000;                % simulation horizon [slots]
dt      = 1e-3;                 % slot duration = 1 ms  (5G NR subframe)

% --- Channel reliabilities ---
p  = [0.8, 0.8];               % [C2, video]

% --- Source 1: C2 / telemetry (small update, generate-at-will) ---
L_c2 = 1;                      % single-packet update

% --- Source 2: Video (multi-packet, GOP structure) ---
fps            = 30;            % camera frame rate
frame_interval = round(1/(fps*dt));  % ≈ 33 slots between frames
GOP            = 15;            % group-of-pictures length
L_I            = 30;            % I-frame: 30 packets  (~45 KB @ 1500 B/pkt)
L_P            = 3;             % P-frame: 3 packets   (~4.5 KB)

% --- Physical / drone ---
v_max       = 15;               % max UAV speed [m/s]
r_min       = 25;                % minimum separation radius [m]
drone_speed = v_max;            % fly at max speed (worst case)
drone_y     = 250;              % straight-line corridor y-position [m]
drone_x0    = 50;               % initial x-position [m]

% --- Ground risk grid (uniform density) ---
grid_W = 500;   grid_H = 500;  % coverage area [m × m]
n_x    = 50;    n_y    = 50;   % number of cells per axis
cell_w = grid_W / n_x;
cell_h = grid_H / n_y;
dA     = cell_w * cell_h;      % cell area [m²]
rho_0  = 0.01;                 % uniform risk density [people/m²]

% Cell centroids (column vectors)
[cx, cy] = meshgrid(cell_w/2 : cell_w : grid_W, ...
                     cell_h/2 : cell_h : grid_H);
cx = cx(:);  cy = cy(:);       % n_x·n_y × 1

% --- Video coherence window (draft eq. 14–17) ---
w_coh = 5;                     % window size [frames]
L_nom = 0.010;                 % nominal one-way latency [s]
H_nom = L_nom + 2*(w_coh - 1)/(3*fps);   % eq. (16)  ≈ 0.099 s

% Linearly decreasing weights  α_i  (eq. 14)
alpha_w = zeros(w_coh, 1);
for ii = 1:w_coh
    alpha_w(ii) = 2*(w_coh + 1 - ii) / (w_coh*(w_coh + 1));
end

% --- Normalisation constants (nominal corridor, draft eq. 7, 11) ---
%   Nominal C2 AoI under RR:  E[h] ≈ 2/p(1) slots  (geometric service,
%   every-other-slot scheduling).  In seconds: h* = 2/(p(1)) * dt.
h_star_s   = (2 / p(1)) * dt;              % nominal C2 AoI [s]
r_star     = r_min + v_max * h_star_s;      % nominal uncertainty radius [m]
R_bar_nav  = 1;                             % placeholder (single drone → 0)
R_bar_gnd  = rho_0 * pi * r_star^2;        % draft eq. (11)

% --- Risk weights (must sum to 1) ---
w_N = 0.33;   w_G = 0.33;   w_V = 0.33;

fprintf('=== Simulation Parameters ===\n');
fprintf('  T = %d slots  (%.1f s)\n', T, T*dt);
fprintf('  Channel: p_C2 = %.2f,  p_vid = %.2f\n', p(1), p(2));
fprintf('  Video: %d fps, GOP=%d,  L_I=%d pkts,  L_P=%d pkts\n', ...
        fps, GOP, L_I, L_P);
fprintf('  Frame interval = %d slots (%.1f ms)\n', frame_interval, frame_interval*dt*1e3);
fprintf('  Nominal: h*=%.3f ms,  r*=%.4f m,  H_nom=%.4f s\n', ...
        h_star_s*1e3, r_star, H_nom);
fprintf('  Weights: w_nav=%.1f  w_gnd=%.1f  w_vid=%.1f\n', w_N, w_G, w_V);
fprintf('  Ground grid: %dx%d cells,  ρ_0 = %.3f /m²\n\n', n_x, n_y, rho_0);

%% =====================  STATE VARIABLES  ================================

% AoI (in slots)
h1 = 1;                        % C2  AoI
h2 = 1;                        % Video AoI (time since last COMPLETE delivery)

% Video source state
vid_remaining  = 0;             % packets left in current frame
vid_gen_slot   = 0;             % generation slot of current frame
frame_counter  = 0;             % counts generated frames (for GOP indexing)
next_frame_slot = 1;            % next frame arrival time [slot]

% Video coherence window history (most-recent first)
frame_gen_hist = nan(w_coh, 1); % generation slots of last w delivered frames
n_delivered    = 0;             % total frames delivered so far

% Round-robin pointer
rr_turn = 1;                   % 1 → schedule C2,  2 → schedule video

% Logging arrays
h1_log      = zeros(T, 1);
h2_log      = zeros(T, 1);
R_nav_log   = zeros(T, 1);
R_gnd_log   = zeros(T, 1);
R_vid_log   = zeros(T, 1);
R_tot_log   = zeros(T, 1);
r_u_log     = zeros(T, 1);     % uncertainty radius [m]
frame_type  = zeros(T, 1);     % 0=none, 1=I-frame arrival, 2=P-frame arrival

%% =====================  MAIN LOOP  =====================================

for t = 1:T

    % ------------------------------------------------------------------
    %  1) Log current AoI  (beginning-of-slot values)
    % ------------------------------------------------------------------
    h1_log(t) = h1;
    h2_log(t) = h2;

    % ------------------------------------------------------------------
    %  2) Video frame arrival
    % ------------------------------------------------------------------
    if t >= next_frame_slot
        frame_counter = frame_counter + 1;
        if mod(frame_counter - 1, GOP) == 0
            L_new = L_I;                  % I-frame
            frame_type(t) = 1;
        else
            L_new = L_P;                  % P-frame
            frame_type(t) = 2;
        end
        % Single-update buffer: replace any unfinished frame
        vid_remaining  = L_new;
        vid_gen_slot   = t;
        next_frame_slot = t + frame_interval;
    end

    % ------------------------------------------------------------------
    %  3) Round-Robin scheduling decision
    % ------------------------------------------------------------------
    sched = rr_turn;
    rr_turn = 3 - rr_turn;               % toggle 1 ↔ 2

    % ------------------------------------------------------------------
    %  4) Transmission & AoI evolution  →  compute h(t+1)
    % ------------------------------------------------------------------
    h1_next = h1 + 1;                     % default: information ages
    h2_next = h2 + 1;

    if sched == 1
        % --- C2 scheduled: generate-at-will, L = 1 ---
        if rand() < p(1)
            h1_next = 1;                  % fresh packet delivered
        end
        % Video not served → h2 ages (already set above)

    else
        % --- Video scheduled ---
        if vid_remaining > 0
            if rand() < p(2)
                vid_remaining = vid_remaining - 1;
                if vid_remaining == 0
                    % === Complete video update delivered! ===
                    h2_next = (t + 1) - vid_gen_slot;
                    %   h2(t+1) = (t+1) − generation timestamp

                    % Shift coherence-window history and record
                    n_delivered = n_delivered + 1;
                    frame_gen_hist = [vid_gen_slot; frame_gen_hist(1:end-1)];
                end
            end
        end
        % C2 not served → h1 ages (already set above)
    end

    h1 = h1_next;
    h2 = h2_next;

    % ------------------------------------------------------------------
    %  5) Compute risk components  (using beginning-of-slot AoI)
    % ------------------------------------------------------------------

    % Drone position (straight line, wrap-around)
    drone_x = drone_x0 + drone_speed * t * dt;
    drone_x = mod(drone_x, grid_W);
    pos = [drone_x, drone_y];

    % ---- R_nav (eq. 6):  zero for single-drone scenario ----
    R_nav = 0;

    % ---- R_ground (eq. 10):  uniform density over uncertainty disk ----
    %   Uncertainty radius from C2 AoI (eq. 2)
    r_u = r_min + v_max * h1_log(t) * dt;     % [m]
    r_u_log(t) = r_u;

    %   Count cells whose centroid falls inside B_u(t)
    %dist_sq = (cx - pos(1)).^2 + (cy - pos(2)).^2;
    %cells_in = sum(dist_sq <= r_u^2);

    %R_gnd_raw = rho_0 * dA * cells_in;
    %R_gnd     = R_gnd_raw / max(R_bar_gnd, 1e-15);   % normalised (eq. 12)

    R_gnd_raw = rho_0 * pi * r_u^2;                        % exact integral
    R_gnd     = R_gnd_raw / max(R_bar_gnd, 1e-15);         % normalised
    % ---- R_vid (eq. 17):  coherence-window excess ----
    if n_delivered >= w_coh
        % Full window available — weighted mean frame AoI
        H_W = 0;
        for ii = 1:w_coh
            h_frame = (t - frame_gen_hist(ii)) * dt;  % [s]
            H_W = H_W + alpha_w(ii) * h_frame;
        end
        R_vid = max(H_W / H_nom - 1, 0);             % eq. (17)
    else
        % Not enough frames delivered yet → fallback: use raw video AoI
        R_vid = max(h2_log(t) * dt / H_nom - 1, 0);
    end

    % ---- Total risk (eq. 18) ----
    R_tot = w_N * R_nav  +  w_G * R_gnd  +  w_V * R_vid;

    % Store
    R_nav_log(t) = R_nav;
    R_gnd_log(t) = R_gnd;
    R_vid_log(t) = R_vid;
    R_tot_log(t) = R_tot;
end

%% =====================  SUMMARY STATISTICS  =============================

fprintf('=== Round-Robin Results (T = %d slots = %.1f s) ===\n', T, T*dt);
fprintf('  Mean AoI  C2   :  %8.2f slots  (%6.2f ms)\n', ...
        mean(h1_log), mean(h1_log)*dt*1e3);
fprintf('  Mean AoI  Video:  %8.2f slots  (%6.2f ms)\n', ...
        mean(h2_log), mean(h2_log)*dt*1e3);
fprintf('  Peak AoI  C2   :  %8.0f slots  (%6.2f ms)\n', ...
        max(h1_log), max(h1_log)*dt*1e3);
fprintf('  Peak AoI  Video:  %8.0f slots  (%6.2f ms)\n', ...
        max(h2_log), max(h2_log)*dt*1e3);
fprintf('  Video frames delivered: %d / %d generated\n', ...
        n_delivered, frame_counter);
fprintf('  ---\n');
fprintf('  Mean R_nav   :  %.6f\n', mean(R_nav_log));
fprintf('  Mean R_ground:  %.6f\n', mean(R_gnd_log));
fprintf('  Mean R_vid   :  %.6f\n', mean(R_vid_log));
fprintf('  Mean R_total :  %.6f\n', mean(R_tot_log));
fprintf('  Peak R_total :  %.6f\n', max(R_tot_log));

%% =====================  PLOTS  ==========================================

time_s  = (1:T)' * dt;                      % [s]
h1_ms   = h1_log * dt * 1e3;                % [ms]
h2_ms   = h2_log * dt * 1e3;                % [ms]

% Smoothing for risk traces (100-sample moving average for readability)
win = 100;
R_gnd_sm = movmean(R_gnd_log, win);
R_vid_sm = movmean(R_vid_log, win);
R_tot_sm = movmean(R_tot_log, win);

% ---- Colour palette ----
col_c2   = [0.00 0.45 0.74];    % blue
col_vid  = [0.85 0.33 0.10];    % orange-red
col_gnd  = [0.13 0.55 0.13];    % forest green
col_rvid = [0.80 0.20 0.20];    % red
col_tot  = [0.10 0.10 0.10];    % near-black

bgGray   = [0.9 0.9 0.9];    % light gray axes background

figure('Name','Dual-Source Round-Robin — AoI & Risk', ...
       'Position',[80 50 1100 900], 'Color',bgGray);

% -------  Panel 1: Age of Information  -----------------------------------
ax1 = subplot(5,1,1);
set(ax1, 'Color', bgGray);
plot(time_s, h1_ms, 'Color', col_c2,  'LineWidth', 0.4); hold on;
plot(time_s, h2_ms, 'Color', col_vid, 'LineWidth', 0.4);
ylabel('AoI [ms]');
title('Age of Information — Round-Robin  (1 drone, 2 sources)');
legend('h_1  (C2 / telemetry)', 'h_2  (video)', ...
       'Location','northeast','FontSize',8);
grid on;  xlim([0 T*dt]);
set(ax1,'FontSize',9);

% -------  Panel 2: R_nav  ------------------------------------------------
ax2 = subplot(5,1,2);
set(ax2, 'Color', bgGray);
plot(time_s, R_nav_log, 'Color', [0.5 0.5 0.5], 'LineWidth', 0.6);
ylabel('$\tilde{R}^{\mathrm{Nav}}$','Interpreter','latex');
title('Navigation Overlap Risk  (R_{Nav} = 0,  single drone)');
ylim([-0.1 1]);
grid on;  xlim([0 T*dt]);
set(ax2,'FontSize',9);

% -------  Panel 3: R_ground  ---------------------------------------------
ax3 = subplot(5,1,3);
set(ax3, 'Color', bgGray);
plot(time_s, R_gnd_log, 'Color', col_gnd, 'LineWidth', 0.3, ...
     'DisplayName','raw'); hold on;
plot(time_s, R_gnd_sm,  'Color', col_gnd, 'LineWidth', 1.8, ...
     'DisplayName','moving avg');
ylabel('$\tilde{R}^{\mathrm{Ground}}$','Interpreter','latex');
title('Ground-Exposure Risk  (uniform density)');
legend('Location','northeast','FontSize',8);
grid on;  xlim([0 T*dt]);
set(ax3,'FontSize',9);

% -------  Panel 4: R_vid  ------------------------------------------------
ax4 = subplot(5,1,4);
set(ax4, 'Color', bgGray);
plot(time_s, R_vid_log, 'Color', col_rvid, 'LineWidth', 0.3, ...
     'DisplayName','raw'); hold on;
plot(time_s, R_vid_sm,  'Color', col_rvid, 'LineWidth', 1.8, ...
     'DisplayName','moving avg');
ylabel('$R^{\mathrm{vid}}$','Interpreter','latex');
title('Video-Coherence Risk');
legend('Location','northeast','FontSize',8);
grid on;  xlim([0 T*dt]);
set(ax4,'FontSize',9);

% -------  Panel 5: R_total  ----------------------------------------------
ax5 = subplot(5,1,5);
set(ax5, 'Color', bgGray);
plot(time_s, R_tot_log, 'Color', [0.7 0.7 0.7], 'LineWidth', 0.3, ...
     'DisplayName','raw'); hold on;
plot(time_s, R_tot_sm,  'Color', col_tot, 'LineWidth', 1.8, ...
     'DisplayName','moving avg');
yline(1, '--r', 'R_u = 1  (nominal capacity)', 'LineWidth', 1, ...
      'LabelHorizontalAlignment','left','FontSize',8);
ylabel('$R_u(t)$','Interpreter','latex');
xlabel('Time [s]');
title(sprintf('Total Safety Risk   (w_{Nav}=%.1f,  w_{Gnd}=%.1f,  w_{Vid}=%.1f)', ...
      w_N, w_G, w_V));
legend('raw','moving avg','Location','northeast','FontSize',8);
grid on;  xlim([0 T*dt]);
set(ax5,'FontSize',9);

linkaxes([ax1 ax2 ax3 ax4 ax5], 'x');

% -------  Auxiliary figure: AoI zoom + frame arrivals  --------------------
figure('Name','AoI Detail — first 2 seconds', ...
       'Position',[200 150 900 400], 'Color',bgGray);

ax6 = gca;
set(ax6, 'Color', bgGray);
t_zoom = time_s <= 2;  % first 2 seconds
plot(time_s(t_zoom), h1_ms(t_zoom), 'Color', col_c2,  'LineWidth', 1); hold on;
plot(time_s(t_zoom), h2_ms(t_zoom), 'Color', col_vid, 'LineWidth', 1);

% Mark frame arrivals
idx_I = find(frame_type == 1 & t_zoom);
idx_P = find(frame_type == 2 & t_zoom);
if ~isempty(idx_I)
    stem(time_s(idx_I), h2_ms(idx_I), 'k^', 'filled', 'MarkerSize', 6, ...
         'DisplayName','I-frame arrival');
end
if ~isempty(idx_P)
    stem(time_s(idx_P), h2_ms(idx_P), 'kv', 'MarkerSize', 4, ...
         'DisplayName','P-frame arrival');
end
ylabel('AoI [ms]');
xlabel('Time [s]');
title('AoI Detail — Round-Robin (first 2 s)');
legend('h_1 (C2)','h_2 (video)','I-frame','P-frame', ...
       'Location','best','FontSize',9);
grid on;

fprintf('\nDone.  Two figures generated.\n');