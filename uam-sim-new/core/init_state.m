function state = init_state(cfg, infra)
% INIT_STATE  Initializes all mutable simulation state.
%
%  Doc-aligned (Sec. 4.1): per-UAV state S(t) includes
%    h_{u,s}(t)  — status AoI            -> dual(u).h1
%    h_{u,v}(t)  — video frame AoI window -> dual(u).frame_gen_hist
%    z_u(t)      — coherence-window AoI  -> dual(u).z_u (computed in step_risk)
%    r_u(t)      — remaining service time -> dual(u).vid_remaining
%    x_{u,s}(t)  — status debt queue      -> dual(u).x_s
%    x_{u,v}(t)  — video debt queue       -> dual(u).x_v

N = cfg.numDrones;

% --- Dual-source state per drone ---
for u = N:-1:1
    state.dual(u).h1              = 0;
    state.dual(u).lastRx_c2       = NaN;

    state.dual(u).h2              = 0;
    state.dual(u).lastRx_vid      = NaN;
    state.dual(u).vid_remaining   = 0;
    state.dual(u).vid_in_flight   = false;   % true iff a video tx already
                                              % consumed a channel in a previous
                                              % slot and must be continued (Sec.
                                              % 4.1: "ongoing video must be
                                              % continued"). Distinct from
                                              % vid_remaining > 0, which only
                                              % means a frame is pending in the
                                              % buffer ready to be started.
    state.dual(u).vid_gen_slot    = 0;
    state.dual(u).vid_L           = 0;
    state.dual(u).vid_pending     = false;
    state.dual(u).vid_pending_slot = 0;
    state.dual(u).vid_pending_L   = 0;
    state.dual(u).frame_counter   = 0;
    state.dual(u).next_frame_slot = infra.startSlots(u);

    state.dual(u).frame_gen_hist  = nan(cfg.w_coh, 1);
    state.dual(u).n_delivered     = 0;

    % Doc Sec. 4.1: coherence-window AoI z_u (computed in step_risk)
    state.dual(u).z_u             = 0;

    % Doc Sec. 4.1: debt queues x_{u,s}, x_{u,v}
    %    x_{u,s}(t+1) = [x_{u,s}(t) + qbar_s - mu_s(t)]^+
    %    x_{u,v}(t+1) = [x_{u,v}(t) + qbar_v - c_u(t)]^+
    state.dual(u).x_s             = 0;
    state.dual(u).x_v             = 0;
end

% --- Communication state ---
state.pendingMsg            = cell(N, 1);
state.lastRxSlot            = zeros(N, 1);
state.snrLast               = nan(N, 1);
state.txSuccess             = zeros(N, 1);
state.txFail                = zeros(N, 1);
state.slotsC2               = zeros(N, 1);
state.slotsVid              = zeros(N, 1);
state.slotsC2Fail           = zeros(N, 1);
state.slotsVidFail          = zeros(N, 1);
state.avgThroughput         = ones(N, 1) * 1e-3;
state.avgThroughputC2       = ones(N, 1) * 1e-3;
state.avgThroughputVid      = ones(N, 1) * 1e-3;
state.rawThroughput         = ones(N, 1) * 1e-3;

for k = 1:N
    state.pendingMsg{k} = struct('pos',[NaN NaN NaN],'timestamp',NaN,'valid',false);
end

% --- Risk history ---
state.riskTime = cell(N,1);  state.riskUnc = cell(N,1);
state.riskMap  = cell(N,1);  state.riskVid = cell(N,1);
state.riskTot  = cell(N,1);  state.riskHat = cell(N,1);
for i = 1:N
    state.riskTime{i} = []; state.riskUnc{i} = [];
    state.riskMap{i}  = []; state.riskVid{i} = [];
    state.riskTot{i}  = []; state.riskHat{i} = [];
end

% --- Aggregate metrics ---
state.rSysTime = []; state.rSysData = [];
state.JhatTime = []; state.JhatData = [];
state.uncMeanT = []; state.uncMeanV = [];
state.mapMeanT = []; state.mapMeanV = [];
state.vidMeanT = []; state.vidMeanV = [];
state.h1MeanT  = []; state.h1MeanV  = [];
state.h2MeanT  = []; state.h2MeanV  = [];

% --- Per-drone AoI history ---
state.h1Time = cell(N,1); state.h1Val = cell(N,1);
state.h2Time = cell(N,1); state.h2Val = cell(N,1);
state.zuTime = cell(N,1); state.zuVal = cell(N,1);
state.xsVal  = cell(N,1); state.xvVal = cell(N,1);
for i = 1:N
    state.h1Time{i} = []; state.h1Val{i} = [];
    state.h2Time{i} = []; state.h2Val{i} = [];
    state.zuTime{i} = []; state.zuVal{i} = [];
    state.xsVal{i}  = []; state.xvVal{i} = [];
end

% --- Timing ---
state.startTimes  = infra.startTimes;
state.endTimes    = infra.endTimes;
state.startSlots  = infra.startSlots;
state.endSlots    = infra.endSlots;

% --- Scheduler ---
state.rrPointer = 1;

% --- Per-step transient ---
state.activeDrones = [];
state.activePos    = zeros(N, 3);
state.aoiRadius    = zeros(N, 1);

% --- Per-drone dense series for CSV export ---
state.snrSeries   = cell(N, 1);
state.snrSlots    = cell(N, 1);
state.schedSeries = cell(N, 1);
state.schedSlots  = cell(N, 1);
state.txSuccSeries = cell(N, 1);
state.txSuccSlots  = cell(N, 1);
for u = 1:N
    state.snrSeries{u}    = [];
    state.snrSlots{u}     = [];
    state.schedSeries{u}  = [];
    state.schedSlots{u}   = [];
    state.txSuccSeries{u} = [];
    state.txSuccSlots{u}  = [];
end
end
