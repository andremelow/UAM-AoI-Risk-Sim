function [startTimes, arrivalMeta] = generate_arrival_times(cfg)
% GENERATE_ARRIVAL_TIMES  Generates drone entry times according to the
%   arrival model specified in cfg.arrivalModel.
%
%   Models
%   ------
%   'uniform'       — original behaviour: uniform random in [minDelay, maxDelay]
%   'nhpp'          — Non-Homogeneous Poisson Process driven by cfg.lambdaProfile
%   'batch'         — Poisson batches; each event spawns cfg.batchSize drones
%   'deterministic' — equally-spaced entry; stress-tests scheduler at fixed load
%   'rush_hour'     — composite: low + two Gaussian peaks (morning / evening)
%
%   Outputs
%   -------
%   startTimes   [numDrones×1] sorted entry times (seconds)
%   arrivalMeta  struct with diagnostics (model used, effective λ, batch sizes…)
%
%   All models honour cfg.numDrones as the hard drone count.
%   If the stochastic models generate more or fewer events, the result is
%   trimmed or replicated to match numDrones exactly.

narginchk(1, 1);

N   = cfg.numDrones;
rng(cfg.rng_seed);   % reproducible for a given scenario seed

model = lower(cfg.arrivalModel);

arrivalMeta.model = model;
arrivalMeta.seed  = cfg.rng_seed;

switch model

    % ------------------------------------------------------------------
    case 'uniform'
        % Original behaviour — preserved for baseline comparisons
        t = sort(cfg.minStartDelay + rand(N,1) .* ...
                 (cfg.maxStartDelay - cfg.minStartDelay));
        arrivalMeta.description = 'Uniform random (original)';
        arrivalMeta.lambda_eff  = N / (cfg.maxStartDelay - cfg.minStartDelay);

    % ------------------------------------------------------------------
    case 'nhpp'
        % Non-Homogeneous Poisson Process via thinning (Lewis-Shedler).
        %
        % cfg.lambdaProfile  — function handle @(t) returning rate [drones/s]
        %                      evaluated over [0, cfg.nhpp_horizon]
        % cfg.nhpp_horizon   — total time window to generate events [s]
        %                      (default: maxStartDelay * 2)
        % cfg.nhpp_lambda_max — upper bound for thinning (auto if absent)

        horizon    = get_field(cfg, 'nhpp_horizon',    cfg.maxStartDelay * 2);
        lambda_fn  = get_field(cfg, 'lambdaProfile',   @(t) ones(size(t)) * N/horizon);

        % Find lambda_max by sampling the profile densely
        t_grid     = linspace(0, horizon, 5000);
        lam_vals   = lambda_fn(t_grid);
        lambda_max = get_field(cfg, 'nhpp_lambda_max', max(lam_vals) * 1.05);

        % Generate homogeneous Poisson with rate lambda_max, then thin
        expected_n = lambda_max * horizon;
        candidates = cumsum(exprnd(1/lambda_max, ceil(expected_n * 3), 1));
        candidates = candidates(candidates <= horizon);

        % Thinning: accept each candidate with probability λ(t)/λ_max
        u        = rand(size(candidates));
        accept   = u <= (lambda_fn(candidates) ./ lambda_max);
        events   = candidates(accept);

        t = reconcile_to_N(events, N, horizon);

        arrivalMeta.description  = 'Non-Homogeneous Poisson (thinning)';
        arrivalMeta.lambda_max   = lambda_max;
        arrivalMeta.lambda_eff   = numel(events) / horizon;
        arrivalMeta.nhpp_horizon = horizon;
        arrivalMeta.raw_events   = numel(events);

    % ------------------------------------------------------------------
    case 'batch'
        % Poisson arrival of groups; models vertiport departure waves.
        %
        % cfg.batchLambda  — group arrival rate [groups/s]  (default 0.3)
        % cfg.batchSize    — [min max] drones per group      (default [2 5])
        % cfg.batchJitter  — std of within-group offset [s] (default 1.5)

        horizon     = get_field(cfg, 'nhpp_horizon',  cfg.maxStartDelay * 2);
        blambda     = get_field(cfg, 'batchLambda',   0.3);
        bsize_range = get_field(cfg, 'batchSize',     [2 5]);
        jitter_std  = get_field(cfg, 'batchJitter',   1.5);

        batch_times = [];
        t_now       = 0;
        while t_now < horizon
            t_now = t_now + exprnd(1/blambda);
            if t_now > horizon, break; end
            n_batch = randi(bsize_range, 1);
            offsets = jitter_std * randn(n_batch, 1);
            batch_times = [batch_times; t_now + offsets]; %#ok<AGROW>
        end
        batch_times = sort(max(batch_times, 0));
        batch_times = batch_times(batch_times <= horizon);

        t = reconcile_to_N(batch_times, N, horizon);

        arrivalMeta.description  = 'Poisson batch arrivals';
        arrivalMeta.batchLambda  = blambda;
        arrivalMeta.batchSize    = bsize_range;
        arrivalMeta.total_events = numel(batch_times);

    % ------------------------------------------------------------------
    case 'deterministic'
        % Equally-spaced entries — maximum scheduler stress at a fixed load.
        %
        % cfg.det_spacing  — seconds between consecutive entries (default auto)

        spacing = get_field(cfg, 'det_spacing', ...
                  (cfg.maxStartDelay - cfg.minStartDelay) / max(N-1, 1));
        t = cfg.minStartDelay + (0:N-1)' * spacing;

        arrivalMeta.description = 'Deterministic equally-spaced';
        arrivalMeta.spacing_s   = spacing;
        arrivalMeta.lambda_eff  = 1 / spacing;

    % ------------------------------------------------------------------
    case 'rush_hour'
        % Composite profile: background Poisson + two Gaussian peaks.
        %
        % cfg.rh_peak1_t    — centre of morning peak [s]  (default 60)
        % cfg.rh_peak2_t    — centre of evening peak [s]  (default 240)
        % cfg.rh_peak_width — std of each peak [s]        (default 30)
        % cfg.rh_base_rate  — background rate [drones/s]  (default 0.05)
        % cfg.rh_peak_rate  — peak rate [drones/s]        (default 0.6)

        horizon  = get_field(cfg, 'nhpp_horizon',   cfg.maxStartDelay * 2);
        t1       = get_field(cfg, 'rh_peak1_t',     horizon * 0.20);
        t2       = get_field(cfg, 'rh_peak2_t',     horizon * 0.70);
        pw       = get_field(cfg, 'rh_peak_width',  horizon * 0.08);
        base_r   = get_field(cfg, 'rh_base_rate',   0.05);
        peak_r   = get_field(cfg, 'rh_peak_rate',   0.6);

        lambda_fn = @(t) base_r ...
            + peak_r * exp(-0.5*((t-t1)/pw).^2) ...
            + peak_r * exp(-0.5*((t-t2)/pw).^2);

        % Re-use NHPP thinning
        cfg_tmp               = cfg;
        cfg_tmp.lambdaProfile = lambda_fn;
        cfg_tmp.nhpp_horizon  = horizon;
        cfg_tmp.arrivalModel  = 'nhpp';
        [t, meta_tmp]         = generate_arrival_times(cfg_tmp);

        arrivalMeta.description = 'Rush-hour composite (2 Gaussian peaks)';
        arrivalMeta.peak1_t     = t1;
        arrivalMeta.peak2_t     = t2;
        arrivalMeta.lambda_fn   = lambda_fn;
        arrivalMeta.nhpp_meta   = meta_tmp;
        return   % t already reconciled inside recursive call

    % ------------------------------------------------------------------
    otherwise
        warning('generate_arrival_times: unknown model "%s", falling back to uniform.', model);
        cfg_tmp              = cfg;
        cfg_tmp.arrivalModel = 'uniform';
        [t, arrivalMeta]     = generate_arrival_times(cfg_tmp);
        return
end

startTimes = sort(t(:));
arrivalMeta.startTimes = startTimes;
end

%% ======================================================================
%  LOCAL HELPERS
%% ======================================================================

function t = reconcile_to_N(events, N, horizon)
% Trim or replicate event list to exactly N entries.
events = sort(events(:));
if numel(events) >= N
    t = events(1:N);
else
    % Too few events — pad by sampling uniformly in the remaining window
    n_missing = N - numel(events);
    pad = sort(rand(n_missing,1) * horizon);
    t   = sort([events; pad]);
end
end

function v = get_field(s, fname, default)
% Safe field read with default fallback.
if isfield(s, fname) && ~isempty(s.(fname))
    v = s.(fname);
else
    v = default;
end
end