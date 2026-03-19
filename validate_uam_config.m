function cfg = validate_uam_config(cfg)
%% Fills missing fields with defaults. Call at the top of the sim.
defaults.schedulingPolicy = 'round-robin';  % 'round-robin' | 'pf-classic' | 'pf-aoi' | 'risk-aware' | 'hybrid'
defaults.sched_alpha      = 0.5;            % hybrid only: weight between PF-AoI and Risk-Aware

defaults.numDrones       = 20;
defaults.corridorLength  = 1800;
defaults.flightTime      = 25;
defaults.updateRate      = 5;
defaults.dronesPerSlot   = 5;
defaults.droneEastPos    = 100;
defaults.minStartDelay   = 20;
defaults.maxStartDelay   = 50;
defaults.fc              = 3.5e9;
defaults.pTransmitDrone  = 23;
defaults.gNB_Gain        = 8;
defaults.noiseFigure     = 6;
defaults.bw              = 20e6;
defaults.thresholdSNR    = 10;
defaults.radioDelay      = 5;
defaults.coreDelay       = 10;
defaults.videoDelay      = 11;
defaults.w_unc           = 0.4;
defaults.w_map           = 0.4;
defaults.w_vid           = 0.2;
defaults.r_min           = 25;
defaults.k_aoi           = 1;
defaults.tau_max         = 500;
defaults.d_crit          = 150;
defaults.R_bar_sys       = 1e5;
defaults.microBSPos      = [0 -1600 -30; 0 0 -30; 0 1600 -30];
defaults.masterPos       = [-600 0 -40];
defaults.numHotspots     = 12;
defaults.rng_seed        = 42;
defaults.manualHotspots  = [];

fields = fieldnames(defaults);
for i = 1:numel(fields)
    f = fields{i};
    if ~isfield(cfg, f)
        cfg.(f) = defaults.(f);
        warning('UAM:config', 'cfg.%s not set — using default: %s', ...
                f, mat2str(defaults.(f)));
    end
end

% Derived / validated fields
cfg.speedVal      = cfg.corridorLength / cfg.flightTime;
cfg.latency_base  = cfg.radioDelay + cfg.coreDelay + cfg.videoDelay;
cfg.thermalNoise  = -174 + 10*log10(cfg.bw);
cfg.v_max         = cfg.speedVal;

% Weight sanity check — weights are scale coefficients, NOT probabilities.
% They do not need to sum to 1. Only flag negative values (unphysical).
if any([cfg.w_unc, cfg.w_map, cfg.w_vid] < 0)
    warning('UAM:config', ...
        'Negative risk weight detected (w_unc=%.3g w_map=%.3g w_vid=%.3g).', ...
        cfg.w_unc, cfg.w_map, cfg.w_vid);
end

end