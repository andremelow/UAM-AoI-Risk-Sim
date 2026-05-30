# UAM-AoI-Risk-Sim — Modular Architecture

## File Tree (21 files)

```
uam-sim/
├── run_sim.m                  ← NEW entry point (replaces monolith)
├── setup_paths.m              ← Adds subdirectories to MATLAB path
│
├── config/                    ← MOVE existing files here
│   ├── build_scenario_config.m    (from repo root)
│   └── validate_uam_config.m     (from repo root)
│
├── core/
│   ├── init_scenario.m        ← Sections 2-3-5: scene, BS, drones
│   ├── init_state.m           ← Sections 6-7-8: comm, payload, risk buffers
│   ├── init_ground_risk.m     ← Section 4: heatmap rhoMap
│   ├── step_read_positions.m  ← Loop: read positions + clear finished
│   ├── step_traffic.m         ← Loop: AVIATOR packet generation
│   ├── step_channel.m         ← Loop: 5G NR channel model
│   ├── step_transmit.m        ← Loop: TX decision + payload/FDR + AoI
│   ├── step_risk.m            ← Loop: R_unc + R_map + R_vid + R_sys
│   └── schedule_sources.m     ← Dispatcher to policies/
│
├── policies/
│   ├── policy_round_robin.m
│   ├── policy_pf_classic.m
│   ├── policy_pf_aoi.m
│   ├── policy_risk_aware.m
│   └── policy_[removed].m
│
├── viz/
│   ├── init_dashboard.m       ← Section 9: figure layout, tiles, panels
│   ├── update_dashboard.m     ← Loop: set() calls, 3D spheres, status bar
│   └── plot_summary.m         ← Section 11: post-sim summary figure
│
└── util/
    ├── drone_status_line.m    ← Helper: format drone status panel line
    ├── collect_results.m      ← Packages state into results struct
    └── generate_arrival_times.m   ← MOVE from repo root
```

## Migration Steps

1. Copy the `uam-sim/` folder into your GitLab repo
2. Move existing files into their new locations:
   ```
   mv build_scenario_config.m  uam-sim/config/
   mv validate_uam_config.m    uam-sim/config/
   mv generate_arrival_times.m uam-sim/util/
   ```
3. Run:
   ```matlab
   cd uam-sim
   run_sim(build_scenario_config())
   ```
4. Verify output matches the monolith, then delete `UAM_AoI_Control_Sim_WithRisk_load_scnarios.m`

## Key Contracts

| Rule | Detail |
|------|--------|
| `state` is the single mutable struct | All `step_*` receive and return it |
| `cfg` is frozen after `validate_uam_config` | No function modifies it |
| `viz/` never writes `state` | `update_dashboard` is read-only |
| Policies are pure functions | Same signature: `txSlots = policy_*(activeDrones, K, state, ...)` |
| Adding a new policy | 1 file in `policies/` + 1 case in `schedule_sources.m` |
