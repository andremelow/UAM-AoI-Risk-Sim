# UAM-AoI-Risk-Sim

**Urban Air Mobility — Age of Information & Risk-Aware Scheduling Simulator**

A MATLAB simulation framework for evaluating multi-drone UAM corridor operations under configurable scheduling policies, incorporating real-time risk assessment, Age of Information (AoI) metrics, and 5G NR link modelling.

---

## Overview

This project simulates a UAM (Urban Air Mobility) corridor where a fleet of autonomous drones transmits telemetry and video data to a ground control station via a 5G NR radio access network. The simulator evaluates five scheduling policies and tracks per-drone and system-wide metrics including AoI, payload latency, risk exposure, and channel quality.

The framework is designed for research into **information-freshness-aware** and **risk-aware** radio resource management for drone fleets.

---

## Repository Structure

```
.
├── UAM_AoI_Control_Sim_WithRisk_load_scnarios.m   # Main simulation entry point
├── build_scenario_config.m                         # Scenario parameter configuration
├── validate_uam_config.m                           # Config validation & default filling
├── schedule_drones.m                               # Scheduling policy implementations
└── uam_scenario_builder_web.html                   # Web-based scenario builder UI
```

---

## Features

- **5 Scheduling Policies**
  - `round-robin` — baseline equal-time allocation
  - `pf-classic` — classic Proportional Fair (throughput-maximising)
  - `pf-aoi` — PF weighted by Age of Information
  - `risk-aware` — prioritises drones with highest combined risk × AoI
  - `hybrid` — convex combination of PF-AoI and Risk-Aware (tunable α)

- **3-Component Risk Model**
  - `R_unc` — uncertainty overlap risk (position/navigation uncertainty)
  - `R_map` — map exposure risk (ground population heatmap)
  - `R_vid` — video quality degradation risk

- **5G NR Radio Model**
  - Path loss from configurable carrier frequency (default 3.5 GHz)
  - Thermal noise floor, noise figure, configurable bandwidth
  - SNR-threshold link success/failure
  - Multi-cell micro-BS + master gNB topology

- **Metrics Collected**
  - Per-drone AoI (ms), payload latency, FDR (<500 ms)
  - Per-drone risk components over time
  - System-wide aggregated risk `R_sys(t)`
  - Exponential moving average throughput per drone

- **Web Scenario Builder** (`uam_scenario_builder_web.html`)
  - Browser-based GUI to configure all simulation parameters
  - Exports `build_scenario_config.m` ready to run

---

## Quick Start

### Requirements
- MATLAB R2025b or later (no additional toolboxes required)

### Run with default config
```matlab
UAM_AoI_Control_Sim_WithRisk(build_scenario_config())
```

### Run with parameter overrides
```matlab
cfg = build_scenario_config();
cfg.numDrones        = 30;
cfg.schedulingPolicy = 'hybrid';
cfg.sched_alpha      = 0.6;
cfg.w_unc            = 0.5;
UAM_AoI_Control_Sim_WithRisk(cfg);
```

### Use the Web Scenario Builder
Open `uam_scenario_builder_web.html` in any modern browser, configure the scenario visually, and click **Export Config** to generate a `build_scenario_config.m` file.

---

## Configuration Parameters

| Parameter | Default | Description |
|---|---|---|
| `schedulingPolicy` | `'pf-classic'` | Scheduling algorithm |
| `sched_alpha` | `0.5` | Hybrid policy weight (α) |
| `numDrones` | `20` | Number of drones in corridor |
| `corridorLength` | `1800 m` | Corridor length |
| `flightTime` | `25 s` | Transit time (sets speed) |
| `updateRate` | `5 Hz` | Telemetry update rate |
| `dronesPerSlot` | `1` | Max simultaneous transmitters |
| `fc` | `3.5 GHz` | Carrier frequency |
| `bw` | `20 MHz` | Channel bandwidth |
| `thresholdSNR` | `10 dB` | Minimum decodable SNR |
| `w_unc / w_map / w_vid` | `0.4 / 0.4 / 0.2` | Risk component weights (must sum to 1) |
| `d_crit` | `150 m` | Critical separation distance |
| `tau_max` | `500 ms` | Maximum tolerable AoI |
| `numHotspots` | `12` | Random ground risk hotspots |
| `rng_seed` | `42` | Reproducibility seed |

See `validate_uam_config.m` for the full list of parameters and their defaults.

---

## Outputs

The simulator produces a 7-panel MATLAB figure:

1. **3D Ground Risk Heatmap** with drone trajectories and AoI uncertainty spheres
2. **Uncertainty Overlap Risk** `R_unc(t)` per drone
3. **Map Exposure Risk** `R_map(t)` per drone
4. **Video-Based Risk** `R_vid(t)` per drone
5. **System-Wide Risk** `R_sys(t)` with threshold line
6. **Age of Information** (ms) per drone
7. **Payload Latency** (ms) & FDR×500 per drone

A live **Drone Status panel** shows per-drone state (position, AoI, SNR, risk) during execution.

---

## Scheduling Policy Details

### `pf-classic`
Maximises long-run fairness by scheduling the drone with the best instantaneous-to-average throughput ratio:
```
score_u = R_inst(u) / T_avg(u)
```

### `pf-aoi`
Extends PF by weighting by information staleness:
```
score_u = h_u(t) / T_avg(u)
```
where `h_u(t)` is the current Age of Information for drone `u`.

### `risk-aware`
Prioritises safety-critical updates; score combines risk and AoI:
```
score_u = R_u(t) × h_u(t)
R_u = w_unc·R_unc + w_map·R_map + w_vid·R_vid
```

### `hybrid`
Weighted blend of normalised PF-AoI and Risk-Aware scores:
```
score_u = α · score_pf_aoi(u) + (1−α) · score_risk(u)
```

---

## Contributing

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/my-policy`
3. Commit your changes: `git commit -m 'Add new scheduling policy'`
4. Push and open a Pull Request

Please ensure new scheduling policies are added to `schedule_drones.m` and documented in this README.

---

## License

This project is released for research and academic use. See `LICENSE` for details.

---

## Citation

If you use this simulator in your research, please cite:

```
@misc{uam_aoi_risk_sim,
  title  = {UAM AoI Risk-Aware Scheduling Simulator},
  year   = {2026},
  url    = {https://github.com/your-org/uam-aoi-risk-sim}
}
```
