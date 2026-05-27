# Doc-Alignment Patch — Notes for Andre

This patch upgrades the simulator to match Sec. 3–6 of the analysis document
(`System model + Risk definition and problem formulation + Scheduling policy
+ Performance guarantee`). All four phases were implemented; the analytic
sanity tests pass when verified independently in Python.

---

## Files added

| File | Purpose |
|---|---|
| `util/compute_lower_bound_doc.m` | Cauchy–Schwarz closed-form `J_LB` from Sec. 5 |
| `policies/policy_max_weight_doc.m` | Doc-aligned Max-Weight (`W_{u,s}`, `W_{u,v}^start`) |
| `test_sanity_doc.m` | Standalone numerical test suite (6 tests, no uavScenario needed) |
| `compare_policies.m` | Side-by-side runner for all 4 policies with `J_LB` benchmark |
| `DOC_ALIGNMENT_NOTES.md` | This file |

## Files modified

| File | Change |
|---|---|
| `config/validate_uam_config.m` | Computes `n_s`, `n_v`, `H_max`, `q_LB_s`, `q_LB_v`, `qbar_s`, `qbar_v`, adds `V_s`, `V_v`, `kappa`, `omega`, `use_normalization` |
| `core/init_state.m` | Adds debt queues `x_s`, `x_v`, `z_u` storage, `riskTot`/`riskHat` series, `JhatTime`/`JhatData`, `zuVal`, `xsVal`, `xvVal` |
| `core/step_risk.m` | Honors `use_normalization` flag (default `false`); stores `z_u` in state; computes `riskHat = n_s h_s + n_v z_u`; aggregates `Jhat = sum_u omega_u Rhat_u` |
| `core/step_transmit.m` | Adds Bernoulli `p_c2`/`p_vid` channel on top of SNR mask; updates debt queues per `x(t+1) = [x + qbar - mu]^+` |
| `core/schedule_sources.m` | Enforces `K_free = K - sum 1{r_u>0}`; reserves channels for ongoing video; routes `'max-weight'` to new policy |
| `util/collect_results.m` | Surfaces `Jhat_emp`, `J_LB`, `rho_emp`, `q_emp_s`, `q_emp_v`; prints summary |

---

## What the new objects mean

```
n_s = w_Nav * (N-1) * v_max + w_Ground * rho_0 * pi * (2 r_min v_max + v_max^2 H_max)
n_v = w_vid / H_nom

Rhat_u(t)   = n_s * h_{u,s}(t) + n_v * z_u(t)
Jhat(t)     = sum_u omega_u Rhat_u(t)
Jhat_emp    = time-average of Jhat (warmup-trimmed)

J_LB        = Theta^2/(2K) + sum_u omega_u (n_s + n_v)
              with Theta = sum sqrt(omega n_s / p_s) + sum sqrt(omega n_v L / p_v)

rho_emp     = Jhat_emp / J_LB         <- optimality ratio for that run

q_LB_s(u)   = K * sqrt(omega_u n_s p_s) / Theta    target throughput for status
q_LB_v(u)   = K * sqrt(omega_u n_v L p_v) / Theta  target throughput for video
qbar_s, qbar_v = q_LB - epsilon       used by the debt queues

x_s(u,t), x_v(u,t)  -> debt queues; should remain bounded under Max-Weight
```

---

## Sanity test — what to run

### Step 1: standalone analytic checks (no uavScenario needed)

In MATLAB, from `uam-sim-new/`:
```matlab
test_sanity_doc
```

Expected output:
```
[Test 1] Surrogate coefficients n_s, n_v
  PASS: n_s = 2.032630e-01 ...
  PASS: n_v = 5.056180e-03 ...

[Test 2] Surrogate is an UPPER bound on R_u
  PASS: max violation = -6.004e+01 (always negative)

[Test 3] KKT optimality of q_LB
  PASS: sum(q_LB/p) = 2.000000e+00 (= K)
  PASS: q_LB_s uniform across UAVs
  PASS: q_LB_v uniform across UAVs

[Test 4] Closed-form J_LB matches constrained min
  PASS: J_LB = 2.195456e+00 = J_brute

[Test 5] Debt-queue fluid stability
  PASS: queue bounded (mean ~11)

[Test 6] Max-Weight policy decision
  PASS: chose C2(drone1)=1 and Vid(drone2)=4

  Results: 8 passed, 0 failed
```

I verified all six tests numerically in Python before delivering. If any
fail in MATLAB, it indicates a porting issue (likely an indexing or
data-type mismatch) — flag and we'll fix it.

### Step 2: integration check (full simulator)

```matlab
cfg = build_scenario_config();
cfg.schedulingPolicy = 'max-weight';
cfg.numDrones = 4;
cfg.dronesPerSlot = 2;
res = run_sim(cfg);
```

In the printed summary, look for:
- `rho` should be **finite and ≥ 1** (theoretical lower bound).
- For the `max-weight` policy, `rho` should be the **lowest** among the
  four policies (the whole point of Sec. 6's optimality-ratio argument).
- `q_emp_s` and `q_emp_v` should be **close to but ≥** the corresponding
  `qbar_s`, `qbar_v` for stability (Sec. 6 Step 4).
- `x_s` and `x_v` final values should be **bounded** (no monotonic growth);
  if either grows linearly with `T`, the debt queue is unstable, which
  means `qbar` was set above the achievable region — reduce `qbar_eps`
  or increase `K`.

### Step 3: side-by-side policy comparison

```matlab
compare_policies
```

Expected ordering of `rho` (smaller is better):
```
max-weight  <  risk-aware  ~  aoi-pure  <  round-robin
```

If `max-weight` does NOT come out ahead, two likely causes:

1. **`V_s`, `V_v` mistuned.** Try `cfg.V_s = 10; cfg.V_v = 10;`
   to give debt queues more weight.
2. **`H_max` too tight.** Increase `cfg.H_max = 8 * (2*N) / (K * p_c2);`
   to widen the linear-bound headroom.

---

## Known semantic decisions

These are choices I made that you should double-check against the doc:

1. **`H_max` analytic estimate.** I used `4 * (2N) / (K * p_c2)` — i.e., 4× the
   round-robin worst-case. The doc just says `0 ≤ h_{u,s}(t) ≤ H_max` without
   prescribing a value. Configurable via `cfg.H_max`.

2. **`epsilon` for debt targets.** Set to `0.01` (Sec. 6 Step 3 says
   `epsilon → 0+`; any small positive works). Configurable via `cfg.qbar_eps`.

3. **Negative Max-Weight scores.** When `W_{u,v}^start < 0`, the strict
   reading of the drift inequality says: don't schedule that source. I
   followed the strict reading. The alternative is to fill `K_free` greedily
   even with negatives — let me know if you want a flag for that.

4. **SNR mask AND Bernoulli.** I kept the geometric SNR threshold from
   `step_channel.m` and added Bernoulli `p_c2`/`p_vid` on top. The doc
   model is purely Bernoulli; if you want pure-doc behavior, set
   `cfg.thresholdSNR = -Inf` to disable the geometric mask.

5. **Normalization off by default.** `cfg.use_normalization = false` aligns
   with Sec. 3 (raw `R_nav`, `R_gnd` as required by the bound derivation).
   Set to `true` only for visual parity with old `csv_export` runs — but
   then `n_s`, `n_v` lose their meaning.

---

## What still depends on the dashboard side (Python/R)

- `J_LB` and `rho` are now reported in `results.J_LB`, `results.rho_emp` for
  CSV export and downstream plotting.
- The empirical optimality ratio for the **paper** can be computed exactly
  as in `collect_results.m` (warmup-trimmed time-average); no further math
  needed in R/Python.

---

## What's NOT changed

- `urban_circuit_sim.m`, `urban_linear_sim.m`, `teste/dual_source_*.m` —
  monolithic legacy scripts. They predate the `state.dual` schema and don't
  use `schedule_sources.m`. I left them untouched. If you need them updated,
  flag separately — that's a larger surgery.
- `viz/init_dashboard.m`, `viz/update_dashboard.m` — they don't reference
  `z1/z2`/`x_s`/`x_v`/`Jhat`, so they keep working unchanged. To plot the
  new quantities, add panels in those files reading from
  `state.JhatData`, `state.xsVal`, `state.xvVal`.
