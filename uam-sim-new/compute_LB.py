"""compute_LB.py -- lower bound on the linear risk surrogate Ĵ.

PROVISIONAL — MUST BE RECONCILED WITH THE PAPER'S THEOREM 1 BEFORE CAMERA-READY.

Same Cauchy–Schwarz closed form as util/compute_lower_bound_doc.m, but with the
flow count set to N_eff = corridorCapacity (the number of UAVs AIRBORNE per slot in
the relay/capacity-drain scenario), NOT cfg.numDrones (total drones over the whole
run ≈ relay_laps·C). The original code uses cfg.numDrones, which inflates the bound
by ≈(N_tot/C)² and makes it exceed the achieved surrogate (validity gate failure).
The codebase already flags N_eff=corridorCapacity for scheduling-theory bounds
(validate_uam_config.m:209-215).

  L_B = Θ²/(2K) + C·(n_s + n_v),   Θ = C·√(n_s·L_s/p_s) + C·√(n_v·L_v/p_v)
  (uniform priorities ω_u = 1; C2 length L_s = 1; video length L_v = L).
"""
import numpy as np


def compute_LB(C, n_s, n_v, p_c2, p_vid, L_v, K, L_s=1):
    Theta = C * np.sqrt(n_s * L_s / p_c2) + C * np.sqrt(n_v * L_v / p_vid)
    return Theta ** 2 / (2.0 * K) + C * (n_s + n_v)
