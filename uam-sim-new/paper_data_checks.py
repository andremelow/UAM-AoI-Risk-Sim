"""paper_data_checks.py -- shared dataset validation for the IEEE paper figures
(fig_ccdf_Ru / fig_p99_envelope). Import from the figure scripts.

Data contract (per cell dir <dir>/K{K}/C{N:02d}/<scenario>/):
  - SEEDS independent runs per policy, each a timestamped triple
    {policy}_N*_{ts}_{slot_series,drone_summary,manifest}.csv;
  - manifest one-row CSV; REQUIRED for the paper figures:
      sim_slots  >= T_REQ                    (horizon check)
      seed / rng_seed                        (seed identity; run_id fallback)
      r_tau_renorm == 1  OR  (r_min, v_max, tau_max, r_star_norm) such that
      r_star_norm == r_min + v_max*tau_max   (saturated-radius renormalisation:
      without it the R=1 envelope line is NOT TLOS-calibrated -> abort).

All validations accumulate failures and abort with a single clear report.
"""
import os, glob, hashlib, sys
import numpy as np, pandas as pd

REQUIRED_POLICIES = ["round-robin-sw", "pf-classic-sw", "mw-sw-v9", "mw-v9-nsw"]


def die(title, lines):
    sys.stderr.write(f"\n*** VALIDATION FAILED: {title} ***\n")
    for l in lines:
        sys.stderr.write(f"  - {l}\n")
    sys.stderr.write("Aborting: figures would be invalid on this dataset.\n")
    sys.exit(1)


def _manifest(path):
    m = pd.read_csv(path)
    return {c: m[c].iloc[0] for c in m.columns}


def discover_runs(data_dir, scen, K, N, policy):
    """All runs of `policy` in cell (K, N), paired by run prefix."""
    base = os.path.join(data_dir, f"K{K}", f"C{N:02d}", scen)
    runs = []
    for mf in sorted(glob.glob(os.path.join(base, f"{policy}_*_manifest.csv"))):
        prefix = mf[: -len("_manifest.csv")]
        ss = prefix + "_slot_series.csv"
        if not os.path.exists(ss):
            continue
        runs.append(dict(run_id=os.path.basename(prefix), manifest=_manifest(mf),
                         ss_path=ss))
    return runs


def _file_md5(path, blocksize=1 << 22):
    h = hashlib.md5()
    with open(path, "rb") as f:
        for blk in iter(lambda: f.read(blocksize), b""):
            h.update(blk)
    return h.hexdigest()


def validate(data_dir, scen, cells, seeds_req, t_req, policies=REQUIRED_POLICIES,
             strict_renorm=True):
    """cells = [(K, N), ...]. Returns {(K, N, policy): [runs]} on success.
    strict_renorm=False downgrades validation 3 to a WARNING (the envelope
    line must then be labelled without the TLOS-calibrated claim)."""
    problems_complete, problems_seeds, problems_renorm = [], [], []
    out = {}

    for (K, N) in cells:
        for pol in policies:
            runs = discover_runs(data_dir, scen, K, N, pol)
            # ---- 2a) byte-identical runs collapse to one (re-runs of the same
            #      seed, e.g. parfor worker-abort retries); the distinctness
            #      requirement then applies to the deduped set ----
            by_hash = {}
            for r in runs:
                by_hash[_file_md5(r["ss_path"])] = r     # keep latest per hash
            if len(by_hash) < len(runs):
                sys.stderr.write(f"  note: K={K} N={N} {pol}: "
                                 f"{len(runs) - len(by_hash)} byte-identical "
                                 f"re-run(s) collapsed\n")
            runs = sorted(by_hash.values(), key=lambda r: r["run_id"])
            # ---- 1) completeness: SEEDS distinct runs, horizon T ----
            if len(runs) < seeds_req:
                problems_complete.append(
                    f"K={K} N={N} {pol}: {len(runs)}/{seeds_req} runs")
            for r in runs:
                slots = float(r["manifest"].get("sim_slots", np.nan))
                if not np.isfinite(slots) or slots < t_req:
                    problems_complete.append(
                        f"K={K} N={N} {pol} run {r['run_id']}: "
                        f"sim_slots={slots:g} < T={t_req:g}")
            # ---- 3) r_tau renormalisation metadata ----
            for r in runs:
                m = r["manifest"]
                ok = False
                if str(m.get("r_tau_renorm", "")).strip() in ("1", "1.0", "true", "True"):
                    ok = True
                elif all(k in m for k in ("r_min", "v_max", "tau_max", "r_star_norm")):
                    want = float(m["r_min"]) + float(m["v_max"]) * float(m["tau_max"])
                    ok = np.isclose(float(m["r_star_norm"]), want, rtol=1e-6)
                if not ok:
                    problems_renorm.append(
                        f"K={K} N={N} {pol} run {r['run_id']}: manifest lacks "
                        f"r_tau_renorm flag / (r_min, v_max, tau_max, r_star_norm) "
                        f"consistent with r* = r_min + v_max*tau_max")
            out[(K, N, pol)] = runs

    # ---- 2b) distinct seed labels on the deduped set ----
    for (K, N, pol), runs in out.items():
        seeds = {str(r["manifest"].get("seed", r["manifest"].get("rng_seed", r["run_id"])))
                 for r in runs}
        if len(seeds) < min(seeds_req, len(runs)):
            problems_seeds.append(
                f"K={K} N={N} {pol}: only {len(seeds)} distinct seed labels "
                f"across {len(runs)} deduped runs")

    if problems_complete:
        die("incomplete dataset (need 4 policies x "
            f"{seeds_req} seeds, horizon T>={t_req:g} slots)", problems_complete)
    if problems_seeds:
        die("seed distinctness", problems_seeds)
    if problems_renorm:
        if strict_renorm:
            die("r_tau renormalisation not attested "
                "(R=1 envelope line would NOT be TLOS-calibrated)", problems_renorm)
        sys.stderr.write(
            f"\nWARNING: r_tau renormalisation not attested on {len(problems_renorm)} "
            "run(s); R=1 line drawn WITHOUT the TLOS-calibrated claim.\n")
    return out


def _ramp_bounds(run):
    """Per-UAV [lo, hi) slot bounds excluding the two 500 m ramps (climb-out and
    landing low-altitude outage); corridor A vs B classified by flight duration."""
    dsf = run["ss_path"].replace("_slot_series.csv", "_drone_summary.csv")
    meta = pd.read_csv(dsf, usecols=["drone_id", "entry_slot", "exit_slot"])
    meta["dur"] = meta.exit_slot - meta.entry_slot
    fr = np.where(meta.dur > meta.dur.min() * 1.5, 500.0/8000.0, 500.0/4264.4)
    meta["lo"] = meta.entry_slot + (fr * meta.dur).astype(int)
    meta["hi"] = meta.entry_slot + ((1.0 - fr) * meta.dur).astype(int)
    return meta[["drone_id", "lo", "hi"]]


def _apply_mask(df, run, exclude_ramps):
    if not exclude_ramps:
        return df
    df = df.merge(_ramp_bounds(run), on="drone_id")
    return df[(df.slot > df.lo) & (df.slot < df.hi)].drop(columns=["lo", "hi"])


def uav_stats(run, burnin, cache_dir, exclude_ramps=False):
    """Per-UAV (n, mean_r, p99_r) of R_u(t) for t > burnin; CSV-cached per run."""
    os.makedirs(cache_dir, exist_ok=True)
    tag = "_noramps" if exclude_ramps else ""
    cpath = os.path.join(cache_dir, run["run_id"] + f"_uavstats{tag}.csv")
    if os.path.exists(cpath):
        return pd.read_csv(cpath)
    df = pd.read_csv(run["ss_path"], usecols=["slot", "drone_id", "r_tot"],
                     dtype={"slot": np.int64, "drone_id": np.int32,
                            "r_tot": np.float32})
    df = df[df.slot > burnin]
    df = _apply_mask(df, run, exclude_ramps)
    g = df.groupby("drone_id").r_tot
    st = pd.DataFrame({"n": g.size(), "mean_r": g.mean(),
                       "p99_r": g.quantile(0.99)}).reset_index()
    st.to_csv(cpath, index=False)
    return st


def uav_series(run, uid, burnin, exclude_ramps=False):
    """R_u(t) samples of one UAV, t > burnin (for the CCDF pooling)."""
    df = pd.read_csv(run["ss_path"], usecols=["slot", "drone_id", "r_tot"],
                     dtype={"slot": np.int64, "drone_id": np.int32,
                            "r_tot": np.float32})
    df = df[(df.slot > burnin) & (df.drone_id == uid)]
    df = _apply_mask(df, run, exclude_ramps)
    return df.r_tot.values
