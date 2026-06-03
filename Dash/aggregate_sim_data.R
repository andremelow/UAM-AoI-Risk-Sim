#!/usr/bin/env Rscript
# aggregate_sim_data.R
#
# Pre-aggregates UAM simulation CSVs into lean files for fast dashboard loading.
#
# Usage (terminal):
#   Rscript aggregate_sim_data.R <csv_dir> [--downsample 50] [--cdf-points 200]
#
# Usage (RStudio / source):
#   source("aggregate_sim_data.R")
#   aggregate_sim_data("path/to/csv_export")
#
# Output: <csv_dir>/agg/ with 5 files:
#   agg_runs.csv       — one row per run  (~KB)
#   agg_drone.csv      — one row per drone (~KB)
#   agg_manifest.csv   — scenario params   (~KB)
#   agg_risk_time.csv  — R_sys(t) 1:N downsampled (~MB)
#   agg_cdf.csv        — 200-pt CDF per run (~KB)
#
# Why: raw slot_series totals 3M+ rows (~600 MB RAM). Shiny chokes on this.
# After aggregation: ~12k rows (~1 MB). Dashboard loads in seconds.

suppressPackageStartupMessages({
  library(dplyr)
  library(readr)
  library(stringr)
  library(purrr)
  library(tidyr)
})

# ── Helper: parse policy, N, and K from filename / path ───────────────────────
parse_run_meta <- function(filename) {
  bn     <- basename(filename)
  run_id <- bn |> str_remove("_(slot_series|drone_summary|manifest)\\.csv$")
  policy <- str_split(run_id, "_N")[[1]][1]
  N      <- str_extract(run_id, "(?<=_N)\\d+") |> as.integer()
  km     <- str_extract(filename, "(?<=/K)(\\d+)/")
  K      <- if (!is.na(km)) as.integer(str_extract(km, "\\d+")) else 1L
  list(run_id = run_id, policy = policy, N = N, K = K)
}

# ── Helper: safe numeric coerce ───────────────────────────────────────────────
as_num <- function(x) suppressWarnings(as.numeric(x))

# ── Main function ─────────────────────────────────────────────────────────────
aggregate_sim_data <- function(csv_dir,
                               downsample  = 50L,
                               cdf_points  = 200L,
                               verbose     = TRUE) {

  agg_dir <- file.path(csv_dir, "agg")
  dir.create(agg_dir, showWarnings = FALSE, recursive = TRUE)

  files       <- list.files(csv_dir, pattern = "\\.csv$", full.names = TRUE, recursive = TRUE)
  # Exclude the agg/ output folder itself (avoid re-reading previous results)
  files       <- files[!str_detect(files, paste0(.Platform$file.sep, "agg", .Platform$file.sep))]
  files       <- files[!str_detect(basename(files), "^sweep_routing_summary")]
  f_manifest  <- files[str_detect(files, "_manifest\\.csv$")]
  f_summary   <- files[str_detect(files, "_drone_summary\\.csv$")]
  f_series    <- files[str_detect(files, "_slot_series\\.csv$")]

  msg <- function(...) if (verbose) message(sprintf(...))
  msg("Found: %d manifests | %d summaries | %d slot_series",
      length(f_manifest), length(f_summary), length(f_series))

  # ── 1. agg_manifest ─────────────────────────────────────────────────────────
  manifest <- map_dfr(f_manifest, function(f) {
    d <- read_csv(f, show_col_types = FALSE)
    meta <- parse_run_meta(f)
    d$policy <- meta$policy
    d$N      <- meta$N
    d$K      <- meta$K
    d
  })
  write_csv(manifest, file.path(agg_dir, "agg_manifest.csv"))
  msg("  agg_manifest.csv  : %d rows", nrow(manifest))

  # ── 2. agg_drone — drone_summary enriched ───────────────────────────────────
  # Weights from manifest (use mean across runs as fallback)
  w_unc_g <- if ("w_unc" %in% names(manifest)) mean(as_num(manifest$w_unc), na.rm = TRUE) else 0.33
  w_vid_g <- if ("w_vid" %in% names(manifest)) mean(as_num(manifest$w_vid), na.rm = TRUE) else 0.34

  drones <- map_dfr(f_summary, function(f) {
    d    <- read_csv(f, show_col_types = FALSE)
    meta <- parse_run_meta(f)
    d$run_id <- meta$run_id
    d$policy <- meta$policy
    d$N      <- meta$N
    d$K      <- meta$K

    # Join per-run weights if available
    mp    <- manifest |> filter(run_id == meta$run_id)
    w_unc <- if (nrow(mp) > 0) as_num(mp$w_unc[1]) else w_unc_g
    w_vid <- if (nrow(mp) > 0) as_num(mp$w_vid[1]) else w_vid_g

    d <- d |> mutate(
      mean_h1  = as_num(mean_h1),
      mean_h2  = as_num(mean_h2),
      flight   = as.integer(exit_slot) - as.integer(entry_slot),
      # Fallback: derive h1/h2 from slot counts when zero (pre-v27 data)
      mean_h1  = if_else(mean_h1 > 0.01, mean_h1,
                         flight / pmax(as.integer(slots_c2_ok),  1L)),
      mean_h2  = if_else(mean_h2 > 0.01, mean_h2,
                         flight / pmax(as.integer(frames_delivered), 1L)),
      ewsaoi   = w_unc * mean_h1 + w_vid * mean_h2,
      c2_share = as.integer(slots_c2_ok) /
                 pmax(as.integer(slots_c2_ok) + as.integer(slots_vid_ok), 1L)
    )
    d
  })
  write_csv(drones, file.path(agg_dir, "agg_drone.csv"))
  msg("  agg_drone.csv     : %d rows", nrow(drones))

  # ── 3. agg_runs — one row per run ───────────────────────────────────────────
  runs <- drones |>
    group_by(run_id, policy, N, K) |>
    summarise(
      n_drones         = n(),
      total_c2_ok      = sum(as.integer(slots_c2_ok)),
      total_c2_fail    = sum(as.integer(slots_c2_fail)),
      total_vid_ok     = sum(as.integer(slots_vid_ok)),
      total_vid_fail   = sum(as.integer(slots_vid_fail)),
      total_frames     = sum(as.integer(frames_delivered)),
      mean_h1          = mean(mean_h1,   na.rm = TRUE),
      mean_h2          = mean(mean_h2,   na.rm = TRUE),
      peak_h1          = max(as_num(peak_h1), na.rm = TRUE),
      peak_h2          = max(as_num(peak_h2), na.rm = TRUE),
      ewsaoi           = mean(ewsaoi,    na.rm = TRUE),
      c2_share         = mean(c2_share,  na.rm = TRUE),
      mean_r_vid       = mean(as_num(mean_r_vid), na.rm = TRUE),
      mean_r_tot       = mean(as_num(mean_r_tot), na.rm = TRUE),
      .groups          = "drop"
    ) |>
    mutate(
      c2_pct           = round(c2_share * 100, 1),
      frames_per_drone = total_frames / pmax(n_drones, 1L),
      fdr_c2           = total_c2_ok / pmax(total_c2_ok + total_c2_fail, 1L)
    )

  # Join manifest columns
  man_keep <- intersect(
    c("run_id", "L_vid", "w_coh", "corridor_length", "update_rate",
      "w_unc", "w_map", "w_vid", "sim_slots", "threshold_snr",
      "corridor_capacity", "routing_mode", "num_routes"),
    names(manifest)
  )
  runs <- left_join(runs, distinct(manifest[, man_keep], run_id, .keep_all = TRUE), by = "run_id")

  # ── Join routing scenario info from sweep_routing_summary_all.csv ────────────
  # The per-run manifest does not record scenario/routing_mode/num_routes.
  # We recover them by positional matching: within each (K, N, policy) group,
  # runs are sorted by timestamp (encoded in run_id) and matched to the fixed
  # scenario order produced by define_routing_scenarios():
  #   1 = 1-route static, 2 = 1-route A*, 3 = N-route static, 4 = N-route A*
  # Accept both "sweep_routing/" (legacy) and "sweep/" (current MATLAB output)
  sw_dir <- if (dir.exists(file.path(csv_dir, "sweep_routing")))
               file.path(csv_dir, "sweep_routing")
             else if (dir.exists(file.path(csv_dir, "sweep")))
               file.path(csv_dir, "sweep")
             else file.path(csv_dir, "sweep_routing")
  sw_path <- file.path(sw_dir, "sweep_routing_summary_all.csv")

  # Fallback: if _all.csv is missing, build it from per-directory summaries.
  # MATLAB's aggregate_summaries() writes this file at experiment end; when the
  # experiment is interrupted it may be absent even though per-(K,N) files exist.
  if (!file.exists(sw_path) && dir.exists(sw_dir)) {
    per_dir_files <- list.files(sw_dir, pattern = "^sweep_routing_summary\\.csv$",
                                full.names = TRUE, recursive = TRUE)
    per_dir_files <- per_dir_files[!grepl("sweep_routing_summary_all\\.csv$", per_dir_files)]
    if (length(per_dir_files) > 0) {
      msg("[aggregate] sweep_routing_summary_all.csv missing — building from %d per-dir files",
          length(per_dir_files))
      sw_parts <- map_dfr(per_dir_files, function(f) {
        k_match <- str_extract(f, "(?<=/K)(\\d+)/")
        K_val   <- if (!is.na(k_match)) as.integer(str_extract(k_match, "\\d+")) else NA_integer_
        d <- tryCatch(read_csv(f, show_col_types = FALSE), error = function(e) NULL)
        if (is.null(d) || nrow(d) == 0) return(NULL)
        d$K <- K_val
        d
      })
      if (!is.null(sw_parts) && nrow(sw_parts) > 0) {
        write_csv(sw_parts, sw_path)
        msg("[aggregate]   → wrote %s (%d rows)", sw_path, nrow(sw_parts))
      }
    }
  }

  if (file.exists(sw_path)) {
    sw <- tryCatch(read_csv(sw_path, show_col_types = FALSE), error = function(e) NULL)
    need <- c("K", "num_drones", "policy", "scenario", "routing_mode", "num_routes")
    if (!is.null(sw) && all(need %in% names(sw))) {
      sw_idx <- sw |>
        rename(N = num_drones) |>
        mutate(N = as.integer(N), K = as.integer(K)) |>
        group_by(K, N, policy) |>
        mutate(pos = row_number()) |>
        ungroup() |>
        select(K, N, policy, pos, scenario, routing_mode, num_routes)

      runs <- runs |>
        mutate(ts_ = str_extract(run_id, "\\d{8}_\\d{6}$")) |>
        group_by(K, N, policy) |>
        arrange(ts_, .by_group = TRUE) |>
        mutate(pos = row_number()) |>
        ungroup() |>
        left_join(sw_idx, by = c("K", "N", "policy", "pos")) |>
        select(-ts_, -pos)
    }
  }

  write_csv(runs, file.path(agg_dir, "agg_runs.csv"))
  msg("  agg_runs.csv      : %d rows", nrow(runs))

  # ── 4 & 5. slot_series — process each file once for both outputs ─────────────
  risk_time_all <- vector("list", length(f_series))
  cdf_all       <- vector("list", length(f_series))
  ds <- as.integer(downsample)
  np <- as.integer(cdf_points)

  for (fi in seq_along(f_series)) {
    f    <- f_series[fi]
    meta <- parse_run_meta(f)
    msg("  [%d/%d] %s", fi, length(f_series), basename(f))

    # Read in chunks via readr (fast C parser)
    raw <- read_csv(f, show_col_types = FALSE,
                    col_types = cols(
                      slot         = col_integer(),
                      drone_id     = col_integer(),
                      h1_slots     = col_double(),
                      h2_slots     = col_double(),
                      snr_db       = col_double(),
                      r_unc        = col_double(),
                      r_map        = col_double(),
                      r_vid        = col_double(),
                      r_tot        = col_double(),
                      source_sched = col_integer(),
                      tx_success   = col_integer()
                    ))

    # Add run_id/policy/N/K if missing
    if (!"run_id" %in% names(raw)) {
      raw$run_id <- meta$run_id
      raw$policy <- meta$policy
      raw$N      <- meta$N
      raw$K      <- meta$K
    } else if (!"K" %in% names(raw)) {
      raw$K <- meta$K
    }

    # ── 4. Risk time series: downsample + system mean across drones ─────────
    risk_time_all[[fi]] <- raw |>
      filter(slot %% ds == 0L) |>
      group_by(run_id, policy, N, K, slot) |>
      summarise(
        r_tot_mean = mean(r_tot, na.rm = TRUE),
        r_unc_mean = mean(r_unc, na.rm = TRUE),
        r_vid_mean = mean(r_vid, na.rm = TRUE),
        r_tot_max  = max(r_tot,  na.rm = TRUE),
        h1_mean    = mean(h1_slots, na.rm = TRUE),
        h2_mean    = mean(h2_slots, na.rm = TRUE),
        snr_mean   = mean(snr_db,   na.rm = TRUE),
        .groups    = "drop"
      )

    # ── 5. CDF: np equally-spaced quantile points of r_tot ──────────────────
    r_vals <- sort(raw$r_tot[raw$r_tot > 0 & !is.na(raw$r_tot)])
    n      <- length(r_vals)
    if (n > 0) {
      probs   <- seq(0, 1, length.out = np)
      cdf_all[[fi]] <- tibble(
        run_id = meta$run_id,
        policy = meta$policy,
        N      = meta$N,
        K      = meta$K,
        cdf    = probs,
        r_tot  = quantile(r_vals, probs = probs, names = FALSE)
      )
    }

    rm(raw); gc(verbose = FALSE)
  }

  risk_time <- bind_rows(risk_time_all)
  write_csv(risk_time, file.path(agg_dir, "agg_risk_time.csv"))
  msg("  agg_risk_time.csv : %d rows (1:%d downsample)", nrow(risk_time), ds)

  cdf_data <- bind_rows(cdf_all)
  write_csv(cdf_data, file.path(agg_dir, "agg_cdf.csv"))
  msg("  agg_cdf.csv       : %d rows (%d points/run)", nrow(cdf_data), np)

  # ── Summary ─────────────────────────────────────────────────────────────────
  out_files <- list.files(agg_dir, full.names = TRUE)
  msg("\nDone. Output in %s/", agg_dir)
  msg("  %-30s  %8s  %6s", "File", "Rows", "KB")
  for (f in out_files) {
    n <- nrow(read_csv(f, show_col_types = FALSE))
    msg("  %-30s  %8d  %6d", basename(f), n, file.size(f) %/% 1024)
  }

  invisible(list(
    runs      = runs,
    drones    = drones,
    manifest  = manifest,
    risk_time = risk_time,
    cdf       = cdf_data
  ))
}

# ── CLI entry point (only when run as Rscript, not when sourced) ─────────────
if (sys.nframe() == 0 && !interactive()) {
  args <- commandArgs(trailingOnly = TRUE)
  if (length(args) == 0) {
    cat("Usage: Rscript aggregate_sim_data.R <csv_dir> [--downsample N] [--cdf-points N]\n")
    quit(status = 1)
  }
  csv_dir    <- args[1]
  downsample <- 50L
  cdf_points <- 200L
  if ("--downsample"  %in% args) downsample <- as.integer(args[which(args=="--downsample") + 1])
  if ("--cdf-points"  %in% args) cdf_points <- as.integer(args[which(args=="--cdf-points") + 1])
  aggregate_sim_data(csv_dir, downsample = downsample, cdf_points = cdf_points)
}
