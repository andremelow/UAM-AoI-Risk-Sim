#!/usr/bin/env Rscript
# Agrega CSVs por-run do experimento capacity_15min em um único summary.
# Rodar do raiz do projeto:
#   Rscript Dash/capacity-lite/make_data.R

suppressMessages({
  library(dplyr)
  library(readr)
  library(purrr)
  library(stringr)
})

SIM_DIRS <- c(
  "uam-sim-new/csv_export/capacity_15min_sw_nsw_assimetrico"
)
OUT_FILE <- "Dash/capacity-lite/data/capacity_summary.csv"

manifests <- unlist(lapply(SIM_DIRS, function(d) {
  list.files(d, pattern = "_manifest\\.csv$", recursive = TRUE, full.names = TRUE)
}))
# Remove duplicates (same run appearing in both dirs)
manifests <- unique(manifests)
cat(sprintf("[make_data] %d manifests encontrados em %s\n",
            length(manifests), paste(SIM_DIRS, collapse = " + ")))

process <- function(mf_path) {
  ds_path <- sub("_manifest\\.csv$", "_drone_summary.csv", mf_path)
  if (!file.exists(ds_path)) return(NULL)

  mf <- tryCatch(read_csv(mf_path, show_col_types = FALSE), error = function(e) NULL)
  ds <- tryCatch(read_csv(ds_path, show_col_types = FALSE), error = function(e) NULL)
  if (is.null(mf) || is.null(ds) || nrow(ds) == 0) return(NULL)

  # K e scenario vêm do path (não do manifest)
  parts  <- str_split(mf_path, "/")[[1]]
  k_tag  <- parts[str_detect(parts, "^K[0-9]+$")]
  sc_tag <- parts[parts %in% c("1r_static","1r_astar","2r_static","2r_astar")]
  if (!length(k_tag) || !length(sc_tag)) return(NULL)

  vid_ok   <- sum(ds$slots_vid_ok,   na.rm = TRUE)
  vid_fail <- sum(ds$slots_vid_fail, na.rm = TRUE)

  exp_tag <- {
    m <- str_match(mf_path, "csv_export/([^/]+)/")
    if (!is.na(m[1, 2])) m[1, 2] else "unknown"
  }

  # run_id extraído do nome do arquivo (ex: policy_N6_20260616_104300)
  run_id <- str_extract(basename(mf_path), "[0-9]{8}_[0-9]{6}")

  tibble(
    experiment       = exp_tag,
    run_id           = run_id,
    K                = as.integer(str_extract(k_tag, "[0-9]+")),
    C                = as.integer(mf$corridor_capacity[1]),
    scenario         = sc_tag,
    routing_mode     = mf$routing_mode[1],
    num_routes       = as.integer(mf$num_routes[1]),
    policy           = mf$policy[1],
    n_drones         = nrow(ds),
    mean_h1          = mean(ds$mean_h1,    na.rm = TRUE),
    mean_h2          = mean(ds$mean_h2,    na.rm = TRUE),
    mean_risk        = mean(ds$mean_r_tot, na.rm = TRUE),
    frames_per_drone = sum(ds$frames_delivered, na.rm = TRUE) / nrow(ds),
    channel_util     = vid_ok / max(vid_ok + vid_fail, 1L)
  )
}

rows <- map(manifests, possibly(process, NULL))
df   <- bind_rows(compact(rows))

if (nrow(df) == 0) stop("Nenhum dado agregado — verifique SIM_DIR.")

# Manter apenas o run mais recente por (experiment, scenario, K, C, policy)
df <- df |>
  arrange(experiment, scenario, K, C, policy, desc(run_id)) |>
  distinct(experiment, scenario, K, C, policy, .keep_all = TRUE) |>
  select(-run_id) |>
  arrange(experiment, scenario, K, C, policy)

dir.create(dirname(OUT_FILE), recursive = TRUE, showWarnings = FALSE)
write_csv(df, OUT_FILE)
cat(sprintf("[make_data] %d linhas escritas → %s\n", nrow(df), OUT_FILE))
cat(sprintf("  Cenários: %s\n",  paste(unique(df$scenario), collapse = " | ")))
cat(sprintf("  K values: %s\n",  paste(sort(unique(df$K)),  collapse = " "))  )
cat(sprintf("  C range:  %d–%d\n", min(df$C), max(df$C)))
