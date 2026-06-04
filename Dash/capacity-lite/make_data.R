#!/usr/bin/env Rscript
# Agrega CSVs por-run do experimento capacity_5min em um único summary.
# Rodar do raiz do projeto:
#   Rscript Dash/capacity-lite/make_data.R

suppressMessages({
  library(dplyr)
  library(readr)
  library(purrr)
  library(stringr)
})

SIM_DIR  <- "uam-sim-new/csv_export/capacity_5min"
OUT_FILE <- "Dash/capacity-lite/data/capacity_summary.csv"

manifests <- list.files(SIM_DIR, pattern = "_manifest\\.csv$",
                        recursive = TRUE, full.names = TRUE)
cat(sprintf("[make_data] %d manifests encontrados em %s\n", length(manifests), SIM_DIR))

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

  tibble(
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

df <- df |>
  arrange(scenario, K, C, policy)

dir.create(dirname(OUT_FILE), recursive = TRUE, showWarnings = FALSE)
write_csv(df, OUT_FILE)
cat(sprintf("[make_data] %d linhas escritas → %s\n", nrow(df), OUT_FILE))
cat(sprintf("  Cenários: %s\n",  paste(unique(df$scenario), collapse = " | ")))
cat(sprintf("  K values: %s\n",  paste(sort(unique(df$K)),  collapse = " "))  )
cat(sprintf("  C range:  %d–%d\n", min(df$C), max(df$C)))
