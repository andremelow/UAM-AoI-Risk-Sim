#!/bin/bash
set -e
cd "$(dirname "$0")"
python plot_mw_otimo.py      --dir csv_export_mw_otimo_tol
python plot_mw_otimo_risk.py --dir csv_export_mw_otimo_tol
echo "=== Plots gerados ==="
ls -lh csv_export_mw_otimo_tol/plots/*.png
