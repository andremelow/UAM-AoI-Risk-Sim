#!/bin/bash
# Run the no-radio test-mode cell for DUR seconds and print per-RNTI UL means from the stdout metrics.
#   exp/run_testmode.sh <tag> <dur_s> [extra gnb args...]      (logs/risk/<tag>.log)
HERE="$(cd "$(dirname "$0")/.." && pwd)"; TAG=$1; DUR=$2; shift 2
DEV_TTY=0 DEV_NAME=ocudu-tm-$TAG bash "$HERE/dev.sh" timeout -s INT "$DUR" /build/apps/gnb/gnb \
  -c /work/configs/smoke_testmode.yml "$@" > "$HERE/logs/risk/$TAG.log" 2>&1
# Columns after the '|' split: UL = pusch rsrp ri mcs brate ok nok ...; skip the first 3 reports (warm-up).
awk -F'|' '/^ +1 +[0-9a-f]+ \|/ { split($1,a," "); r=a[2]; n[r]++; if (n[r]<=3) next;
     split($3,u," "); b=u[5]; m=1; if (b~/M$/) m=1e6; else if (b~/k$/) m=1e3; sub(/[Mk]$/,"",b);
     br[r]+=b*m; ok[r]+=u[6]; c[r]++ }
   END { for (r in c) tot+=ok[r]; for (r in c) printf "rnti %s  ul_brate %.2f Mbps  ul_grants/s %.1f  share %.3f\n", r, br[r]/c[r]/1e6, ok[r]/c[r], ok[r]/tot }' \
  "$HERE/logs/risk/$TAG.log" | sort
