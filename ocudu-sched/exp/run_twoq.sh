#!/bin/bash
# Two-source test-mode run: N emulated UEs with a periodic C2 source and a GOP video source (no radio, K=1 UL grant
# per slot). Writes logs/twoq/<tag>.{yml,log,csv} and prints the per-source AoI table.
#   exp/run_twoq.sh <tag> <dur_s> [extra gnb args, e.g. -c /work/configs/policy_risk.yml]
# Traffic parameters come from the environment (defaults below).
HERE="$(cd "$(dirname "$0")/.." && pwd)"; TAG=$1; DUR=$2; shift 2
: "${NOF_UES:=8}" "${C2_PERIOD_MS:=250}" "${C2_BYTES:=100}" "${VIDEO_FPS:=30}" "${VIDEO_GOP:=15}"
: "${VIDEO_I_BYTES:=30000}" "${VIDEO_P_BYTES:=3000}"
mkdir -p "$HERE/logs/twoq"
cat > "$HERE/logs/twoq/$TAG.yml" <<YML
test_mode:
  test_ue:
    rnti: 0x44
    nof_ues: $NOF_UES
    cqi: 15
    ri: 1
    pdsch_active: false
    pusch_active: true
    c2_video_traffic: true
    c2_period_ms: $C2_PERIOD_MS
    c2_bytes: $C2_BYTES
    video_fps: $VIDEO_FPS
    video_gop: $VIDEO_GOP
    video_i_bytes: $VIDEO_I_BYTES
    video_p_bytes: $VIDEO_P_BYTES
    traffic_log: /work/logs/twoq/$TAG.csv
YML
DEV_TTY=0 DEV_NAME=ocudu-twoq-$TAG bash "$HERE/dev.sh" timeout -s INT "$DUR" /build/apps/gnb/gnb \
  -c /work/configs/smoke_testmode.yml -c /work/configs/k1_per_slot.yml -c "/work/logs/twoq/$TAG.yml" "$@" \
  > "$HERE/logs/twoq/$TAG.log" 2>&1
python3 "$HERE/exp/analyze_twoq.py" "$HERE/logs/twoq/$TAG.csv"
