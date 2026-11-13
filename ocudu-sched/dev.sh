#!/bin/bash
# Run a command inside the OCUDU dev container (source bind-mounted, build dir in a docker volume).
#   ./dev.sh                      -> interactive shell
#   ./dev.sh ninja -C /build gnb  -> incremental rebuild
# Env: DEV_NAME (container name), DEV_NET (docker network, default host), DEV_TTY=0 for non-interactive.
HERE="$(cd "$(dirname "$0")" && pwd)"
TTY="-it"; { [ "${DEV_TTY:-1}" = 0 ] || [ ! -t 0 ]; } && TTY=""
exec docker run --rm $TTY --privileged --network "${DEV_NET:-host}" \
  ${DEV_NAME:+--name "$DEV_NAME"} \
  -v /dev/bus/usb:/dev/bus/usb \
  -v "$HERE":/work -v ocudu-build:/build -v ocudu-ccache:/ccache \
  -e CCACHE_DIR=/ccache -w /work \
  ocudu-dev:24.04 "${@:-bash}"
