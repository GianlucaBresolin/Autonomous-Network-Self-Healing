#!/usr/bin/env sh
set -eu

mode="${1:-sim}"
shift || true

case "$mode" in
  sim)
    exec /project/build-docker/swarm_demo_sim1 "$@"
    ;;
  tuner)
    exec /project/build-docker/controller_tuner "$@"
    ;;
  *)
    exec "$mode" "$@"
    ;;
esac
