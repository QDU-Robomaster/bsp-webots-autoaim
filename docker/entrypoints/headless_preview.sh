#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
build_dir="${XR_BUILD_DIR:-${repo_root}/build}"
controller_path="${build_dir}/rm_auto_aim"
# 每次都增量构建，改过源码不会跑旧程序 / Build incrementally every time so edits are never
# run on an old binary.
bash "${repo_root}/docker/entrypoints/build.sh"
exec python3 "${repo_root}/run_headless_preview.py" \
  --repo "${repo_root}" \
  --controller "${controller_path}" \
  --runtime-sec "${XR_RUNTIME_SEC:-60}" \
  --sim-flow-rate "${XR_SIM_FLOW_RATE:-0.1}" \
  --run-root "${XR_RUN_ROOT:-${repo_root}/.docker-runs}"
