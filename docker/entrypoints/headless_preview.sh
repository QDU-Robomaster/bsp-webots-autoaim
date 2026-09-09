#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
build_dir="${XR_BUILD_DIR:-${repo_root}/build}"
controller_path="${build_dir}/rm_auto_aim"
if [[ ! -x "${controller_path}" ]]; then
  bash "${repo_root}/docker/entrypoints/build.sh"
fi
exec python3 "${repo_root}/run_headless_preview.py" \
  --repo "${repo_root}" \
  --controller "${controller_path}" \
  --runtime-sec "${XR_RUNTIME_SEC:-10}" \
  --sim-flow-rate "${XR_SIM_FLOW_RATE:-0.1}" \
  --run-root "${XR_RUN_ROOT:-${repo_root}/.docker-runs}"
