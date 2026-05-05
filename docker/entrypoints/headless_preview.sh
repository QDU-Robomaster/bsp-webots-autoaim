#!/usr/bin/env bash
set -euo pipefail

repo_root=/workspace
run_root="${XR_RUN_ROOT:-${repo_root}/.docker-runs}"
runtime_sec="${XR_RUNTIME_SEC:-10}"
sim_flow_rate="${XR_SIM_FLOW_RATE:-0.1}"
controller_path="${repo_root}/build/rm_auto_aim"

if [[ ! -x "${controller_path}" ]]; then
  /bin/bash "${repo_root}/docker/entrypoints/build.sh"
fi

mkdir -p "${run_root}"

cd "${repo_root}"

python3 run_headless_preview.py \
  --repo "${repo_root}" \
  --runtime-sec "${runtime_sec}" \
  --sim-flow-rate "${sim_flow_rate}" \
  --run-root "${run_root}"
