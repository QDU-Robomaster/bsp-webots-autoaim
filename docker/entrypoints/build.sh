#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${repo_root}"
bash docker/entrypoints/prepare.sh

python3 -m xrobot.GenerateMain --output User/xrobot_main.hpp --config User/xrobot.yaml
build_dir="${XR_BUILD_DIR:-${repo_root}/build}"
openvino_dir="${OpenVINO_DIR:-}"
if [[ -z "${openvino_dir}" ]]; then
  for candidate in /opt/intel/openvino_2025/runtime/cmake /opt/intel/openvino_*/runtime/cmake; do
    if [[ -f "${candidate}/OpenVINOConfig.cmake" ]]; then
      openvino_dir="${candidate}"
      break
    fi
  done
fi
cmake_args=(-S "${repo_root}" -B "${build_dir}" -G Ninja
  -DCMAKE_BUILD_TYPE="${XR_BUILD_TYPE:-Release}"
  -DAUTO_AIM_PREVIEW_IMAGE="${AUTO_AIM_PREVIEW_IMAGE:-1}"
  -DAUTO_AIM_BUILD_ACCEPTANCE="${XR_BUILD_ACCEPTANCE:-OFF}")
if [[ -n "${openvino_dir}" ]]; then
  cmake_args+=(-DOpenVINO_DIR="${openvino_dir}")
fi
cmake "${cmake_args[@]}"
targets=(rm_auto_aim)
if [[ "${XR_BUILD_ACCEPTANCE:-OFF}" == "ON" || "${XR_BUILD_ACCEPTANCE:-OFF}" == "1" ]]; then
  targets+=(rm_auto_aim_acceptance)
fi
cmake --build "${build_dir}" -j"${XR_BUILD_JOBS:-4}" --target "${targets[@]}"
