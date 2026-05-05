#!/usr/bin/env bash
set -euo pipefail

repo_root=/workspace
preview_image="${AUTO_AIM_PREVIEW_IMAGE:-0}"

cd "${repo_root}"

git config --global --add safe.directory "${repo_root}"
git submodule update --init --recursive

if [[ "${XR_FORCE_XROBOT_SETUP:-0}" == "1" ]]; then
  xrobot_setup
elif [[ ! -d "${repo_root}/Modules/ArmorTracker/.git" || ! -d "${repo_root}/Modules/CameraFrameSync/.git" ]]; then
  cat >&2 <<'EOF'
Required XRobot modules are not initialized.
Run xrobot_setup manually in the repository, or set XR_FORCE_XROBOT_SETUP=1 for this Docker run.
EOF
  exit 2
fi

python3 -m xrobot.GenerateMain --output User/xrobot_main.hpp --config User/xrobot.yaml
cmake -S . -B build -G Ninja -DAUTO_AIM_PREVIEW_IMAGE="${preview_image}"
cmake --build build -j"$(nproc)"
