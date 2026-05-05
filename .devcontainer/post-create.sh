#!/usr/bin/env bash
set -euo pipefail

repo_root=/workspace

git config --global --add safe.directory "${repo_root}"

cd "${repo_root}"

if ! git -C libxr rev-parse --verify HEAD >/dev/null 2>&1; then
  git submodule deinit -f libxr || true
  rm -rf libxr .git/modules/libxr
fi

git submodule sync --recursive
git submodule update --init --recursive --force
libxr_commit=4c06409f482250e231879a83fe406d8387011fff
git -C libxr cat-file -e "${libxr_commit}^{commit}" 2>/dev/null ||
  git -C libxr fetch origin "${libxr_commit}"
git -C libxr checkout --detach "${libxr_commit}"
