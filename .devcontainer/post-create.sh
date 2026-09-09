#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Keep the checked-out libxr and local module candidates; never force an old revision.
bash "${repo_root}/docker/entrypoints/prepare.sh"
