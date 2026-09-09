#!/usr/bin/env bash
# Initialize missing dependencies without resetting an already-prepared local candidate.
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${repo_root}"
git config --global --add safe.directory "${repo_root}"

if [[ ! -f libxr/CMakeLists.txt ]]; then
  git submodule update --init --recursive
fi
if ! git -C libxr rev-parse --verify HEAD >/dev/null 2>&1; then
  echo "libxr is not an initialized checkout; preserve it and repair its checkout explicitly." >&2
  exit 2
fi

if [[ "${XR_FORCE_XROBOT_SETUP:-0}" == "1" ]]; then
  xrobot_setup
fi
python3 - <<'PY'
from pathlib import Path
import yaml

config = yaml.safe_load(Path("Modules/modules.yaml").read_text(encoding="utf-8"))
missing = []
for entry in config["modules"]:
    name = entry.split("/", 1)[1].split("@", 1)[0]
    if not (Path("Modules") / name / "CMakeLists.txt").is_file():
        missing.append(name)
if missing:
    raise SystemExit("Missing modules: " + ", ".join(missing) +
                     ". Initialize the declared dependencies explicitly; prepared local candidates are not overwritten.")
print("Dependencies present; retaining all current module worktrees.")
PY
printf 'libxr='; git -C libxr rev-parse HEAD
