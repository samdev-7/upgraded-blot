#!/usr/bin/env bash
# Launch the Blot UI using the pipx-managed vpype venv (PySide6, QtMultimedia,
# vpype_viewer, skimage, pyobjc-AVFoundation all live there).
set -euo pipefail

HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$HERE/../.." && pwd)"
PY="$HOME/.local/pipx/venvs/vpype/bin/python"

if [[ ! -x "$PY" ]]; then
    echo "vpype pipx env not found at $PY" >&2
    echo "install: pipx install --python python3.13 vpype[all] && pipx inject vpype pyserial scikit-image pyobjc-framework-AVFoundation" >&2
    exit 1
fi

cd "$REPO_ROOT"
exec "$PY" rmrrf/ui/blot_ui.py "$@"
