#!/usr/bin/env bash
set -euo pipefail

PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# shellcheck disable=SC1091
source "${PACKAGE_DIR}/scripts/python_venv_setup.sh"

python3 "${PACKAGE_DIR}/model/model_test.py" "$@"
