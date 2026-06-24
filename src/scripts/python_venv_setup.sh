#!/usr/bin/env bash
set -euo pipefail

PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${PACKAGE_DIR}/.venv"
REQUIREMENTS="${PACKAGE_DIR}/requirements.txt"
REQUIREMENTS_STAMP="${VENV_DIR}/.requirements.sha256"

python_major_minor() {
  "$1" -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")'
}

venv_config_major_minor() {
  [ -f "${VENV_DIR}/pyvenv.cfg" ] || return 1
  awk -F' = ' '$1 == "version" { split($2, version, "."); print version[1] "." version[2]; exit }' \
    "${VENV_DIR}/pyvenv.cfg"
}

venv_site_packages() {
  "${VENV_DIR}/bin/python3" -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])'
}

venv_needs_recreate() {
  [ -x "${VENV_DIR}/bin/python3" ] || return 0

  local configured_version runtime_version site_packages
  configured_version="$(venv_config_major_minor)" || return 0
  runtime_version="$(python_major_minor "${VENV_DIR}/bin/python3")" || return 0
  [ -n "${configured_version}" ] && [ "${configured_version}" = "${runtime_version}" ] || return 0

  site_packages="$(venv_site_packages)" || return 0
  [ -d "${site_packages}" ] || return 0

  "${VENV_DIR}/bin/python3" -m pip --version >/dev/null 2>&1 || return 0
  return 1
}

requirements_changed() {
  [ ! -f "${REQUIREMENTS_STAMP}" ] && return 0
  command -v sha256sum >/dev/null 2>&1 || return 1

  local current_hash stored_hash
  current_hash="$(sha256sum "${REQUIREMENTS}" | awk '{print $1}')"
  stored_hash="$(awk '{print $1}' "${REQUIREMENTS_STAMP}")"
  [ "${current_hash}" != "${stored_hash}" ]
}

source_relaxed() {
  local source_path="$1"
  local status

  set +u
  # shellcheck disable=SC1090
  source "${source_path}"
  status="$?"
  set -u
  return "${status}"
}

if ! command -v python3 >/dev/null 2>&1; then
  echo "python3 is required but was not found in PATH" >&2
  return 1 2>/dev/null || exit 1
fi

if venv_needs_recreate; then
  echo "Creating virtual environment: ${VENV_DIR}"
  python3 -m venv --clear --system-site-packages "${VENV_DIR}"
  SHOULD_INSTALL="true"
elif requirements_changed; then
  SHOULD_INSTALL="true"
else
  SHOULD_INSTALL="false"
fi

source_relaxed "${VENV_DIR}/bin/activate"

if [ "${SHOULD_INSTALL}" = "true" ]; then
  python3 -m pip install --upgrade pip
  python3 -m pip install -r "${REQUIREMENTS}"

  if command -v sha256sum >/dev/null 2>&1; then
    sha256sum "${REQUIREMENTS}" > "${REQUIREMENTS_STAMP}"
  fi
fi

echo "Using virtual environment: ${VENV_DIR}"
