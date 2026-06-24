#!/usr/bin/env bash

PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${PACKAGE_DIR}/.venv"
VENV_PYTHON="${VENV_DIR}/bin/python3"
REQUIREMENTS="${PACKAGE_DIR}/requirements.txt"
REQUIREMENTS_STAMP="${VENV_DIR}/.requirements.sha256"
PSPSO_BUILD_DIR="${PACKAGE_DIR}/model/PSPSO/build"
export PIP_CACHE_DIR="${VENV_DIR}/.pip-cache"

fail() {
  echo "setup.sh: $*" >&2
  return 1
}

venv_is_usable() {
  [ -x "${VENV_PYTHON}" ] &&
    "${VENV_PYTHON}" -m pip --version >/dev/null 2>&1
}

create_venv() {
  echo "Creating Python environment: ${VENV_DIR}"
  if ! python3 -m venv --clear --system-site-packages "${VENV_DIR}"; then
    cat >&2 <<EOF
setup.sh: failed to create the virtual environment.

For the standard ROS Noetic/Ubuntu container install the required packages:
  apt-get update
  apt-get install -y python3-venv python3-pip

Then run:
  source src/scripts/setup.sh
EOF
    return 1
  fi

  if ! venv_is_usable; then
    fail "the virtual environment was created without pip; install python3-venv and python3-pip"
    return 1
  fi
}

warn_if_pspso_is_incompatible() {
  [ -d "${PSPSO_BUILD_DIR}" ] || return 0

  local extension_suffix
  local import_error
  extension_suffix="$("${VENV_PYTHON}" -c \
    'import sysconfig; print(sysconfig.get_config_var("EXT_SUFFIX") or "")')"

  if [ -n "${extension_suffix}" ] &&
     [ ! -f "${PSPSO_BUILD_DIR}/pspso${extension_suffix}" ]; then
    cat >&2 <<EOF

WARNING: PSPSO is not built for this Python interpreter.
  Python: $("${VENV_PYTHON}" --version 2>&1)
  Expected: ${PSPSO_BUILD_DIR}/pspso${extension_suffix}

The existing PSPSO module must be rebuilt inside this container using the
same Python version. This does not prevent world generation or simulation,
but model/model_test.py will not be able to import pspso.
EOF
    return 0
  fi

  if ! import_error="$("${VENV_PYTHON}" -c \
    'import sys
sys.path.insert(0, sys.argv[1])
import pspso
print(pspso.__file__)' "${PSPSO_BUILD_DIR}" 2>&1)"; then
    cat >&2 <<EOF

WARNING: the PSPSO module exists but cannot be loaded:
  ${import_error}

It must be rebuilt inside this container using its Python interpreter and
system libraries. This does not prevent world generation or simulation.
EOF
  fi
}

setup_environment() {
  if ! command -v python3 >/dev/null 2>&1; then
    fail "python3 is required but was not found in PATH"
    return 1
  fi

  if ! venv_is_usable; then
    if [ -d "${VENV_DIR}" ]; then
      echo "Existing virtual environment is incomplete; recreating it."
    fi
    create_venv || return 1
  fi

  echo "Installing Python dependencies from ${REQUIREMENTS}..."
  "${VENV_PYTHON}" -m pip install --upgrade pip || return 1
  "${VENV_PYTHON}" -m pip install -r "${REQUIREMENTS}" || return 1

  if command -v sha256sum >/dev/null 2>&1; then
    sha256sum "${REQUIREMENTS}" > "${REQUIREMENTS_STAMP}"
  fi

  warn_if_pspso_is_incompatible

  # Activate only after the environment has been created and validated.
  # shellcheck disable=SC1091
  source "${VENV_DIR}/bin/activate"

  cat <<EOF
Virtual environment is ready:
  ${VENV_DIR}

Python:
  $("${VENV_PYTHON}" --version 2>&1)

Current shell is using:
  $(command -v python3)

For ROS usage in this shell, source the workspace after activating the venv:
  source <workspace>/devel/setup.bash
EOF
}

if setup_environment; then
  status=0
else
  status="$?"
fi
return "${status}" 2>/dev/null || exit "${status}"
