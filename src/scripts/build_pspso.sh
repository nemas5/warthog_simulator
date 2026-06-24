#!/usr/bin/env bash
set -euo pipefail

PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_DIR="$(cd "${PACKAGE_DIR}/.." && pwd)"
PSPSO_SOURCE_DIR="${WORKSPACE_DIR}/PSPSO"
PSPSO_BUILD_DIR="${PACKAGE_DIR}/model/PSPSO/build"
OLD_PSPSO_CMAKE_BUILD_DIR="${PACKAGE_DIR}/model/PSPSO/cmake-build"

if [ ! -f "${PSPSO_SOURCE_DIR}/CMakeLists.txt" ]; then
  echo "PSPSO source directory was not found: ${PSPSO_SOURCE_DIR}" >&2
  exit 1
fi

rm -rf "${PSPSO_BUILD_DIR}" "${OLD_PSPSO_CMAKE_BUILD_DIR}"
mkdir -p "${PSPSO_BUILD_DIR}"

cmake \
  -S "${PSPSO_SOURCE_DIR}" \
  -B "${PSPSO_BUILD_DIR}" \
  -DPython3_EXECUTABLE="$(command -v python3)" \
  -DPSPSO_ARTIFACT_DIR="${PSPSO_BUILD_DIR}"

cmake --build "${PSPSO_BUILD_DIR}"

echo "PSPSO artifacts are ready:"
echo "  ${PSPSO_BUILD_DIR}"
