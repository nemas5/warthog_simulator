#!/usr/bin/env bash
set -euo pipefail

PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_DIR="$(cd "${PACKAGE_DIR}/.." && pwd)"
VENV_DIR="${PACKAGE_DIR}/.venv"
REQUIREMENTS="${PACKAGE_DIR}/requirements.txt"
REQUIREMENTS_STAMP="${VENV_DIR}/.requirements.sha256"
URDF_EXTRAS_PATH="${PACKAGE_DIR}/urdf/warthog_ground_truth.urdf.xacro"

SPAWN_X="0.0"
SPAWN_Y="0.0"
SPAWN_Z="0.5"
GUI="false"
TRAJECTORY_TYPE="circle"
TRAJECTORY_DURATION_SEC="1800"
TRAJECTORY_PUBLISH_RATE="20"
RUN_PARSER="true"

usage() {
  cat <<EOF
Usage: $(basename "$0") TARGET [options]

TARGET is a name from src/worlds_library:
  <TARGET>/<TARGET>.world       run one world
  <TARGET>/*/*.world            run every world in a set

Options:
  --x VALUE                   Spawn X coordinate (default: ${SPAWN_X})
  --y VALUE                   Spawn Y coordinate (default: ${SPAWN_Y})
  --z VALUE                   Spawn Z coordinate (default: ${SPAWN_Z})
  --gui                       Run Gazebo with GUI
  --trajectory-type NAME      circle, segments, or interpolated
                              (default: ${TRAJECTORY_TYPE})
  --duration-sec VALUE        Duration per world in seconds (default: ${TRAJECTORY_DURATION_SEC})
  --rate VALUE                Command publish rate in Hz (default: ${TRAJECTORY_PUBLISH_RATE})
  --no-parse                  Do not convert recorded bags to CSV
  -h, --help                  Show this help

Examples:
  $(basename "$0") world_example
  $(basename "$0") basis_example --duration-sec 300 --trajectory-type circle
EOF
}

die() {
  echo "$*" >&2
  exit 1
}

require_value() {
  if [ "$#" -lt 2 ] || [ -z "$2" ]; then
    echo "Option $1 requires a value" >&2
    exit 2
  fi
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

requirements_changed() {
  [ ! -f "${REQUIREMENTS_STAMP}" ] && return 0
  command -v sha256sum >/dev/null 2>&1 || return 1

  local current_hash stored_hash
  current_hash="$(sha256sum "${REQUIREMENTS}" | awk '{print $1}')"
  stored_hash="$(awk '{print $1}' "${REQUIREMENTS_STAMP}")"
  [ "${current_hash}" != "${stored_hash}" ]
}

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

venv_is_usable() {
  [ -x "${VENV_DIR}/bin/python3" ] || return 1

  local configured_version runtime_version site_packages
  configured_version="$(venv_config_major_minor)" || return 1
  runtime_version="$(python_major_minor "${VENV_DIR}/bin/python3")" || return 1
  [ -n "${configured_version}" ] && [ "${configured_version}" = "${runtime_version}" ] || return 1

  site_packages="$(venv_site_packages)" || return 1
  [ -d "${site_packages}" ] || return 1

  "${VENV_DIR}/bin/python3" -m pip --version >/dev/null 2>&1
}

prepend_venv_pythonpath() {
  local site_packages
  site_packages="$(venv_site_packages)" || return 0
  [ -d "${site_packages}" ] || return 0

  case ":${PYTHONPATH:-}:" in
    *":${site_packages}:"*) ;;
    *) export PYTHONPATH="${site_packages}${PYTHONPATH:+:${PYTHONPATH}}" ;;
  esac
}

prepare_environment() {
  if ! venv_is_usable || requirements_changed; then
    echo "Preparing virtual environment..."
    source_relaxed "${PACKAGE_DIR}/scripts/python_venv_setup.sh"
  else
    source_relaxed "${VENV_DIR}/bin/activate"
  fi

  if [ -f "${WORKSPACE_DIR}/devel/setup.bash" ]; then
    source_relaxed "${WORKSPACE_DIR}/devel/setup.bash"
  elif [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
    source_relaxed "${WORKSPACE_DIR}/install/setup.bash"
  fi
  prepend_venv_pythonpath

  command -v roslaunch >/dev/null 2>&1 ||
    die "roslaunch is not available. Build and source the workspace first."
  command -v timeout >/dev/null 2>&1 ||
    die "timeout is required but was not found in PATH."
}

run_world() {
  local world_path="$1"
  local bag_path="$2"
  local world_name surface_map_path status

  world_name="$(basename "${world_path}" .world)"
  surface_map_path="${world_path%.world}_surface_map.json"
  [ -f "${surface_map_path}" ] ||
    die "Surface map does not exist for ${world_name}: ${surface_map_path}"

  echo
  echo "Running ${world_name}"
  echo "  world: ${world_path}"
  echo "  bag:   ${bag_path}"

  set +e
  timeout --signal=INT --kill-after=20s "${TRAJECTORY_DURATION_SEC}s" \
    roslaunch warthog_simulator simulation.launch \
      "world_path:=${world_path}" \
      "urdf_extras_path:=${URDF_EXTRAS_PATH}" \
      "x:=${SPAWN_X}" \
      "y:=${SPAWN_Y}" \
      "z:=${SPAWN_Z}" \
      "gui:=${GUI}" \
      "surface_map_path:=${surface_map_path}" \
      "bag_path:=${bag_path}" \
      "trajectory_type:=${TRAJECTORY_TYPE}" \
      "trajectory_duration_sec:=${TRAJECTORY_DURATION_SEC}" \
      "trajectory_publish_rate:=${TRAJECTORY_PUBLISH_RATE}"
  status="$?"
  set -e

  case "${status}" in
    0|124|130) ;;
    *) die "roslaunch failed for ${world_name} with status ${status}" ;;
  esac

  [ -f "${bag_path}" ] || die "Expected bag file was not produced: ${bag_path}"
}

if [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ]; then
  usage
  exit 0
fi

if [ "$#" -eq 0 ]; then
  usage >&2
  exit 2
fi

TARGET="$1"
shift

case "${TARGET}" in
  ""|"."|".."|*/*|--*)
    echo "TARGET must be a name without slashes: ${TARGET}" >&2
    exit 2
    ;;
esac

while [ "$#" -gt 0 ]; do
  case "$1" in
    --x|--y|--z|--trajectory-type|--duration-sec|--rate)
      require_value "$@"
      case "$1" in
        --x) SPAWN_X="$2" ;;
        --y) SPAWN_Y="$2" ;;
        --z) SPAWN_Z="$2" ;;
        --trajectory-type) TRAJECTORY_TYPE="$2" ;;
        --duration-sec) TRAJECTORY_DURATION_SEC="$2" ;;
        --rate) TRAJECTORY_PUBLISH_RATE="$2" ;;
      esac
      shift 2
      ;;
    --gui)
      GUI="true"
      shift
      ;;
    --no-parse)
      RUN_PARSER="false"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

TARGET_DIR="${PACKAGE_DIR}/worlds_library/${TARGET}"
SINGLE_WORLD="${TARGET_DIR}/${TARGET}.world"
RUN_ID="$(date +%Y%m%d_%H%M%S)"

if [ -f "${SINGLE_WORLD}" ]; then
  MODE="world"
  WORLD_FILES=("${SINGLE_WORLD}")
  BAG_DIR="${WORKSPACE_DIR}/bags"
  DATASET_DIR="${WORKSPACE_DIR}/datasets"
elif [ -d "${TARGET_DIR}" ]; then
  MODE="set"
  mapfile -t WORLD_FILES < <(
    find "${TARGET_DIR}" -mindepth 2 -maxdepth 2 -type f -name "*.world" | sort
  )
  [ "${#WORLD_FILES[@]}" -gt 0 ] ||
    die "No world or world set found for '${TARGET}' in ${TARGET_DIR}"
  BAG_DIR="${WORKSPACE_DIR}/bags/${TARGET}"
  DATASET_DIR="${WORKSPACE_DIR}/datasets/${TARGET}"
else
  die "World target does not exist: ${TARGET_DIR}"
fi

[ -f "${URDF_EXTRAS_PATH}" ] ||
  die "URDF extras file does not exist: ${URDF_EXTRAS_PATH}"
export URDF_EXTRAS="${URDF_EXTRAS_PATH}"
export WARTHOG_URDF_EXTRAS="${URDF_EXTRAS_PATH}"
export GAZEBO_MODEL_PATH="${PACKAGE_DIR}${GAZEBO_MODEL_PATH:+:${GAZEBO_MODEL_PATH}}"

prepare_environment
mkdir -p "${BAG_DIR}" "${DATASET_DIR}"

echo "Starting Warthog simulation:"
echo "  target:     ${TARGET} (${MODE})"
echo "  worlds:     ${#WORLD_FILES[@]}"
echo "  trajectory: ${TRAJECTORY_TYPE}, ${TRAJECTORY_DURATION_SEC}s per world"
echo "  urdf extra: ${WARTHOG_URDF_EXTRAS}"
echo "  bags:       ${BAG_DIR}"
echo "  datasets:   ${DATASET_DIR}"

for world_path in "${WORLD_FILES[@]}"; do
  world_name="$(basename "${world_path}" .world)"
  if [ "${MODE}" = "world" ]; then
    bag_path="${BAG_DIR}/${TARGET}.bag"
  else
    bag_path="${BAG_DIR}/${world_name}_${RUN_ID}.bag"
  fi
  run_world "${world_path}" "${bag_path}"
done

if [ "${RUN_PARSER}" = "true" ]; then
  echo
  echo "Parsing recorded bags into CSV datasets..."
  python3 "${PACKAGE_DIR}/bag_parser/gazebo_dataset_parser.py" "${TARGET}"
fi

echo
echo "Simulation complete:"
echo "  bags:     ${BAG_DIR}"
echo "  datasets: ${DATASET_DIR}"
