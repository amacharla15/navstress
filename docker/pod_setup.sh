#!/usr/bin/env bash
# NavStress pod setup for a RunPod pod running the official Isaac Sim container
# (nvcr.io/nvidia/isaac-sim:6.2.0, Ubuntu 24.04). Installs ROS 2 Jazzy + Nav2 from the
# ROS apt repository, builds the needed IsaacSim-ros_workspaces packages and this repo
# on the persistent network volume, and writes an environment file.
#
# Safe to re-run: each step checks what is already in place. On a fresh pod with an
# existing volume the apt packages reinstall from the .deb cache on the volume, and
# the workspaces are only rebuilt when their git commit changed.
#
# Usage: bash docker/pod_setup.sh [--dry-run] [--with-claude-code] [--no-cache-links]
# Docs:  docs/RUNPOD.md

set -Eeuo pipefail

VOLUME="${NAVSTRESS_VOLUME:-/workspace}"
ROOT="${VOLUME}/navstress"
REPO_URL="${NAVSTRESS_REPO_URL:-https://github.com/amacharla15/navstress.git}"
REPO_REF="${NAVSTRESS_REPO_REF:-main}"
ISAAC_SIM_ROOT="${NAVSTRESS_ISAAC_SIM_ROOT:-/isaac-sim}"
ISAAC_SIM_VERSION="6.2.0"
ISAAC_WS_URL="https://github.com/isaac-sim/IsaacSim-ros_workspaces.git"
ISAAC_WS_TAG="IsaacSim-6.2.0"
ROS_DISTRO_NAME="jazzy"
# The container's own user (Dockerfile: useradd --uid 1234 isaac-sim). Isaac Sim and
# the ROS nodes run as this user; see docs/RUNPOD.md.
RUN_USER="isaac-sim"
RUN_UID=1234

APT_ARCHIVES="${ROOT}/apt-archives"
DOWNLOADS="${ROOT}/downloads"
ISAAC_WS_REPO="${ROOT}/IsaacSim-ros_workspaces"
ISAAC_WS="${ISAAC_WS_REPO}/jazzy_ws"
NAV_WS="${ROOT}/ws"
NAV_REPO="${NAV_WS}/src/navstress"
RESULTS="${ROOT}/results"
ENV_FILE="${ROOT}/env.sh"

# Isaac Sim's Nav2 tutorial installs navigation2 + nav2_bringup; the rest are what
# NavStress and carter_navigation need at runtime.
ROS_PACKAGES=(
  "ros-${ROS_DISTRO_NAME}-ros-base"
  ros-dev-tools
  "ros-${ROS_DISTRO_NAME}-navigation2"
  "ros-${ROS_DISTRO_NAME}-nav2-bringup"
  "ros-${ROS_DISTRO_NAME}-pointcloud-to-laserscan"
  "ros-${ROS_DISTRO_NAME}-simulation-interfaces"
  "ros-${ROS_DISTRO_NAME}-rosbag2-storage-mcap"
)
BASE_PACKAGES=(ca-certificates curl git locales software-properties-common build-essential tmux)

DRY_RUN=0
WITH_CLAUDE=0
CACHE_LINKS=1
SUDO=()

log() { printf '[pod_setup] %s\n' "$*"; }
warn() { printf '[pod_setup] WARNING: %s\n' "$*" >&2; }
die() {
  printf '[pod_setup] ERROR: %s\n' "$*" >&2
  exit 1
}
run() {
  if ((DRY_RUN)); then
    printf '[pod_setup]   would run: %s\n' "$*"
  else
    "$@"
  fi
}
as_root() { run "${SUDO[@]}" "$@"; }
apt_get() {
  # Keep downloaded .debs on the volume so a fresh container reinstalls without
  # downloading gigabytes again.
  as_root env DEBIAN_FRONTEND=noninteractive apt-get \
    -o Dir::Cache::Archives="${APT_ARCHIVES}" \
    -o APT::Keep-Downloaded-Packages=true "$@"
}
missing_debs() {
  local p
  for p in "$@"; do
    dpkg-query -W -f='${Status}' "${p}" 2>/dev/null | grep -q "install ok installed" ||
      printf '%s\n' "${p}"
  done
}
source_ros() {
  # ROS setup scripts read unset variables, so relax -u while sourcing.
  set +u
  # shellcheck source=/dev/null
  source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
  set -u
}

usage() {
  sed -n '2,14p' "$0" | sed 's/^# \{0,1\}//'
}

parse_args() {
  while (($#)); do
    case "$1" in
      --dry-run) DRY_RUN=1 ;;
      --with-claude-code) WITH_CLAUDE=1 ;;
      --no-cache-links) CACHE_LINKS=0 ;;
      -h | --help)
        usage
        exit 0
        ;;
      *) die "unknown argument: $1 (see --help)" ;;
    esac
    shift
  done
}

check_os() {
  local id version
  id="$(. /etc/os-release && echo "${ID}")"
  version="$(. /etc/os-release && echo "${VERSION_ID}")"
  # docs/DECISIONS.md D1: Jazzy on 24.04. A 22.04 image would need Humble with Isaac
  # Sim's internal ROS libraries, which this script does not implement.
  [[ "${id}" == "ubuntu" && "${version}" == "24.04" ]] ||
    die "expected Ubuntu 24.04 (Isaac Sim 6.2 container), found ${id} ${version}"
  log "OS: Ubuntu ${version}"
}

check_privileges() {
  if [[ "$(id -u)" -eq 0 ]]; then
    SUDO=()
  elif command -v sudo >/dev/null && sudo -n true 2>/dev/null; then
    SUDO=(sudo -n)
  else
    die "need root (or passwordless sudo) to install apt packages; running as $(id -un). \
See 'If the pod is not root' in docs/RUNPOD.md."
  fi
  log "privileges: $(id -un)${SUDO[*]:+ via sudo}"
}

check_volume() {
  [[ -d "${VOLUME}" ]] || die "${VOLUME} does not exist; attach the network volume (docs/RUNPOD.md)"
  [[ -w "${VOLUME}" ]] || ((${#SUDO[@]})) || die "${VOLUME} is not writable"
  local free_gb
  free_gb="$(df -Pk "${VOLUME}" | awk 'NR==2 {print int($4 / 1048576)}')"
  log "volume: ${VOLUME} (${free_gb} GB free)"
  ((free_gb >= 20)) || warn "less than 20 GB free on ${VOLUME}; the install needs ~15 GB"
  as_root mkdir -p "${ROOT}" "${APT_ARCHIVES}/partial" "${DOWNLOADS}" "${RESULTS}"
}

check_gpu_and_isaac() {
  if command -v nvidia-smi >/dev/null; then
    log "GPU: $(nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader)"
  else
    warn "nvidia-smi not found: is this a GPU pod?"
  fi
  if [[ -x "${ISAAC_SIM_ROOT}/isaac-sim.sh" ]]; then
    log "Isaac Sim: ${ISAAC_SIM_ROOT}"
  else
    warn "${ISAAC_SIM_ROOT}/isaac-sim.sh not found: is this the Isaac Sim ${ISAAC_SIM_VERSION} image?"
  fi
  id "${RUN_USER}" >/dev/null 2>&1 || warn "user ${RUN_USER} not found; files stay owned by $(id -un)"
}

install_base() {
  local missing
  mapfile -t missing < <(missing_debs "${BASE_PACKAGES[@]}")
  if ((${#missing[@]})); then
    log "installing base packages: ${missing[*]}"
    apt_get update
    apt_get install -y --no-install-recommends "${missing[@]}"
  else
    log "base packages: present"
  fi
  if ! locale -a 2>/dev/null | grep -qi '^en_US\.utf-\?8$'; then
    # From the ROS 2 Jazzy install guide: ROS needs a UTF-8 locale.
    as_root locale-gen en_US en_US.UTF-8
    as_root update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  fi
  export LANG=en_US.UTF-8
}

setup_ros_apt() {
  if dpkg-query -W -f='${Status}' ros2-apt-source 2>/dev/null | grep -q "install ok installed"; then
    log "ROS apt source: present"
    return
  fi
  # Steps from the ROS 2 Jazzy "Ubuntu (deb packages)" guide.
  if ! grep -qs "noble-updates" /etc/apt/sources.list.d/ubuntu.sources; then
    warn "noble-updates missing from ubuntu.sources; the ROS guide warns this can break ros-dev-tools"
  fi
  as_root add-apt-repository -y universe
  if ((DRY_RUN)); then
    log "would download and install the latest ros2-apt-source .deb from GitHub"
    return
  fi
  local version="${ROS_APT_SOURCE_VERSION:-}" codename deb url
  if [[ -z "${version}" ]]; then
    # ROS_APT_SOURCE_VERSION can be set by hand if the GitHub API is rate-limited.
    version="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest |
      grep -F '"tag_name"' | awk -F'"' '{print $4}')" ||
      die "could not reach api.github.com; set ROS_APT_SOURCE_VERSION (see docs/RUNPOD.md)"
  fi
  [[ -n "${version}" ]] || die "could not read the latest ros-apt-source release from GitHub"
  codename="$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")"
  deb="${DOWNLOADS}/ros2-apt-source_${version}.${codename}_all.deb"
  url="https://github.com/ros-infrastructure/ros-apt-source/releases/download/${version}/ros2-apt-source_${version}.${codename}_all.deb"
  if [[ ! -s "${deb}" ]]; then
    curl -fsSL -o "${deb}.part" "${url}" || die "download failed: ${url}"
    mv "${deb}.part" "${deb}"
  fi
  as_root dpkg -i "${deb}"
  log "ROS apt source: installed ${version}"
}

install_ros() {
  local missing
  mapfile -t missing < <(missing_debs "${ROS_PACKAGES[@]}")
  if ((${#missing[@]})); then
    log "installing ROS packages: ${missing[*]}"
    apt_get update
    apt_get install -y "${missing[@]}"
  else
    log "ROS 2 ${ROS_DISTRO_NAME} + Nav2: present"
  fi
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    as_root rosdep init
  fi
  if ((DRY_RUN)); then
    log "would run: rosdep update"
  else
    rosdep update --rosdistro "${ROS_DISTRO_NAME}" >/dev/null
  fi
}

clone_at() {
  # clone_at <url> <ref> <dir>: clone, or check out <ref> in an existing clean clone.
  local url="$1" ref="$2" dir="$3"
  if [[ -d "${dir}/.git" ]]; then
    if [[ -n "$(git -C "${dir}" status --porcelain)" ]]; then
      warn "${dir} has local changes; leaving it as is"
      return
    fi
    run git -C "${dir}" fetch --depth 1 origin "${ref}"
    run git -C "${dir}" checkout --quiet FETCH_HEAD
  else
    run git clone --quiet --depth 1 --branch "${ref}" "${url}" "${dir}"
  fi
}

build_if_changed() {
  # build_if_changed <name> <repo_dir> <ws_dir> <rosdep paths, space-separated> <colcon args...>
  local name="$1" repo="$2" ws="$3" paths=()
  read -r -a paths <<<"$4"
  shift 4
  local head marker="${ws}/.navstress_built_commit"
  head="$(git -C "${repo}" rev-parse HEAD 2>/dev/null || echo unknown)"
  if [[ -f "${ws}/install/setup.bash" && -f "${marker}" && "$(cat "${marker}")" == "${head}" ]]; then
    log "${name}: already built at ${head:0:10}"
    return
  fi
  log "${name}: building ${head:0:10}"
  if ((DRY_RUN)); then
    log "would run in ${ws}: rosdep install + colcon build $*"
    return
  fi
  (
    cd "${ws}"
    source_ros
    rosdep install --from-paths "${paths[@]}" --ignore-src -y -r --rosdistro "${ROS_DISTRO_NAME}"
    colcon build --event-handlers console_cohesion- "$@"
  )
  echo "${head}" >"${marker}"
}

setup_isaac_ws() {
  clone_at "${ISAAC_WS_URL}" "${ISAAC_WS_TAG}" "${ISAAC_WS_REPO}"
  # Only the three packages the Nova Carter navigation sample needs; the rest of the
  # workspace (MoveIt, ros2_control submodules, humanoid demos) is not built.
  build_if_changed "IsaacSim-ros_workspaces" "${ISAAC_WS_REPO}" "${ISAAC_WS}" \
    "src/navigation/carter_navigation src/navigation/isaac_ros_navigation_goal src/isaacsim_bringup" \
    --packages-up-to carter_navigation isaacsim_bringup isaac_ros_navigation_goal
}

setup_navstress() {
  run mkdir -p "${NAV_WS}/src"
  clone_at "${REPO_URL}" "${REPO_REF}" "${NAV_REPO}"
  build_if_changed "navstress" "${NAV_REPO}" "${NAV_WS}" "src"
}

link_isaac_caches() {
  # The official container docs mount these as volumes; on RunPod the pod's container
  # disk is wiped on stop, so point them at the network volume instead. Saves the
  # shader and asset cache rebuild (minutes) on every fresh pod.
  ((CACHE_LINKS)) || return 0
  [[ -d "${ISAAC_SIM_ROOT}" ]] || return 0
  local pair src dst
  for pair in \
    ".cache:cache/main" \
    ".nv/ComputeCache:cache/computecache" \
    "kit/cache:cache/kit" \
    ".nvidia-omniverse/logs:logs" \
    ".nvidia-omniverse/config:config" \
    ".local/share/ov/data:data" \
    ".local/share/ov/pkg:pkg"; do
    src="${ISAAC_SIM_ROOT}/${pair%%:*}"
    dst="${ROOT}/isaac-sim-cache/${pair#*:}"
    link_dir "${src}" "${dst}"
  done
  link_dir /var/cache/hub "${ROOT}/isaac-sim-cache/hub"
}

link_dir() {
  local src="$1" dst="$2"
  if [[ -L "${src}" && "$(readlink -f "${src}")" == "$(readlink -f "${dst}")" ]]; then
    return
  fi
  as_root mkdir -p "${dst}" "$(dirname "${src}")"
  if [[ -d "${src}" && ! -L "${src}" ]]; then
    as_root cp -a -n "${src}/." "${dst}/"
    as_root rm -rf "${src}"
  fi
  as_root ln -sfn "${dst}" "${src}"
  ((DRY_RUN)) || log "cache: ${src} -> ${dst}"
}

write_env() {
  local tmp
  tmp="$(mktemp)"
  cat >"${tmp}" <<EOF
# Generated by docker/pod_setup.sh. Source this in every pod shell.
set +u
source /opt/ros/${ROS_DISTRO_NAME}/setup.bash
source ${ISAAC_WS}/install/setup.bash
source ${NAV_WS}/install/setup.bash
export NAVSTRESS_RESULTS_DIR=${RESULTS}
export NAVSTRESS_ISAAC_SIM_VERSION=${ISAAC_SIM_VERSION}
export NAVSTRESS_ISAAC_SIM_ROOT=${ISAAC_SIM_ROOT}
# Read by isaacsim_bringup's run_isaacsim.py to find the Isaac Sim install.
export isaac_sim_package_path=${ISAAC_SIM_ROOT}
# The owner accepts the Isaac Sim EULA by setting ACCEPT_EULA=Y on the pod; Kit
# needs it in this form when isaac-sim.sh is launched directly.
if [ "\${ACCEPT_EULA:-}" = "Y" ]; then export OMNI_KIT_ACCEPT_EULA=YES; fi
cd ${NAV_REPO}
EOF
  as_root install -m 0644 "${tmp}" "${ENV_FILE}"
  rm -f "${tmp}"
  local rc home
  for home in "${HOME}" "$(getent passwd "${RUN_USER}" | cut -d: -f6)"; do
    [[ -n "${home}" && -d "${home}" ]] || continue
    rc="${home}/.bashrc"
    if ! grep -qsF "${ENV_FILE}" "${rc}"; then
      as_root sh -c "printf '\n[ -f %s ] && . %s\n' '${ENV_FILE}' '${ENV_FILE}' >> '${rc}'"
    fi
  done
  ((DRY_RUN)) || log "environment: ${ENV_FILE} (sourced from ~/.bashrc)"
}

hand_over_to_run_user() {
  # Build as root, run as the container's user: Isaac Sim 6.x is designed to run as
  # uid 1234, and running the ROS nodes as the same user avoids cross-user DDS
  # shared-memory permission problems.
  id "${RUN_USER}" >/dev/null 2>&1 || return 0
  as_root chown -R "${RUN_UID}:${RUN_UID}" "${NAV_WS}" "${ISAAC_WS_REPO}" "${RESULTS}"
  if [[ -d "${ROOT}/isaac-sim-cache" ]]; then
    as_root chown -R "${RUN_UID}:${RUN_UID}" "${ROOT}/isaac-sim-cache"
  fi
}

install_claude_code() {
  ((WITH_CLAUDE)) || return 0
  if command -v claude >/dev/null || [[ -x "${HOME}/.local/bin/claude" ]]; then
    log "Claude Code: present"
    return
  fi
  # Official native installer (code.claude.com/docs/en/setup). It installs into
  # ~/.local on the container disk, which the docs recommend over network storage.
  if ((DRY_RUN)); then
    log "would run: curl -fsSL https://claude.ai/install.sh | bash"
  else
    curl -fsSL https://claude.ai/install.sh | bash
  fi
}

summary() {
  log "----------------------------------------------------------------"
  if ((DRY_RUN)); then
    log "dry run: nothing was changed"
  else
    log "ROS: $(dpkg-query -W -f='${Version}' "ros-${ROS_DISTRO_NAME}-ros-base" 2>/dev/null || echo '?')"
    log "Nav2: $(dpkg-query -W -f='${Version}' "ros-${ROS_DISTRO_NAME}-navigation2" 2>/dev/null || echo '?')"
    log "simulation_interfaces: $(dpkg-query -W -f='${Version}' \
      "ros-${ROS_DISTRO_NAME}-simulation-interfaces" 2>/dev/null || echo '?')"
    log "IsaacSim-ros_workspaces: $(git -C "${ISAAC_WS_REPO}" describe --tags 2>/dev/null || echo '?')"
    log "navstress: $(git -C "${NAV_REPO}" rev-parse --short HEAD 2>/dev/null || echo '?')"
  fi
  log "next: source ${ENV_FILE}, then follow 'Run' in docs/RUNPOD.md"
  log "REMINDER: stop or terminate this pod when you are done (runpodctl pod stop \$RUNPOD_POD_ID)."
}

main() {
  parse_args "$@"
  check_os
  check_privileges
  check_volume
  if ((!DRY_RUN)); then
    mkdir -p "${ROOT}/logs" 2>/dev/null || as_root mkdir -p "${ROOT}/logs"
    exec > >(tee -a "${ROOT}/logs/pod_setup_$(date -u +%Y%m%dT%H%M%SZ).log") 2>&1
  fi
  check_gpu_and_isaac
  install_base
  setup_ros_apt
  install_ros
  setup_isaac_ws
  setup_navstress
  link_isaac_caches
  write_env
  hand_over_to_run_user
  install_claude_code
  summary
}

# Run only when executed, so the functions can be sourced and tested on their own.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
