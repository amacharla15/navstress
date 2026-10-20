# Decisions

Each entry: the decision, the evidence, and what would change it. Where official
docs or sources contradict SPEC.md, the docs win and the change is recorded here.

## D1 — ROS 2 distro: Jazzy (Phase 0)

- **Rule (SPEC §3):** Jazzy if the Isaac Sim container OS is Ubuntu 24.04; Humble with
  internal libs if it is 22.04.
- **Evidence:** `tools/docker/Dockerfile` at tag `v6.2.0` of
  `github.com/isaac-sim/IsaacSim` (the source the NGC image is built from) starts
  `FROM nvcr.io/nvidia/base/ubuntu:noble-20251013`, i.e. Ubuntu 24.04.
- **Outcome:** Jazzy. CI runs in `ubuntu:noble` with `required-ros-distributions: jazzy`.
- **Not yet verified:** the build sandbox cannot reach `nvcr.io`, so the pulled
  image itself has not been inspected. `docker/pod_setup.sh` re-checks
  `/etc/os-release` on the pod and refuses to continue on anything but 24.04.

## D2 — Isaac Sim version: 6.2.0 (Phase 0)

- SPEC says "Isaac Sim 6.x". As of 2026-10-20 the newest release tag in both
  `isaac-sim/IsaacSim` and `isaac-sim/IsaacSim-ros_workspaces` is 6.2.0
  (`IsaacSim-6.2.0`, commit `f215318`), and the ros_workspaces launch files pin
  `version: 6.2.0`. A `release/6.3` branch exists but has no release tag.

## D3 — `simulation_interfaces` 1.5.1, batch spawning via `SpawnEntities` (Phase 0)

- The Jazzy release in rosdistro is 1.5.1. Its `SpawnEntity.srv` says
  "This interface is deprecated in favour of SpawnEntities.srv".
- Isaac Sim 6.2's `isaacsim.ros2.sim_control` (extension 1.6.12) registers both,
  with no service prefix (`/spawn_entities`, `/reset_simulation`, ...), per
  `source/extensions/isaacsim.ros2.sim_control/python/impl/simulation_control.py`.
- NavStress uses `SpawnEntities`.

## D4 — Linting split: ruff for Python, ament_lint + clang-format for C++ (Phase 0)

- CLAUDE.md: "Lint: ruff for Python, ament_lint / clang-format for C++."
- Python packages therefore do not carry the `ament_flake8`/`ament_pep257` test
  wrappers (they would duplicate ruff with conflicting rules). ruff runs as its own
  CI job, pinned to 0.17.1 (latest on PyPI on 2026-10-20).
- The C++ package uses `ament_lint_auto` with `ament_lint_common`, plus
  `ament_cmake_clang_format`. `ament_cmake_uncrustify` is switched off because it
  and clang-format would reformat the same lines differently. `.clang-format` at the
  repo root is a copy of `ament_clang_format`'s bundled config, so editors and the
  linter agree.

## D5 — CI: setup-ros v0.7 + action-ros-ci v0.4 in `ubuntu:noble` (Phase 0)

- Versions are the latest tags of `ros-tooling/setup-ros` and
  `ros-tooling/action-ros-ci` on 2026-10-20; the pattern (setup-ros in a clean
  container, then action-ros-ci with `package-name`) is the one in the
  action-ros-ci README. rosdep resolves every package.xml dependency from apt.

## D6 — License: Apache-2.0 (Phase 0, assumption)

- package.xml requires a license. Apache-2.0 matches ROS 2 core and the NVIDIA
  packages NavStress builds on. Logged in OPEN_QUESTIONS.md for the owner to confirm.

## Dependencies

One line per dependency beyond ROS core, as CLAUDE.md requires.

| Dependency | Package(s) | Why |
| --- | --- | --- |
| `ament_cmake_clang_format` | navstress_monitor (test) | Enforces the C++ formatter CLAUDE.md names. |
