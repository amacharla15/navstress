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

## D7 — Schema written against the pydantic v1 API (Phase 1)

- SPEC asks for a pydantic schema. rosdep resolves `python3-pydantic` to the Ubuntu
  apt package, which on 24.04 is **1.10.14** (checked with `apt-cache policy` in a
  noble container). That is what CI and the pod get.
- `schema.py` imports `pydantic.v1` when it exists (pydantic 2, and 1.10.17+) and
  falls back to `pydantic` otherwise, and uses only the v1 API. The test suite passes
  unchanged on 1.10.14 / Python 3.12 and on 2.13.5 / Python 3.13, with the same
  pinned config hash.

## D8 — Clamp numbers, reject everything else; hash excludes labels (Phase 1)

- SPEC: "All numeric fields clamped to safe ranges; anything outside the allowlist
  is rejected." Clamping runs on the raw JSON before pydantic, so every clamp can be
  reported back (path, original, clamped) and logged for the LLM to see.
- Non-finite numbers, booleans or strings in numeric fields, unknown keys, unknown
  assets or override names, more than 8 obstacles and duplicate obstacle names are
  rejected rather than clamped: there is no safe value to clamp them to.
- The config hash is sha256 of canonical JSON (sorted keys, floats rounded to 1e-6,
  -0.0 folded to 0.0) **without** `scenario_id` and `family`. Renaming a scenario
  doesn't change what runs, so it shouldn't change the hash. A test pins one hash
  so an accidental canonicalization change fails loudly.
- Goals snap to the reachable goal set by `goal_id` (position from the set, yaw from
  the request if given), so the same goal always hashes the same.

## D9 — Reachable goals: our own sampler with a connectivity check (Phase 1)

- SPEC: goals "must be in a known reachable set, or sampled from free space".
  NVIDIA's `isaac_ros_navigation_goal` RandomGoalGenerator samples free cells but
  never checks that they connect to the robot, so a sealed-off free pocket counts as
  a goal. `navstress_scenarios.goals` inflates non-free cells by a robot radius,
  flood-fills from the start, and samples only inside that region, seeded.
- The default radius (0.45 m) is a judgement call between Carter's 0.25 m half-width
  and its 0.66 m circumradius; OPEN_QUESTIONS Q5.

## D10 — Obstacles are generated primitives only (Phase 1)

- `isaacsim.ros2.sim_control` spawns from a USD URI only (`resource_string` is
  ignored per its source), and asset paths for pallets in the 6.2 asset library were
  not verifiable from the sandbox. The allowlist is therefore `box` and `cylinder`,
  which the scenario manager writes as small USD files (Phase 2). A "pallet" family
  uses a pallet-sized box. Adding a real pallet asset is OPEN_QUESTIONS Q6.

## D11 — Nav2 override allowlist bounded by Carter's own limits (Phase 1)

- Allowed: `max_vel_x` [0.1, 0.8], `max_vel_theta` [0.1, 0.7],
  `local_inflation_radius` [0.1, 1.5], `global_inflation_radius` [0.1, 2.0]. The
  velocity caps equal Carter's `velocity_smoother.max_velocity` (0.8 m/s, 0.7 rad/s),
  so an override can't request more than the smoother would pass anyway.

## D12 — Results as CSV, not Parquet (Phase 1)

- SPEC allows "results.parquet (or CSV)". CSV needs no pyarrow/pandas, diffs in git,
  and opens anywhere. Rows are appended per episode so a crash loses at most one.

## D13 — LLM over plain HTTP, Gemini's OpenAI-compatible endpoint by default (Phase 1)

- The client is ~60 lines of `urllib`, so no vendor SDK is needed. Default base URL is
  `https://generativelanguage.googleapis.com/v1beta/openai/`, Google's documented
  OpenAI-compatibility endpoint. The model id is **not** defaulted (preview ids
  change); `.env.example` has a placeholder and `--list-models` (Phase 2 CLI) asks
  the endpoint. `response_format: json_object` can be turned off with
  `NAVSTRESS_LLM_JSON_MODE=0` for endpoints that reject it.
- Tests use `MockLLM` and a stub HTTP server on 127.0.0.1; no test reaches a provider.

## D14 — The code, not the LLM, decides what runs (Phase 1)

- The agent drops scenarios outside the focus family in narrowing rounds, dedupes by
  config hash, truncates to the episode budget round-robin across families, and
  caps LLM calls. A sweep is only accepted if its parameter path resolves in every
  scenario and takes at least two values.
- The boundary is reported as the bracketing pair of swept values where success
  crosses 50%, not an interpolated point: with a few episodes per value, a midpoint
  would claim precision the data doesn't have.

## D15 — Report charts: static PNG, light and dark, validated palette (Phase 1)

- Docs and the static site need images that render without JavaScript, so charts are
  matplotlib PNGs, each rendered twice (light and dark surfaces) and swapped with
  `<picture>` + `prefers-color-scheme`. Colors are a blue/orange pair with separate
  light and dark steps, checked against both surfaces for colorblind separation and
  3:1 contrast (all checks pass). "Before" is a
  ring and "after" a dot, so identity never rests on color alone. Every chart has a
  table with the same numbers next to it.

## Dependencies

One line per dependency beyond ROS core, as CLAUDE.md requires.

| Dependency | Package(s) | Why |
| --- | --- | --- |
| `ament_cmake_clang_format` | navstress_monitor (test) | Enforces the C++ formatter CLAUDE.md names. |
| `python3-pydantic` | navstress_scenarios | SPEC names pydantic for the scenario schema. |
| `python3-yaml` | navstress_scenarios | Reads map_server map YAML and goal-set files. |
| `python3-numpy` | navstress_scenarios | Occupancy-grid inflation and flood fill over whole maps. |
| `python3-pil` | navstress_scenarios | Decodes the map PNG/PGM image the same way map_server does. |
| `python3-matplotlib` | navstress_report | Renders the report charts to PNG headlessly. |
| `python3-pytest` | all Python packages (test) | Test runner colcon uses for ament_python packages. |
