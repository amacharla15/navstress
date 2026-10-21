# Decisions

Each entry: the decision, the evidence, and what would change it. Where official
docs or sources contradict the build spec, the docs win and the change is recorded here.

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

- Project rules: "Lint: ruff for Python, ament_lint / clang-format for C++."
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

## D16 — Our monitor is `navstress_collision_monitor` (Phase 2)

- SPEC calls the C++ node "collision_monitor". Carter's Nav2 params already run Nav2's
  own node named `collision_monitor`, and two nodes with one name break parameter
  services and logging. The package and executable keep the SPEC name; the node is
  `navstress_collision_monitor`.

## D17 — QoS, topic by topic (Phase 2)

| Topic | QoS | Why |
| --- | --- | --- |
| lidar scans/clouds, odometry (subscriptions) | Sensor Data (best effort, keep last 5) | Isaac Sim publishes sensors best effort; a reliable subscriber never matches. Best effort also matches reliable publishers. |
| noise_injector output | Sensor Data | Nav2's costmaps and AMCL subscribe with SensorDataQoS. |
| `/navstress/clearance` | best effort | A stream where only the newest value matters. |
| `/navstress/collision_events` | reliable, depth 10/50 | Rare and must not be dropped; each one fails an episode. |
| `/navstress/active_scenario` | reliable, transient local, depth 1 | Latched: a node that starts late still gets the active scenario. |
| `/initialpose` | reliable (default) | AMCL subscribes with SystemDefaultsQoS (`amcl_node.cpp`). |
| rosbag2 recorder | per-topic overrides file | Without it the recorder can subscribe RELIABLE before discovering a publisher and silently record nothing from best-effort topics. The end-to-end test caught exactly this (`/scan_raw` and `/odom` missing from bags). |

## D18 — Noise goes into what Nav2 sees; clearance comes from clean data (Phase 2)

- The noise injector sits between the sensor and Nav2. The collision monitor reads
  the clean topic, so "how close did the robot really get" doesn't degrade along with
  the perception being stress-tested.
- With both noise parameters at zero the relay forwards the same message object, so
  baseline and noise scenarios pass through an identical topology.

## D19 — ApplyScenario refuses to run something other than the spec (Phase 2)

- Order: reset → spawn → (robot placement) → noise → Nav2 overrides → announce →
  initial pose → clear costmaps → settle.
- Hard failures (episode becomes `setup_failed`, nothing runs): reset or spawn
  fails, a noise injector is unreachable, an override's Nav2 node is unreachable, or a
  scenario sets a start pose while `robot_entity` is empty. Running anyway would
  record a result for a scenario that didn't happen.
- Soft steps: costmap clearing is skipped (and logged) when the service is missing,
  because a missing service there doesn't change the scenario.
- Overrides are re-applied every scenario, with launch values (read once from the
  Nav2 node) for anything the scenario doesn't override, so no override leaks into
  the next episode.
- Robot teleport is off by default: Isaac Sim's `SetEntityState` moves a prim with an
  `XformPrim` unless the prim has `PhysicsRigidBodyAPI`, and whether that teleports an
  articulated robot correctly is unverified (OPEN_QUESTIONS Q7). Episodes start where
  `ResetSimulation` (timeline stop/play) puts the robot.

## D20 — Obstacle USD files: static colliders, kinematic bodies for movers (Phase 2)

- Static obstacles are a scaled `Cube`/`Cylinder` with `PhysicsCollisionAPI`. Moving
  ones also get `PhysicsRigidBodyAPI` with `physics:kinematicEnabled = 1`, PhysX's way
  of representing externally driven objects. Files are named by content hash and
  written atomically.
- Checked with Pixar's `usd-core` 26.8 (PyPI) in the sandbox: both variants open, have
  the intended schemas, and their bounding boxes match the requested size with the
  base on z = 0. That check found that an empty `prepend apiSchemas = []` is a USDA
  parse error, which the generator now avoids; a unit test guards it.

## D21 — Episode outcome rules (Phase 2)

- Collision beats everything: an episode with a collision event fails as
  `collision`, even if Nav2 later reports success. With `stop_on_collision` (default)
  the goal is cancelled at the first event.
- `stuck`: the robot stayed within `stuck_radius_m` for the whole of
  `stuck_window_s` (30 s on the pod). That is longer than Carter's Nav2 progress
  checker (0.5 m in 10 s), so Nav2's own recoveries get their chance first.
- `timeout`: sim-time `episode_timeout_s`, plus a wall-clock backstop
  (3 × timeout + 60 s) in case the sim clock stalls.
- `nav2_aborted`: Nav2 aborted, rejected, or cancelled the goal on its own.
- Path length integrates odometry, ignoring steps under 1 cm (sensor jitter); final
  distance uses Nav2 feedback's `current_pose` in the map frame.

## D22 — One MCAP bag per episode via `ros2 bag record` (Phase 2)

- A subprocess keeps recording off the runner's executor threads, uses the
  documented CLI (`-s mcap`, `--topics`, `--use-sim-time`,
  `--qos-profile-overrides-path`, all checked with `ros2 bag record --help` on
  Jazzy), and is stopped with SIGINT so the MCAP summary is written.

## D23 — The fake simulator and what it can't prove (Phase 2)

- `navstress_fakesim` serves the `simulation_interfaces` services NavStress uses,
  publishes a clean scan, odometry and TF, and runs a fake `NavigateToPose` server
  over a 2D world: walls, obstacles, a lidar that only sees obstacles at least 0.2 m
  tall, a planner that only avoids what that lidar sees, and a robot that can't move
  into anything. Its noise rule (abort probability rising with noise) is invented.
- It proves the wiring: services, actions, QoS, TF, bags, result files, the agent
  loop and the report all work together. It proves nothing about Nav2, Isaac Sim or
  the hypothesis. Its numbers never go into README.md or docs/RESULTS.md as findings,
  and the report labels any fakesim run with a FAKESIM banner.

## D24 — Test isolation (Phase 2)

- The C++ node test uses `ament_add_ros_isolated_gtest`; the end-to-end launch test
  reserves a `ROS_DOMAIN_ID` with `domain_coordinator` before launching. Parallel
  `colcon test` jobs on one machine therefore can't see each other's topics.

## D25 — A run id is written once (Phase 2)

- RunBatch refuses a `run_id` whose `results.csv` already exists. Appending a second
  batch to an old run would mix code versions and configs under one metadata file.

## D26 — Pod setup: apt on the container, everything else on the volume (Phase 3)

- SPEC: install ROS 2 + Nav2 + IsaacSim-ros_workspaces + NavStress "idempotently,
  onto a persistent network volume". ROS debs install under `/opt/ros`, which is on
  the container disk that RunPod wipes on stop. So the script keeps apt's download
  cache on the volume (`Dir::Cache::Archives`): a fresh pod reinstalls from local
  `.deb`s instead of downloading again. The two colcon workspaces, results and bags,
  and Isaac Sim's caches live on the volume itself.
- ROS comes from the official route in the Jazzy install guide (`ros2-apt-source`),
  and Nav2 as Isaac Sim's Nav2 tutorial installs it (`navigation2`, `nav2-bringup`).
  A conda/RoboStack prefix on the volume was considered and rejected: it would mix a
  second toolchain with Isaac Sim's and isn't what NVIDIA's tutorials use.
- Each step checks before acting (installed debs, clone present, workspace built at
  the same commit). `--dry-run` prints the plan. Tested here: shellcheck 0.9.0 and
  0.11.0 clean; dry run end to end in an Ubuntu 24.04 container; base packages, apt
  cache, locale and the non-root refusal for real; clone + colcon build + "skip if
  unchanged" for the NavStress workspace in the Jazzy container. Not testable here
  (no packages.ros.org, no GPU): the ROS install and the Isaac workspace build.

## D27 — Root to install, uid 1234 to run (Phase 3)

- The 6.2 image is built to run as `isaac-sim` (uid 1234); older forum advice to pass
  `--allow-root` to Kit predates that and is unverified for 6.2. Setup runs as root
  (apt needs it), then hands the workspaces, results and caches to uid 1234. Isaac
  Sim and all ROS nodes run as that one user, which also avoids cross-user DDS
  shared-memory permission errors.

## D28 — Headless Isaac Sim, started by the launch file (Phase 3)

- The image's entrypoint starts the WebRTC streaming app. SPEC says headless with no
  GUI streaming, so the RunPod template overrides the entrypoint to idle, and
  `isaac.launch.py` starts Isaac Sim through NVIDIA's `isaacsim_bringup` with
  `--no-window --enable isaacsim.ros2.sim_control` and the scene URL from
  `carter_navigation_isaacsim.launch.xml`.

## D29 — Nav2 inputs through the noise relays, via a params copy (Phase 3)

- `/scan` (AMCL, global costmap) is made by `pointcloud_to_laserscan`, so that node
  publishes `/scan_raw` and a relay publishes `/scan`; no Nav2 param changes. The
  local costmap's voxel layer reads `/front_3d_lidar/lidar_points` from Isaac Sim, so
  the launch writes a copy of Carter's params with only that observation topic
  renamed. On the real Carter file that changes exactly one key. The copy is named by
  content hash and its path and sha256 go into run metadata. Carter's own file stays
  untouched as the baseline config; the Phase 6 fix is passed as `nav2_params:=`.
- All runs, baseline included, go through the relays (zero noise = pass-through), so
  baseline and noisy scenarios share one topology.

## D30 — Pod-only packages are not declared in package.xml (Phase 3)

- `isaac.launch.py` needs `carter_navigation`, `isaacsim_bringup` and `nav2_bringup`.
  The first two are not released as debs, so declaring them would make CI's rosdep
  fail. The launch file looks them up at runtime. A CI test still imports and builds
  every launch description, so syntax and argument errors are caught before the pod.

## Dependencies

One line per dependency beyond ROS core, as the project rules require.

| Dependency | Package(s) | Why |
| --- | --- | --- |
| `ament_cmake_clang_format` | navstress_monitor (test) | Enforces the C++ formatter the project rules name. |
| `python3-pydantic` | navstress_scenarios | SPEC names pydantic for the scenario schema. |
| `python3-yaml` | navstress_scenarios | Reads map_server map YAML and goal-set files. |
| `python3-numpy` | navstress_scenarios | Occupancy-grid inflation and flood fill over whole maps. |
| `python3-pil` | navstress_scenarios | Decodes the map PNG/PGM image the same way map_server does. |
| `python3-matplotlib` | navstress_report | Renders the report charts to PNG headlessly. |
| `python3-pytest` | all Python packages (test) | Test runner colcon uses for ament_python packages. |
| `rclpy`, `rclcpp`, `rcl_interfaces`, `action_msgs` | nodes | ROS 2 client libraries; parameter and action status types. |
| `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `std_msgs` | nodes, interfaces | Standard poses, scans, clouds, odometry. |
| `sensor_msgs_py` | navstress_scenarios | Reads/writes PointCloud2 as NumPy for the cloud noise path. |
| `nav2_msgs` | scenarios, eval, fakesim | `NavigateToPose` action and `ClearEntireCostmap` service. |
| `simulation_interfaces` | scenarios, fakesim | The standard simulator-control API Isaac Sim implements (SPEC). |
| `tf2`, `tf2_ros`, `tf2_geometry_msgs` | navstress_monitor | Sensor-to-base transforms for clearance. |
| `tf2_ros_py` | navstress_fakesim | Python TF broadcasters for the fake robot. |
| `ros2bag`, `rosbag2_storage_mcap` | navstress_eval | Per-episode MCAP recording (SPEC). |
| `python3-yaml` | navstress_eval | Writes the rosbag2 QoS override file. |
| `ament_index_python`, `launch`, `launch_ros` | bringup, fakesim | Finding installed config; launch files. |
| `ament_cmake_gtest`, `ament_cmake_ros` | navstress_monitor (test) | gtest, and domain-isolated gtest for the node test. |
| `launch_testing`, `launch_testing_ament_cmake` | navstress_bringup (test) | The end-to-end launch test (SPEC). |
| `domain_coordinator`, `rosbag2_py` | navstress_bringup (test) | Private ROS domain for the launch test; reading recorded bags back. |
