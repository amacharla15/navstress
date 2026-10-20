# Walkthrough

Every file in the repo, in plain English: what it does, why it exists, and one
interview question about it with the answer. Grouped by the phase that added or
last changed it.

---

## Phase 0 — workspace skeleton

Build/test status for this phase: `colcon build` and `colcon test` pass for all eight
packages in a local `ros:jazzy-ros-base` (Ubuntu 24.04) container. That sandbox can't
reach packages.ros.org, so `nav2_msgs`, `simulation_interfaces` and
`ament_cmake_clang_format` were built from their `jazzy` branches there; CI installs
the released binaries through rosdep instead. GitHub Actions CI runs on every push.

### Repo root

**`SPEC.md`, `CLAUDE.md`** — The build spec and the working rules, copied in from the
project brief so they sit next to the code they govern. SPEC.md says what to build
phase by phase; CLAUDE.md says how (no invented numbers, check docs before using an
API, which linters).
*Q: Why keep the spec in the repo instead of a wiki?* — A: Reviewers can diff the
code against the exact spec revision it was built from, and the spec's own history
shows when requirements changed.

**`README.md`** — Short project pitch, a phase status table, and the local build/test
commands. It says plainly that no simulation results exist yet. Phase 7 replaces it
with the full README (architecture diagram, quickstarts, results summary).
*Q: Why put "no results yet" in the README?* — A: Rule 2 of CLAUDE.md: every number
must trace to a real run. A reader should never mistake a skeleton for findings.

**`LICENSE`** — The Apache License 2.0 text, fetched from apache.org. It matches the
`<license>` tag in every package.xml. The choice is an assumption the owner can
change (OPEN_QUESTIONS Q1).
*Q: Why does a ROS package need a license tag at all?* — A: package.xml format 3
requires one; bloom and the ROS buildfarm refuse packages without it, and users need
to know the terms before depending on the code.

**`pyproject.toml`** — ruff configuration only: 99-character lines (the ROS
convention), Python 3.12 (Jazzy on Ubuntu 24.04), and a rule set covering pyflakes,
pycodestyle, import order, bugbear and pyupgrade. Packages are still built by
colcon from their own setup.py, so this file does not make the repo pip-installable.
*Q: Why one repo-level ruff config instead of one per package?* — A: All Python
packages share one style; a single config means one place to change it, and ruff
picks it up from any subdirectory.

**`.clang-format`** — A copy of the configuration bundled with `ament_clang_format`
(Google style, 100 columns, braces on their own line for classes and functions).
`ament_cmake_clang_format` uses that same file in `colcon test`, so editor formatting
and the CI linter agree.
*Q: Why not just let uncrustify (the ament_lint_common default) format C++?* — A:
CLAUDE.md names clang-format. Running both would make them fight over the same lines,
so uncrustify is turned off in navstress_monitor's CMakeLists.

**`.gitignore`** — Ignores colcon's `build/ install/ log/`, Python caches, the local
`.env`, generated `site/`, and everything under `results/` except its README and
`.gitkeep`. Bags and per-episode files are large and machine-specific; only summaries
that docs/RESULTS.md cites get committed, with `git add -f`.
*Q: Why ignore results by default instead of committing everything?* — A: MCAP bags
are tens of MB per episode. Committing them would bloat the repo, and the summaries
plus config hashes are enough to trace every reported number.

**`.env.example`** — Placeholder environment variables for the LLM endpoint (base
URL, model id, API key), the results directory, and the Isaac Sim version recorded
in run metadata. The real `.env` is gitignored, so no secret is ever committed (rule
5).
*Q: Why an OpenAI-compatible endpoint instead of one vendor's SDK?* — A: Gemini,
OpenAI and many local servers speak the same chat-completions API, so one small HTTP
client covers all of them and the provider becomes a config value.

**`results/README.md`, `results/.gitkeep`** — Describe the per-run directory layout
(`results.csv`, `run_metadata.json`, `scenarios.jsonl`, `bags/`) and keep the
otherwise-ignored directory in git.
*Q: Why store `scenarios.jsonl` next to the results?* — A: It is the exact
validated input, so Phase 6 can rerun the identical set and match rows by config
hash.

### CI

**`.github/workflows/ci.yml`** — Two jobs. `lint` runs `ruff check` and
`ruff format --check` with ruff pinned to 0.17.1. `build_and_test` starts a clean
`ubuntu:noble` container, installs ROS 2 Jazzy with `ros-tooling/setup-ros@v0.7`,
then `ros-tooling/action-ros-ci@v0.4` runs rosdep, `colcon build` and `colcon test`
for all eight packages.
*Q: What does action-ros-ci do that a plain `colcon build` step doesn't?* — A: It
checks out the repo into a fresh workspace, runs `rosdep install` from every
package.xml (so missing dependencies fail CI), builds, tests, and fails the job on
any test failure via `colcon test-result`.

### docs/

**`docs/WALKTHROUGH.md`** — This file. It is updated after every phase with an entry
for each file added or changed.
*Q: Why write per-file explanations?* — A: It forces each file to justify its
existence and doubles as interview preparation: every design choice has a stated
reason.

**`docs/DESIGN.md`** — The architecture (who calls whom, and why the LLM never
drives) and a careful reading of the Carter Nav2 costmap config against the Nav2
Jazzy source. That reading predicts the local voxel layer only marks obstacles that
fill three 0.2 m voxel bands (about 0.4 m tall), because `mark_threshold: 2` means
"more than two voxels". It is labelled as a prediction for Phase 5 to test.
*Q: Why read the costmap source instead of trusting the YAML comments?* — A: The
YAML carries keys that the layer never declares (`voxel_size`, `obstacle_range`), and
Nav2 silently ignores unknown keys. Only the source shows which values are actually
in effect.

**`docs/DECISIONS.md`** — Each decision with its evidence: Jazzy because the Isaac
Sim 6.2.0 Dockerfile is `FROM ubuntu:noble`, Isaac Sim 6.2.0 as the latest tag,
`SpawnEntities` because `SpawnEntity` is deprecated in simulation_interfaces 1.5.1,
the linting split, the CI actions, the license. It also holds the one-line
justification for every dependency.
*Q: How was the Jazzy/Humble rule applied without pulling the 20 GB image?* — A: By
reading `tools/docker/Dockerfile` at the Isaac Sim `v6.2.0` tag, which starts
`FROM nvcr.io/nvidia/base/ubuntu:noble-...`. The pod setup script re-checks
`/etc/os-release`, because the image itself was not inspected.

**`docs/OPEN_QUESTIONS.md`** — Unresolved items, the default taken for each, and what
resolves it (license, container OS confirmation, maintainer email).
*Q: Why log questions instead of just asking?* — A: CLAUDE.md rule 7: pick a
reasonable default, record it, and keep building everything that doesn't depend on
the answer.

### src/navstress_interfaces (ament_cmake, rosidl)

**`CMakeLists.txt`, `package.xml`** — Generate C++ and Python code for the NavStress
messages, services and actions via `rosidl_generate_interfaces`, depending on
`geometry_msgs` and `std_msgs`. The package joins `rosidl_interface_packages` so
tooling knows it only holds interfaces.
*Q: Why a separate interfaces package?* — A: Both the C++ monitor and the Python
nodes depend on it. Keeping interfaces apart avoids dependency cycles and lets other
projects reuse the messages without pulling in node code.

**`msg/PlanarPose.msg`** — x, y, yaw in the map frame. Used for goals, start poses
and obstacle placement.
*Q: Why not `geometry_msgs/Pose2D`?* — A: Pose2D is deprecated in ROS 2, and a full
`Pose` with a quaternion is harder for an LLM to fill in correctly. Planar x/y/yaw
is all a warehouse floor needs.

**`msg/Obstacle.msg`** — One obstacle: name, allowlisted asset key, pose, size,
height, and optional straight-line motion (end point and speed).
*Q: Why is height a first-class field?* — A: The core hypothesis is about obstacles
too low for the costmap's sensors, so the height sweep must be expressible directly.

**`msg/SensorNoise.msg`** — Gaussian range-noise standard deviation and per-return
dropout probability, applied by the noise_injector relay.
*Q: Why inject noise in a relay instead of in Isaac Sim?* — A: SPEC keeps Isaac Sim
untouched. A relay works identically against the fake simulator and the real one,
and the clean topic stays available for the collision monitor.

**`msg/NavOverride.msg`** — One name/value pair for an allowlisted Nav2 parameter
override (e.g. `max_vel_x`).
*Q: Why name/value pairs instead of a free-form YAML blob?* — A: Each name is checked
against an allowlist with clamped ranges, so a scenario can't set arbitrary Nav2
parameters.

**`msg/ScenarioSpec.msg`** — The ROS transport form of a validated scenario: labels
(`scenario_id`, `family`), the config hash, goal (and `goal_id` from the reachable
set), optional start pose, obstacles, sensor noise, overrides, seed and timeout. The
authoritative validation lives in Python (`navstress_scenarios.schema`).
*Q: Why carry `config_hash` in the message?* — A: Every result row stores it, so any
number in a report traces back to the exact scenario configuration that produced it.

**`msg/EpisodeResult.msg`** — One episode's outcome: success, failure reason (string
constants `timeout`, `nav2_aborted`, `collision`, `stuck`, `setup_failed`), Nav2's
error code, time to goal, path length, final distance, minimum clearance,
collisions, recoveries, bag path and wall time.
*Q: Why string constants rather than an enum integer for the failure reason?* — A:
ROS 2 messages have no enum type. Strings keep CSV rows and reports readable without
a lookup table, and the constants keep the spelling consistent.

**`msg/Clearance.msg`** — Published by the C++ collision monitor: minimum clearance
to the footprint, an in-collision flag, the closest point, and robot speed.
*Q: Why include robot speed?* — A: To tell "robot drove into something" from "a
moving obstacle hit a stationary robot" when reading collision events.

**`srv/ApplyScenario.srv`** — Request a scenario; get success and a message back
once the sim is reset, populated and the robot localized.
*Q: Why is this a service but running an episode an action?* — A: Setup is a bounded
request/response. An episode runs for tens of seconds, needs progress feedback and
must be cancellable, which is what actions are for.

**`action/RunEpisode.action`** — Goal: a scenario, a run id and whether to record a
bag. Result: an EpisodeResult. Feedback: phase, elapsed time, distance remaining,
running minimum clearance.
*Q: What happens to the feedback if the client disconnects?* — A: The episode keeps
running on the server; feedback is best-effort, and the result is still written to
the run's CSV.

**`action/RunBatch.action`** — Goal: a list of scenarios, run id, label, record
flag. Result: run id, results path, counts, and every EpisodeResult. Feedback:
progress and the last result.
*Q: Why does the batch write files itself instead of leaving it to the client?* — A:
A crashed or disconnected client must not lose results. The runner appends each row
as soon as its episode ends.

### src/navstress_monitor (ament_cmake, C++)

**`CMakeLists.txt`, `package.xml`** — C++17 package skeleton with `-Wall -Wextra
-Wpedantic`, `ament_lint_auto` (cppcheck, cpplint, lint_cmake, xmllint, copyright)
plus clang-format, with uncrustify off. Phase 2 adds the node and gtests.
*Q: What does `ament_lint_auto_find_test_dependencies()` do?* — A: It reads every
`ament_lint*`/`ament_cmake_*` test dependency in package.xml and registers each one
as a test, so adding a linter is a one-line package.xml change.

### src/navstress_bringup (ament_cmake)

**`CMakeLists.txt`, `package.xml`** — Empty install-only package that will hold
launch files and parameter files (Phase 2/3).
*Q: Why ament_cmake for a package with no C++?* — A: It's the conventional choice for
data-only packages: `install(DIRECTORY launch params ...)` is one line, and later
launch_testing tests register with `add_launch_test`.

### Python packages: navstress_scenarios, navstress_eval, navstress_agent, navstress_report, navstress_fakesim

**`package.xml`, `setup.py`, `setup.cfg`, `resource/<pkg>`, `<pkg>/__init__.py`**
(one set per package) — Standard ament_python layout. `setup.py` installs
`package.xml` and the empty `resource/<pkg>` marker into the ament index (that's how
`ros2 pkg` finds the package); `setup.cfg` puts console scripts in `lib/<pkg>` so
`ros2 run` finds them. Each `__init__.py` holds only a one-line description; code
arrives in Phases 1–2.
*Q: What breaks if `resource/<pkg>` is missing?* — A: The package still builds, but
it isn't registered in the ament index, so `ros2 run`, `ros2 launch` and
`get_package_share_directory()` can't find it.
