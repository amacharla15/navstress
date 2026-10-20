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

---

## Phase 1 — schema, agent loop, report (pure Python)

Build/test status for this phase: `colcon build` and `colcon test` pass in the local
Jazzy container (99 tests, 0 failures). The scenario tests also pass under pydantic
2.13.5 on Python 3.13, with the same pinned hash. None of this code needs a ROS
runtime; the ROS wiring comes in Phase 2.

### src/navstress_scenarios

**`navstress_scenarios/schema.py`** — The ScenarioSpec schema and the only door from
LLM output to the simulator. Numeric fields are clamped to fixed ranges before
pydantic sees them, and every clamp is reported back. Unknown keys, assets or Nav2
override names, non-finite numbers, booleans or strings where a number belongs, and
too many obstacles are rejected with the field path in the message. It also
canonicalizes and hashes specs (labels excluded) and builds the machine-readable
schema summary the prompt uses, from the same tables the validator enforces.
*Q: Why clamp some things and reject others?* — A: A height of 5 m has an obvious
safe neighbour (the 2 m cap), so clamping keeps a useful scenario and logs the
change. An unknown field like `cmd_vel` or an asset named "forklift" has no safe
neighbour; guessing would mean running something nobody asked for.

**`navstress_scenarios/goals.py`** — Loads a map_server map (YAML + image, with the
same pixel-to-occupancy rule as map_server), marks cells within a robot radius of
anything non-free as blocked, flood-fills from the start, and samples seeded goals
only inside the connected region. Also defines `GoalSet`, the named reachable goals
that validation snaps goals to. A `sample_goals` CLI writes a goal-set YAML with the
map's sha256 and the sampling parameters recorded.
*Q: Why isn't "free" good enough for a goal?* — A: A free cell inside a sealed room
is free but unreachable; a run to it fails for reasons that have nothing to do with
the navigation stack. Connectivity from the start pose is what "reachable" means.

**`navstress_scenarios/generate.py`** — Builds the baseline set: N scenarios to
seeded-random goals from the goal set, each with its own seed, and a `make_baseline`
CLI that writes them as `scenarios.jsonl`.
*Q: How do you guarantee the Phase 6 rerun uses the identical baseline?* — A: The
same (goal set, N, seed) produces the same specs and hashes, and the exact specs
are also stored next to the results, so the rerun reads the file rather than
regenerating.

**`test/test_schema.py`** — 44 tests: clamping per field with the clamp notes,
allowlist rejections (asset, override, unknown top-level and obstacle fields, too
many or duplicate obstacles, NaN/inf, bool/str numbers, bad labels and versions),
goal resolution against a goal set and a free-space map, yaw normalization, and
hashing (pinned golden hash, key order and float noise ignored, labels ignored,
every physical change detected, tamper-proof JSONL round trip).
*Q: Why pin a literal hash in a test?* — A: Every stored result row carries a hash.
If a refactor silently changed canonicalization, old and new runs would stop
matching in before/after comparisons. The pinned value makes that a test failure.

**`test/test_goals.py`** — Builds a two-room PNG map where one room is sealed, then
checks map coordinates, inflation (including the map edge), that the sealed room is
free but unreachable, deterministic sampling with separation constraints, loud
failure when constraints can't be met, YAML round trip, and baseline determinism.
*Q: Why a synthetic map instead of the real warehouse map?* — A: The test needs a
known right answer (the sealed room). The real map only exists on the pod, and its
answer isn't known in advance.

**`package.xml`, `setup.py` (changed)** — Declare pydantic, PyYAML, NumPy and Pillow
as runtime deps and install the `sample_goals` and `make_baseline` executables.
*Q: Why `python3-pil` instead of pip's Pillow?* — A: rosdep keys map to apt packages,
so CI and the pod get the same Ubuntu-built versions without pip in the loop.

### src/navstress_eval

**`navstress_eval/results_io.py`** — The results-file contract: the CSV column order
(matching EpisodeResult), typed parsing (booleans, ints, `inf` clearance),
consistency checks (no failure reason on a success, only known reasons on a
failure), per-episode append, `scenarios.jsonl`, and `run_metadata.json` (git commit
and dirty flag, ROS distro, simulator and version, GPU from `nvidia-smi`, Nav2 params
sha256).
*Q: Why write a row per episode instead of once at the end?* — A: A 50-episode batch
on Isaac Sim takes an hour. If the node or the pod dies halfway, the finished
episodes are still on disk with their hashes.

**`navstress_eval/summary.py`** — Per-family statistics (success rate, failure-reason
counts, collisions, minimum finite clearance, median time to goal, worst cases
ranked collision > stuck > aborted > timeout), the weakest-family rule, the
success-vs-value sweep curve, and the boundary as a bracketing pair.
*Q: Why does an infinite clearance count as "no data" rather than "very safe"?* — A:
Infinity means the monitor never saw a return. Treating it as a large clearance
would make a blind run look like the safest one.

**`test/test_results_io.py`, `test/test_summary.py`** — Round trip with types and
infinity, rejection of inconsistent rows and unexpected headers, run-id format,
metadata round trip; summary counts, worst-case ordering, weakest-family
tie-breaks, sweep curves ignoring unknown hashes, and boundary/no-boundary cases.
*Q: What does "ignoring unknown hashes" protect against?* — A: A result whose
scenario isn't in the run's spec file can't be placed on the sweep axis. Dropping it
is honest; guessing its value would invent a data point.

**`package.xml` (changed)** — Adds `navstress_scenarios` (for `get_path`).
*Q: Why does eval depend on scenarios and not the other way round?* — A: Scenarios
define inputs; evaluation reads inputs and outputs. Keeping the arrow one way avoids
an import cycle.

### src/navstress_agent

**`navstress_agent/llm.py`** — `OpenAICompatibleClient` (POST `chat/completions`, GET
`models`, Bearer auth, optional JSON mode, errors with the HTTP body), configured
only from `NAVSTRESS_LLM_*` env vars; `extract_json` tolerant of code fences and
prose; and `MockLLM`, which reads the prompt's `CONTEXT_JSON` line and returns four
exploration families, or a one-parameter sweep of the focus family's worst case.
*Q: Isn't a mock that follows instructions too easy?* — A: Yes, deliberately. It
tests the loop's plumbing and the code-side narrowing, not the model's reasoning.
Whether a real model proposes good scenarios is a Phase 5 question with logged
prompts and replies to judge it by.

**`navstress_agent/prompts.py`** — System prompt (role, "you never control the
robot", reply format) and per-round user messages: budget, exploration or narrowing
instructions, a results table, the worst cases of the focus family, and a
`CONTEXT_JSON` line with the schema, goals and start pose.
*Q: Why put the schema in every request instead of the system prompt?* — A: Each
request is then self-contained and fully logged, so any reply can be judged against
exactly what the model was shown.

**`navstress_agent/loop.py`** — `StressAgent`: per round it prompts, parses,
validates every scenario, drops off-focus scenarios and duplicate hashes, truncates
to the episode budget round-robin across families, retries with the error when a
reply is unusable (within the call budget), runs the batch through an injected
`run_batch` function, summarizes, and narrows the next round onto the weakest
family. Sweeps get a success curve and a bracketed boundary. Everything goes to
`agent_log.jsonl`; the final result goes to `agent_summary.json`.
*Q: Why inject `run_batch` instead of calling ROS directly?* — A: The loop can be
tested in milliseconds with a fake world and no ROS at all. In Phase 2 the real
implementation is a RunBatch action client, and the loop doesn't change.

**`test/test_loop.py`** — The required 2-round test: round 1 explores four families;
the fake world makes `low_obstacle` fail; round 2 runs only `low_obstacle`, sweeps
`obstacles[0].height` with everything else fixed, and the reported boundary
brackets the fake world's 0.25 m threshold. Also checks the log's event sequence and
contents, budget truncation, retry on bad JSON, rejected-but-logged scenarios
(including `cmd_vel`), off-focus dropping, and the early stop when nothing failed.
*Q: How do you know the narrowing is done by code and not luck?* — A: One test
scripts a reply that mixes families in round 2; the off-focus scenario is dropped
and logged, and only the focus family reaches `run_batch`.

**`test/test_llm.py`** — Runs the HTTP client against a stub server on 127.0.0.1:
request path, auth header, body and JSON mode; model listing; HTTP and connection
errors; env-var configuration; JSON extraction; and the mock's context parsing.
*Q: Does any test call a real LLM?* — A: No (CLAUDE.md rule 4). The only network
traffic is to a local stub server the test starts itself.

**`package.xml` (changed)** — Adds `navstress_eval` and `navstress_scenarios`.
*Q: Why is the LLM client in this package and not a shared one?* — A: Only the agent
talks to an LLM; a separate package for one module would be structure without
benefit.

### src/navstress_report

**`navstress_report/data.py`** — Loads a run (rows, metadata, specs by hash) or an
agent run (its summary plus each round's batch directory).
*Q: Why reload from disk instead of passing results in memory?* — A: The report
must show what's on disk, the same files a reader can check, never an in-memory
state that may differ.

**`navstress_report/compare.py`** — Pairs before/after rows by config hash, counts
unmatched scenarios, and lists per-family rates, regressions and fixes. A hash that
appears twice in one run is an error.
*Q: Why pair by hash rather than by row order or scenario id?* — A: Order can change
and ids are labels. The hash is the configuration itself, so a pair is guaranteed
to be the same physical scenario.

**`navstress_report/charts.py`** — Three PNG charts, each in light and dark: success
by family (single-series bars, worst first), the boundary sweep (line with the
crossing bracket shaded), and before/after per family (ring = before, dot = after).
*Q: Why both a ring and a color for before/after?* — A: When both values are 100%
the marks overlap. Shape keeps them distinguishable there, and for readers who
can't tell the colors apart.

**`navstress_report/render.py`** — Builds the report once as blocks (headings,
provenance lines, tables, images) and renders them to Markdown and to a
self-contained HTML page. Every table has a provenance line (results file, git
commit, simulator, ROS distro, GPU, Nav2 params hash). A FAKESIM banner appears if
any run came from the fake simulator. Rewrites only the section between its
begin/end markers, so hand-written parts of RESULTS.md survive.
*Q: How does the report enforce "every number traces to a results file"?* — A: It
has no input except results directories, and it prints each table's source path
and commit right next to it.

**`navstress_report/cli.py`** — `ros2 run navstress_report report --run ... --agent
... --before ... --after ... --out-md docs/RESULTS.md --site site/`.
*Q: Why is the static site generated rather than hand-written?* — A: Then the site
and RESULTS.md can never disagree: both are rendered from the same blocks.

**`test/test_report.py`** — Runs the real agent loop with the mock LLM and a batch
function that writes real results files into a temp dir, plus a "fixed" rerun of
the same scenarios. It then checks the Markdown (banner, provenance, tables,
relative bag links, sweep bracket, hash matching, fixed/regression counts), the
PNGs, and the HTML. It also checks that reruns keep hand-written sections, that
comparisons report regressions and unmatched rows, and the CLI argument checks.
*Q: Isn't generating a report from fake data risky?* — A: It only happens inside the
test's temp dir, the data is marked `simulator: fakesim`, and the test asserts the
FAKESIM banner is shown. Nothing from it lands in docs/.

**`package.xml`, `setup.py` (changed)** — Adds matplotlib, eval and scenarios as
runtime deps, the agent as a test dep, and the `report` executable.
*Q: Why use the Agg backend?* — A: CI and the pod are headless. Agg renders straight
to PNG without needing a display.

### docs and README (changed)

**`docs/DECISIONS.md`** — D7–D15 (pydantic v1 API, clamp vs reject and hash rules,
goal sampler, primitive obstacles, override ranges, CSV, HTTP LLM client, code-side
narrowing, chart rendering) and the dependency table.
**`docs/OPEN_QUESTIONS.md`** — Q4 (LLM model id), Q5 (goal-sampling radius), Q6
(real pallet asset).
**`README.md`** — Phase 1 marked done.
*Q: Why list dependencies in a decisions file?* — A: CLAUDE.md requires a one-line
reason for each. Unexplained dependencies are how a small project ends up with a
large attack surface.

---

## Phase 2 — ROS nodes, fake simulator, end-to-end test

Build/test status for this phase: `colcon build` and `colcon test` pass in the local
Jazzy container: 181 results, 0 failures, 7 skipped. The skips are `ament_cppcheck`,
which skips by default on cppcheck 2.x. That includes the end-to-end launch test
(about 2 minutes). The CLIs (`stress_agent --mock`, `make_baseline`, `run_batch`,
`report`) were also run by hand against the fake simulator. None of this has touched
Isaac Sim or Nav2; that is Phase 4.

### src/navstress_scenarios

**`navstress_scenarios/conversions.py`** — Converts a validated ScenarioSpec to the
ROS message and back, plus yaw↔quaternion. On the way back the message is validated
again and its `config_hash` must match its content, or it is rejected.
*Q: Why re-validate a message that was built from a validated spec?* — A: Messages
cross process boundaries and can be hand-written with `ros2 action send_goal`.
Checking the hash on arrival means a result row can never carry a hash that doesn't
describe what actually ran.

**`navstress_scenarios/usd_assets.py`** — Writes each obstacle size as a small USDA
file (scaled unit Cube or Cylinder, base on the floor, collision enabled, kinematic
rigid body if it moves), named by content hash and written atomically. Also reads
the `navstress_*` metadata back, which is how the fake simulator learns obstacle
sizes.
*Q: Why files instead of sending geometry in the spawn request?* — A: Isaac Sim's
`isaacsim.ros2.sim_control` spawns from a USD URI and ignores `resource_string`
(checked in its 6.2.0 source). Isaac Sim and NavStress share the pod's filesystem,
so a path is all it needs.

**`navstress_scenarios/noise.py`** — Pure functions: Gaussian noise on valid
LaserScan returns (clipped to the sensor range, invalid returns untouched), dropouts
as +inf ("no return" in REP 117), and the PointCloud2 version that moves each point
along its ray.
*Q: Why move points along the ray rather than adding x/y/z noise?* — A: A lidar
measures range along each beam. Its error is in range, not sideways.

**`navstress_scenarios/motion.py`** — Back-and-forth position along a segment at
constant speed, and the map→world offset transform.
*Q: Why compute position from elapsed time instead of integrating velocity?* — A:
It can't drift, and the same scenario puts the obstacle in the same place at the
same time after the goal is sent.

**`navstress_scenarios/rosutil.py`** — Blocking service calls with timeouts from
inside callbacks, and get/set of another node's parameters through its standard
parameter services.
*Q: Why doesn't polling a future inside a service callback deadlock?* — A: The nodes
use a MultiThreadedExecutor with a reentrant callback group, so the reply is handled
on another thread while this one waits. With a single-threaded executor it would
deadlock.

**`navstress_scenarios/noise_injector.py`** — Relays LaserScan or PointCloud2
(`message_type`) from `input_topic` to `output_topic` with the current noise.
`gaussian_std_m`, `dropout_prob` and `seed` are ROS parameters with range
descriptors (the same ranges as the schema). The scenario manager sets them per
scenario, and a new seed re-seeds the generator. Zero noise forwards the message
untouched.
*Q: Why Sensor Data QoS on the output too?* — A: Nav2's costmap layers and AMCL
subscribe with SensorDataQoS. Publishing best effort keeps the relay behaving like
the sensor it replaces.

**`navstress_scenarios/obstacle_mover.py`** — Listens on the latched
`/navstress/active_scenario` topic and, at `rate_hz`, sends `SetEntityState` for
each moving obstacle. It never queues a second request for an obstacle whose last
one hasn't been answered.
*Q: Why not queue every update?* — A: If the simulator is slower than the update
rate, a queue grows without bound and the obstacle lags further and further behind
where the scenario says it is. Skipping keeps it as current as the simulator allows.

**`navstress_scenarios/scenario_manager.py`** — The `/navstress/apply_scenario`
service: reset, spawn all obstacles in one `SpawnEntities` call with fixed names,
optional robot placement, noise parameters, Nav2 overrides (with launch values
restored for anything not overridden), announce on `/navstress/active_scenario`,
`/initialpose` for AMCL, clear costmaps, settle. Anything that would make the run
differ from the spec fails the call.
*Q: Why republish `/initialpose` after every reset?* — A: Isaac Sim's reset stops
and replays the timeline, putting the robot back at its authored pose, while AMCL
still believes it is wherever the last episode ended. Without a fresh initial pose
the next episode would start mislocalized.

**`test/test_ros_pieces.py`** — 15 tests: spec↔message round trip with and without a
start pose, tamper rejection, quaternions, noise statistics/dropout rate/range
clipping/reproducibility, ray-direction preservation for clouds, shuttle motion,
map→world, USD content (including the empty-`apiSchemas` regression), asset caching
and metadata.
*Q: How was the USD validated if Isaac Sim isn't available?* — A: With Pixar's
`usd-core` from PyPI: both variants open, have the right schemas and bounding boxes.
That found a real parse error. The unit test pins the fix; the USD library itself
isn't a CI dependency.

**`package.xml`, `setup.py` (changed)** — ROS runtime deps and the
`scenario_manager`, `noise_injector`, `obstacle_mover` executables.
*Q: Why are the nodes plain `main()` entry points rather than components?* — A:
These are Python nodes, which can't be composed into one process the way C++
components can. Separate processes keep a crash in one from taking down the others.

### src/navstress_eval

**`navstress_eval/eval_runner.py`** — The `/navstress/run_episode` and
`/navstress/run_batch` action servers. Per episode: ApplyScenario, start the bag,
send `NavigateToPose`, then watch at 10 Hz for Nav2's result, a collision event, the
stuck detector, the sim-time timeout or the wall-clock backstop (cancelling the goal
when it stops the episode itself). Then stop the bag, score, publish on
`/navstress/episode_results` and append the CSV row. Batches validate every scenario
up front, write `scenarios.jsonl` and metadata, refuse to reuse a run id, and
support cancellation between episodes.
*Q: Why does a collision fail the episode even if Nav2 reaches the goal?* — A: In a
warehouse, reaching the goal by hitting a pallet is not a pass. Counting it as
success would hide exactly the failures this project is looking for.

**`navstress_eval/episode.py`** — ROS-free bookkeeping: path length with a jitter
threshold, the stuck detector (no progress across a full window), and the outcome
rules with their priority order.
*Q: Why is the stuck window longer than Nav2's progress checker?* — A: Nav2 should
get to run its recoveries first; "stuck" here means those recoveries are looping
without Nav2 giving up.

**`navstress_eval/bag_recorder.py`** — Starts `ros2 bag record -s mcap` for one
episode in its own process session with a QoS override file, waits for discovery,
and stops it with SIGINT so the MCAP file is finalized.
*Q: What went wrong before the QoS override file existed?* — A: The end-to-end test
found bags missing `/scan_raw` and `/odom`. The recorder had subscribed RELIABLE to
best-effort publishers, which never match, and ROS only prints a warning. Now every
best-effort topic is subscribed best effort and latched topics transient-local.

**`navstress_eval/batch_client.py`** — The RunBatch action client used by the agent
and by the `run_batch` CLI, which runs a `scenarios.jsonl` file (optionally checked
against a goal set) as one batch.
*Q: Why does the client have no timeout on the batch result?* — A: A 50-episode batch
on Isaac Sim takes about an hour. A client timeout would abandon a healthy batch;
the per-episode timeouts on the server already bound each episode.

**`navstress_eval/conversions.py`** — EpisodeResult message ↔ CSV row dict, keyed by
the one `RESULT_FIELDS` list.
*Q: Why derive both directions from `RESULT_FIELDS`?* — A: The CSV, the message and
the report then can't disagree about which columns exist.

**`test/test_episode.py`** — Path integration with jitter, stuck detection
(including a sliding window), every outcome-priority case, row↔message round trip,
the recorder's command line and QoS overrides.
*Q: Which outcome rule is easiest to get wrong?* — A: Priority. A goal cancelled by
the runner for being stuck comes back from Nav2 as "canceled". The reason must be
the runner's (`stuck`), not Nav2's status (`nav2_aborted`).

**`package.xml`, `setup.py` (changed)** — ROS deps, `ros2bag` and
`rosbag2_storage_mcap`, PyYAML, and the `eval_runner` and `run_batch` executables.
*Q: Why depend on `rosbag2_storage_mcap` explicitly?* — A: The MCAP storage plugin is
what `-s mcap` loads. Without the dependency rosdep might not install it, and
recording would fail only at runtime.

### src/navstress_monitor (C++)

**`include/navstress_monitor/clearance.hpp`, `src/clearance.cpp`** — ROS-free
geometry: even-odd point-in-polygon, distance to a segment, signed distance to the
footprint, minimum clearance over a point set (optionally ignoring points inside the
footprint), a hysteresis collision tracker that counts rising edges, and a parser
for Nav2's footprint string.
*Q: Why ignore points inside the footprint?* — A: On a real robot they are lidar
returns from the robot's own body. Counting them would report a permanent
collision.

**`include/navstress_monitor/collision_monitor_node.hpp`,
`src/collision_monitor_node.cpp`, `src/main.cpp`** — The `navstress_collision_monitor`
node. It subscribes to any number of LaserScan and PointCloud2 topics (Sensor Data
QoS) and to odometry, and transforms points into the base frame with tf2 (latest
transform, since sensors are rigidly mounted). Cloud points are height-filtered.
Sources older than `source_timeout_s` are ignored. It publishes `/navstress/clearance`
(best effort) and `/navstress/collision_events` (reliable, rising edges only). The
default footprint is Carter's Nav2 footprint.
*Q: Why look up the latest transform instead of the one at the scan's stamp?* — A:
The sensor→base transform is static, so the latest is exact. It also avoids
dropping scans whose stamp is slightly newer than the newest TF in the buffer.

**`test/test_clearance.cpp`** — 8 gtests: convex and concave point-in-polygon in
both windings, segment distance with clamping, signed distance including Carter's
footprint, self-return filtering, the "no points means invalid, not zero" case,
hysteresis with two separate collisions, and footprint parsing errors.
*Q: Why is "no points" invalid rather than zero clearance?* — A: Zero would be a
collision. No data is not a collision, and it's reported as +inf with
`in_collision` false.

**`test/test_collision_monitor_node.cpp`** — 2 gtests running the real node with a
static TF (laser 0.2 m ahead of base_link): the published clearance and closest
point match hand-computed values in the base frame, and a run of near-touching scans
produces one event, a second after clearing the hysteresis band.
*Q: What does this test catch that the geometry tests can't?* — A: The wiring:
parameter parsing, QoS matching, the TF transform direction, and that the event
publisher fires once per collision.

**`CMakeLists.txt`, `package.xml` (changed)** — Two libraries (`clearance` with no ROS
types, and the node), the executable, the plain gtest, an
`ament_add_ros_isolated_gtest` for the node test, and lint (clang-format, cpplint,
cppcheck, copyright, xmllint, lint_cmake).
*Q: Why split the geometry into its own library?* — A: Its tests link without
rclcpp and run in milliseconds, and the math can be reused without a node.

### src/navstress_agent

**`navstress_agent/cli.py`** — `ros2 run navstress_agent stress_agent`: loads
`.env` (without overriding the environment), picks the mock or the
OpenAI-compatible client, reads the goal set, optionally seeds round 1 with a
baseline run's rows, and runs the agent with the RunBatch client.
`--list-models` asks the endpoint which models exist.
*Q: Why refuse to start without a model id instead of picking a default?* — A:
Free-tier model ids change. A silent default that later disappears would fail
mid-run; an explicit setting fails at startup with a clear message.

**`package.xml`, `setup.py` (changed)** — Adds `rclpy` and the `stress_agent`
executable.
*Q: Does the agent node ever publish to the robot?* — A: No. Its only ROS output is a
RunBatch goal containing validated scenarios.

### src/navstress_fakesim

**`navstress_fakesim/world.py`** — The fake world: walls, box and cylinder
obstacles, a square robot that can't move into anything, a 360-ray lidar that only
sees obstacles at least `lidar_height` tall, and an A* planner (8-connected,
inflated by the footprint's half-diagonal) straightened by line-of-sight that only
avoids what the lidar sees.
*Q: Why make low obstacles invisible to the fake planner?* — A: So the pipeline has a
real failure to find in CI, of the same shape as the hypothesis. It is a test
fixture, not evidence about Nav2.

**`navstress_fakesim/fake_sim.py`** — The `fake_sim` node: `simulation_interfaces`
services (features, reset, spawn one/many from NavStress USDA files, delete,
get/set entity state, get entities), clean `/scan_raw`, `/odom` and TF (best effort,
like Isaac Sim), and a fake `navigate_to_pose` server. That server follows the
plan, replans, counts "recoveries" when blocked, aborts when they run out, and
applies an invented noise rule.
*Q: What can a fakesim pass not tell you?* — A: Anything about Nav2's planners,
costmaps, AMCL or Isaac Sim's sensors and physics. It proves the services, actions,
QoS, TF, bags, result files, agent and report fit together.

**`config/world.yaml`, `config/goals.yaml`** — A 14 × 8 m room with two shelves, and
six hand-picked goals with clear straight lines from the start.
*Q: Why hand-pick fakesim goals instead of sampling them?* — A: With straight,
unobstructed paths, where a mock obstacle lands relative to the path is
predictable, so the end-to-end test can rely on low obstacles blocking.

**`test/test_world.py`** — Every goal has a one-segment path; the scan sees walls and
tall obstacles but not low ones; cylinder ray hits; the planner detours around
visible obstacles and drives straight through invisible ones; blocking ignores
height; reset clears obstacles.
*Q: Why test the fake at all?* — A: If the fake were wrong, the end-to-end test could
pass or fail for reasons unrelated to the code under test.

**`package.xml`, `setup.py` (changed)** — ROS deps (`tf2_ros_py` for the Python
broadcasters), the config files and the `fake_sim` executable.
*Q: Why is the fake a ROS package and not a pytest fixture?* — A: It has to be a
separate process speaking real ROS interfaces, or the test would bypass the very
wiring it exists to check.

### src/navstress_bringup

**`launch/navstress_nodes.launch.py`** — Starts scenario_manager, noise_injector,
obstacle_mover, navstress_collision_monitor and eval_runner with one params file,
`use_sim_time`, and `results_dir`.
*Q: Why one params file for all nodes?* — A: The settings that must agree (entity
prefix, map→world offset, topic names) sit side by side, so a mismatch is visible.

**`launch/fakesim.launch.py`** — The fake simulator plus the NavStress nodes with
`params/navstress_fakesim.yaml`.
*Q: How would you run the same pipeline against Isaac Sim?* — A: Same nodes, a
different params file, and Isaac Sim + Nav2 instead of fake_sim (Phase 3 adds that
launch file).

**`params/navstress_fakesim.yaml`** — Parameters for the fake: no Nav2 overrides,
no AMCL, no costmaps, the monitor on the clean `/scan_raw` with a 0.6 m square
footprint, bag topics with best-effort/latched QoS lists, shorter stuck window.
*Q: Why does the monitor read `/scan_raw` and not `/scan`?* — A: `/scan` carries the
injected noise. Safety is measured on the clean data (D18).

**`test/test_e2e_fakesim.py`** — The Phase 2 acceptance test. It launches the
pipeline in a private ROS domain and runs the real agent loop (mock LLM, 2 rounds,
≤8 episodes per round) over the RunBatch action. It then checks: round 2 narrows
onto round 1's weakest family; every episode wrote a row and none failed setup;
metadata says fakesim; low obstacles fail in the fake; the bags contain `/scan_raw`,
`/scan`, `/odom` and `/navstress/clearance`. The report built from those files has
the FAKESIM banner. A second test drives one RunEpisode directly.
*Q: What did this test find that unit tests didn't?* — A: The rosbag2 QoS mismatch
(D17): the bags were silently missing best-effort topics. No unit test could see
it, because it only shows up with real publishers, subscribers and discovery
timing.

**`CMakeLists.txt`, `package.xml` (changed)** — Install launch and params; register
the launch test with a 900 s timeout; runtime and test dependencies.
*Q: Why `add_launch_test` instead of a pytest?* — A: launch_testing starts and stops
the processes around the test and collects their output, and colcon reports it like
any other test.

### docs and README (changed)

**`docs/DECISIONS.md`** — D16–D25 (node name, per-topic QoS, clean vs noisy data,
ApplyScenario strictness, USD assets, outcome rules, bag recording, the fake's
limits, test isolation, one-shot run ids) and new dependency rows.
**`docs/OPEN_QUESTIONS.md`** — Q3 resolved (owner identity); Q7–Q12 on Isaac-side
facts that can only be checked on the pod: robot prim path and teleport, map/world
alignment, kinematic bodies in `SetEntityState`, floor height, contact sensing,
and routing the 3D cloud through the noise relay.
**`README.md`** — Phase 2 done, and how to run the fake pipeline by hand.
*Q: Why so many open questions about Isaac Sim?* — A: Each is a fact that can only
be observed on the GPU pod. Writing them down with a default and a check is what
keeps Phase 4 from discovering them as silent wrong results.

**Package maintainer and copyright lines (changed, all packages)** — Now
`Akshith Macharla <amacharla@csuchico.edu>`, the identity the owner supplied for
this repo.

---

## Phase 3 — pod setup script and docs scaffolding

Build/test status for this phase: `docker/pod_setup.sh` passes shellcheck 0.9.0
(Ubuntu 24.04's) and 0.11.0 (latest) with no findings, and CI now runs shellcheck.
In an Ubuntu 24.04 container standing in for the pod, the script refused to run as a
non-root user, completed `--dry-run`, and really installed the base packages into an
apt cache on the "volume". It then stopped cleanly where this sandbox blocks GitHub
and packages.ros.org. In the Jazzy container its clone + build step built all eight
NavStress packages from a fresh clone and skipped the rebuild on a second run.
**The ROS/Nav2 install, the Isaac workspace build and `isaac.launch.py` have not run
anywhere yet**; that is Phase 4, on the pod. `colcon test`: all packages pass,
including the new launch-file and Nav2-params tests.

**`docker/pod_setup.sh`** — Sets up a pod running `nvcr.io/nvidia/isaac-sim:6.2.0`.
It checks Ubuntu 24.04, root, volume, GPU and Isaac Sim, then installs base tools and
a UTF-8 locale, adds the ROS apt source via `ros2-apt-source` (per the Jazzy guide),
and installs ROS 2 Jazzy, Nav2, `pointcloud_to_laserscan`, `simulation_interfaces`
and the MCAP plugin. Debs are cached on the volume. It clones IsaacSim-ros_workspaces
at `IsaacSim-6.2.0` and builds only the three Carter packages, clones and builds
NavStress (each rebuilt only when its commit changes), links Isaac Sim's caches onto
the volume, writes `env.sh`, and hands ownership to uid 1234. Options: `--dry-run`,
`--with-claude-code`, `--no-cache-links`. It ends by reminding you to stop the pod.
*Q: What makes a setup script idempotent, concretely?* — A: Every step first checks
the state it would create (installed debs via dpkg, a clone's `.git`, a build marker
holding the commit it was built from, a symlink's target) and does nothing if it's
already right. Re-running is then a cheap verification, and a half-finished run can
simply be repeated.

**`src/navstress_bringup/launch/isaac.launch.py`** — The pod launch. Isaac Sim
headless via NVIDIA's `isaacsim_bringup` (with `isaacsim.ros2.sim_control` enabled
and Carter's scene), Nav2 via `nav2_bringup` with Carter's map and a rerouted copy of
Carter's params, `pointcloud_to_laserscan` with Carter's exact settings but output
`/scan_raw`, the NavStress nodes, and a second noise relay for the 3D cloud.
`launch_isaac:=false` attaches to an Isaac Sim that's already running;
`nav2_params:=` is where the Phase 6 fix goes in.
*Q: Why not just include NVIDIA's `carter_navigation.launch.xml`?* — A: It starts
RViz unconditionally (the pod is headless), and its `/scan` and local-costmap inputs
come straight from the simulator, so noise couldn't be injected without changing
NVIDIA's files. The launch copies its settings and changes only those two things.

**`src/navstress_bringup/params/navstress_isaac.yaml`** — NavStress parameters for
the pod: Carter's odometry topic (`/chassis/odom`), the AMCL initial pose as the
default start, the Nav2 override targets (controller and both inflation layers), the
costmap clear services, both noise relays, bag topics with their QoS lists (raw point
clouds left out for size), and the collision monitor on the clean 3D cloud plus the
2D lidars if they exist. Unverified values are flagged with their OPEN_QUESTIONS
numbers.
*Q: Why is the collision monitor fed the 3D cloud rather than `/scan`?* — A: `/scan`
is derived from that cloud with a height band and only the front half, and it is the
noisy topic. The raw cloud is clean, all-round, and height-filtered by the monitor
itself.

**`src/navstress_bringup/launch/navstress_nodes.launch.py` (changed)** — New
`nav2_params_file` argument passed to `eval_runner`, so run metadata records which
Nav2 config was used and its sha256.
*Q: Why record the Nav2 params hash per run?* — A: Phase 6 compares the same
scenarios under two Nav2 configs. The hash proves which config each row ran with.

**`src/navstress_scenarios/navstress_scenarios/nav2_params.py`** — Finds every costmap
observation source in a Nav2 params file, writes a copy with the requested topics
renamed (and a header listing exactly what changed), named by content hash. Fails if
a requested topic isn't read by any source.
*Q: Why fail when a topic isn't found?* — A: If NVIDIA renames the lidar topic in a
future release, a silent no-op would leave noise injection disconnected while every
scenario still reported its noise settings. Failing at launch makes that visible.

**`src/navstress_scenarios/test/test_nav2_params.py`** — Against a trimmed copy of
Carter's costmap structure: only the mapped topic changes, the input isn't mutated,
the written file is stable and content-named, and a missing topic is an error. (The
real Carter file was also checked by hand: exactly one key changes.)
*Q: Why not test against the real Carter file in CI?* — A: It belongs to another
repository and isn't installed in CI. The trimmed copy keeps the exact nesting the
code depends on, with its source cited.

**`src/navstress_bringup/test/test_launch_files.py`, `CMakeLists.txt`, `package.xml`
(changed)** — Imports every launch file and builds its description (including the
Isaac one, whose package lookups happen only at run time), and checks every params
file is valid YAML with `ros__parameters`. Registered with `ament_add_pytest_test`;
`pointcloud_to_laserscan` added as a runtime dependency.
*Q: What can this catch for a launch file that never runs in CI?* — A: Syntax
errors, broken imports, renamed or missing launch arguments, and malformed params.
These are the usual first-five-minutes failures on an expensive GPU pod.

**`docs/RUNPOD.md`** — The owner checklist (RunPod billing and alert, SSH key, LLM key
as a RunPod secret, optional Claude/NGC accounts, EULA acceptance), then: GPU choice
(RTX ≥16 GB; not A100/H100, which lack RT cores; driver ≥570.169), the network
volume, the template (image, entrypoint override to stay headless, disk sizes, env
vars `ACCEPT_EULA`, `PRIVACY_CONSENT`, `NVIDIA_DRIVER_CAPABILITIES`), SSH and tmux,
`pod_setup.sh`, the non-root fallback, the run and check commands for Phases 4–5,
Claude Code on the pod, and stop/terminate with a scheduled auto-stop. Sources
listed.
*Q: Why override the container's entrypoint?* — A: The image's entrypoint starts
Isaac Sim's WebRTC streaming app. SPEC wants headless with no GUI streaming, and
NavStress needs to start Isaac Sim itself with the sim-control extension.

**`docs/RESULTS.md`, `docs/INTERVIEW.md`** — Scaffolds. RESULTS.md says plainly that
there are no results yet and lays out the Phase 4–6 sections (the report fills its
generated section between markers). INTERVIEW.md lists the SPEC topics and where
each answer will come from.
*Q: Why create RESULTS.md before there are results?* — A: The report tool rewrites
only its marked section, so the hand-written bring-up notes from Phase 4 have a home
that later report runs won't overwrite.

**`.github/workflows/ci.yml` (changed)** — The lint job now also runs shellcheck on
`docker/*.sh`.
*Q: Why lint a script that only runs on a pod?* — A: Because it only runs on a pod.
A quoting bug found in CI costs nothing; found on a rented GPU it costs money and a
setup cycle.

**`docs/DECISIONS.md`, `docs/OPEN_QUESTIONS.md`, `README.md` (changed)** — D26–D30
(volume layout and apt cache, root-to-install/uid-1234-to-run, headless start, Nav2
input rerouting, undeclared pod-only packages); Q13–Q17 (pod user, headless RTX
rendering, disk size, extension loading, initial pose); Phase 3 marked done, with
"not yet run on a pod".
