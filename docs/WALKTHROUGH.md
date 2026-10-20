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
