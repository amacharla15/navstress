# NavStress — build spec

## 1. What it is
A ROS 2 project that answers: *where does a standard robot navigation stack break, and does a fix actually help?*

- **Robot and world:** NVIDIA's Nova Carter in the Isaac Sim small-warehouse navigation sample.
- **Navigation:** Nav2, driven through the `NavigateToPose` action.
- **Scenario control:** Isaac Sim's `isaacsim.ros2.sim_control` extension, which implements the standard ROS 2 `simulation_interfaces` package (spawn/delete entities, set entity state, reset, step, timeline control). NavStress talks to the simulator only through these ROS 2 interfaces — no custom code inside Isaac Sim.
- **The agent:** an LLM proposes test scenarios as structured JSON, the system runs them, the agent reads the results and zooms in on failures. The LLM never commands the robot; it only proposes scenario configs (validated against a schema) and goals that go through Nav2.
- **The fix:** change the navigation configuration to address the worst failure found, rerun the identical scenario set, and report before/after.

## 2. The worked example (the shape of the final results — numbers below are placeholders, real ones must come from runs)
1. **Baseline:** 50 runs to random reachable goals in the warehouse. Record success rate, time to goal, path length, collisions, minimum clearance.
2. **Agent round 1:** the agent proposes scenario families, e.g. a pallet half-blocking an aisle, lidar noise/dropout, a moving obstacle crossing the path, and a low obstacle. Each family runs N episodes.
3. **Agent round 2:** the agent narrows the worst family (e.g. sweeps obstacle height) to find the boundary where success collapses.
4. **Hypothesis to test, not assume:** obstacles lower than what the costmap's sensor input "sees" are invisible to the planner. First read the `carter_navigation` Nav2 params to find which sensor feeds the costmap and its height limits; that determines whether this failure exists. Report what you find either way.
5. **Fix and rerun:** e.g. add a depth/point-cloud observation source with a lower min height to the costmap, or whatever addresses the actual worst failure. Rerun the identical scenario set and report before/after.

## 3. Stack decisions
- **ROS 2 distro: prefer Jazzy.** Isaac Sim 6.x supports Humble and Jazzy; with Humble, Isaac Sim must use its internal ROS libraries because system Humble is built for Python 3.10 while Isaac Sim uses Python 3.12. Jazzy avoids that. Decision rule: use Jazzy if the Isaac Sim container's OS is Ubuntu 24.04; if it is 22.04, use Humble with internal libs. Record the outcome in DECISIONS.md.
- **Base setup:** follow Isaac Sim's own "ROS 2 Navigation" (Nova Carter + Nav2) and "ROS 2 Launch" tutorials and the `IsaacSim-ros_workspaces` repo (`carter_navigation`, `isaac_ros_navigation_goal`, `isaacsim_bringup`). Reuse them; do not rewrite what they provide.
- **Sensors:** Isaac Sim publishes Nova Carter sensors with Sensor Data QoS; every subscriber must match.
- **Sensor noise:** injected by our own relay node (subscribe to the real topic, publish a noisy copy, remap Nav2's input to it). Isaac Sim stays untouched.
- **Recording:** rosbag2 for every episode (MCAP storage), viewable in Foxglove.
- **LLM:** any OpenAI-compatible endpoint, configured by env vars; default to a free tier (e.g. Gemini's OpenAI-compatible endpoint — verify current docs). Mock LLM for all tests.

## 4. Repo layout (a colcon workspace)
```
src/
  navstress_interfaces/   msg/srv/action: ScenarioSpec.msg, EpisodeResult.msg, ApplyScenario.srv, RunEpisode.action, RunBatch.action
  navstress_bringup/      launch files (Isaac Sim + Nav2 + NavStress nodes), params, Foxglove layout
  navstress_scenarios/    (Python) scenario schema (pydantic), scenario_manager node (calls simulation_interfaces), noise_injector node, obstacle_mover node
  navstress_eval/         (Python) eval_runner node: RunEpisode/RunBatch action servers
  navstress_monitor/      (C++, rclcpp) collision_monitor node
  navstress_agent/        (Python) LLM agent node + CLI
  navstress_report/       (Python) results → charts → markdown report + static HTML site
  navstress_fakesim/      (Python) fake simulator + fake Nav2 action server for CI (no GPU)
docker/                   pod setup script (see Phase 4)
results/                  run outputs (gitignored except summaries)
docs/                     WALKTHROUGH, DESIGN, DECISIONS, INTERVIEW, RESULTS, RUNPOD, OPEN_QUESTIONS
.github/workflows/ci.yml
```

## 5. Components

**ScenarioSpec (schema, shared by agent, manager, runner):**
- `goal`: pose in the map frame (must be in a known reachable set, or sampled from free space).
- `obstacles`: list of {asset (from an allowlist of simple USD shapes/pallets), pose, size/height, optional velocity for moving obstacles}.
- `sensor_noise`: {gaussian_std_m, dropout_prob} applied by noise_injector.
- `nav_overrides`: small allowlisted set of Nav2 params (e.g. max velocity) — optional.
- `seed`, `episode_timeout_s`.
All numeric fields clamped to safe ranges; anything outside the allowlist is rejected with a clear error. The config hash (sha256 of canonical JSON) is stored with every result.

**scenario_manager (Python node):** service `ApplyScenario`. Resets the sim (simulation_interfaces ResetSimulation), spawns/places obstacles (SpawnEntity / SetEntityState), configures the noise injector and obstacle mover, places the robot at the start pose, and returns once the sim is ready.

**noise_injector (Python node):** relays the lidar/scan input Nav2 uses, adding Gaussian range noise and random dropout per the current scenario. Parameters updated per scenario.

**obstacle_mover (Python node):** moves "dynamic" obstacles along a straight segment at a set speed using SetEntityState.

**collision_monitor (C++ node):** subscribes to the robot's range sensor input and odometry, uses tf2 and the robot footprint to compute minimum clearance, publishes clearance and collision events (clearance below a threshold, or contact if available). Unit tests with gtest.

**eval_runner (Python node):**
- `RunEpisode` action: ApplyScenario → start rosbag2 recording → send `NavigateToPose` → monitor until success, failure, or timeout → stop recording → publish/save EpisodeResult.
- EpisodeResult: success, failure_reason (timeout / nav2_aborted / collision / stuck), time_to_goal_s, path_length_m, min_clearance_m, collisions, recoveries_triggered, scenario hash, bag path.
- `RunBatch` action: runs a list of scenarios, writes `results/<run_id>/results.parquet` (or CSV) plus run metadata (git commit, Isaac Sim version, ROS distro, GPU).

**stress_agent (Python node + CLI):**
- Round loop: given a summary of previous results, the LLM returns a JSON list of ScenarioSpecs plus a one-line hypothesis per family → validate → RunBatch → summarize (success rate per family, failure reasons, worst cases) → next round focusing on the weakest family (boundary search).
- Budget controls: max rounds, max episodes per round, max LLM calls.
- Every round's prompt, raw response, validated scenarios, and summary saved to `results/<run_id>/agent_log.jsonl`.

**report (Python):** charts (success rate per family, boundary sweep curve, before/after), failure table with links to bags, writes `docs/RESULTS.md` and a static site in `site/` (deployable to Vercel), embedding the demo videos.

**fakesim (Python, CI only):** fake simulation_interfaces services, fake scan/odom publishers, and a fake NavigateToPose action server with configurable success/failure, so the full pipeline (agent → batch → report) runs in CI without a GPU.

## 6. Phases

Phases 0–3 can be built without a GPU (cloud session). Phases 4–7 are **pod only**.

**Phase 0 — workspace skeleton.** colcon workspace, interfaces package, package.xml/CMakeLists/setup.py for every package, GitHub Actions CI using ROS's standard setup actions (setup-ros + action-ros-ci) on the chosen distro: build, colcon test, ruff, ament_lint.
Done when: CI builds the empty packages green.

**Phase 1 — schema, agent, report (pure Python, no ROS runtime needed for core logic).** ScenarioSpec with validation and hashing; agent round loop against an injected "run batch" function and the mock LLM; report generator from sample results.
Done when: unit tests cover validation/clamping/allowlist rejection, hash stability, a 2-round agent loop with mock LLM narrowing onto the weakest family, and report generation.

**Phase 2 — ROS nodes.** scenario_manager, noise_injector, obstacle_mover, eval_runner, collision_monitor (C++), plus fakesim.
Done when: a launch_testing integration test runs agent → RunBatch → results → report end to end against fakesim in CI; collision_monitor gtests pass.

**Phase 3 — docs scaffolding + pod setup script.** docs/RUNPOD.md (owner checklist), `docker/pod_setup.sh` that, on a fresh pod running the official Isaac Sim container image, installs the chosen ROS 2 distro + Nav2 + IsaacSim-ros_workspaces + this workspace, idempotently, onto a persistent network volume.
RUNPOD.md must cover: GPU choice (RTX A5000 or similar RTX card with ≥16 GB VRAM — no A100/H100, they lack RT cores), container image (official Isaac Sim image; check whether NGC login is needed), env vars (`ACCEPT_EULA=Y`, `PRIVACY_CONSENT=Y`, `NVIDIA_DRIVER_CAPABILITIES=all` — Isaac Sim needs Vulkan), network volume for persistence, headless operation only (no GUI streaming), how to install and run Claude Code on the pod, and stopping/terminating the pod.
Done when: the script passes shellcheck and RUNPOD.md is complete. Stop here if not on the pod.

**Phase 4 — bring-up (pod only).** Run pod_setup.sh. Launch Isaac Sim headless with the Nova Carter warehouse scene and Nav2 via the official launch files; enable `isaacsim.ros2.sim_control`; confirm the services exist (`ros2 service list`), sensors arrive with matching QoS, and one manual NavigateToPose goal succeeds. Then run one RunEpisode end to end with a recorded bag.
Done when: docs/RESULTS.md has a "bring-up" section with the exact commands, versions (Isaac Sim, ROS distro, Nav2, driver, GPU), and one successful episode result.

**Phase 5 — baseline + agent rounds (pod only).** Baseline batch (50 episodes). Agent rounds 1 and 2 (budget: ≤ 2 rounds of ≤ 40 episodes each to start). Read the Nav2 costmap config and record which sensors feed it and their height limits before interpreting results.
Done when: results files exist for all runs, and RESULTS.md has per-family tables and the boundary-sweep curve, all generated by the report tool.

**Phase 6 — fix and rerun (pod only).** Choose a fix that targets the worst real failure; explain the reasoning in DESIGN.md; apply it as a separate params file / launch argument (baseline config stays intact); rerun the identical scenario set (same seeds and hashes).
Done when: RESULTS.md shows before/after for the same scenarios, including any regressions the fix caused.

**Phase 7 — demo + docs (pod only for video capture).**
- Video: a third-person camera rendered in Isaac Sim (headless), recorded for 2–3 representative episodes (baseline failure, fixed success), converted to MP4 with ffmpeg. Plus Foxglove layout for replaying bags.
- README: pitch, architecture diagram (mermaid), quickstart for CI/fakesim, pod quickstart, results summary with links.
- docs/INTERVIEW.md: ~20 questions answered from this codebase (QoS and why Sensor Data QoS; tf2 frames; Nav2 costmap layers and why the failure happened; actions vs services vs topics; why the LLM never drives the robot; how scenario hashes make runs reproducible; what the fakesim proves and what it cannot; limitations of sim results).
- Static report site in `site/` ready for Vercel.
Done when: a reader can reproduce the CI pipeline from the README and follow RUNPOD.md to reproduce the pod results.

## 7. Cost guardrails
- Target total GPU spend: ~$3–5 (RTX A5000-class pod at ~$0.27/hr, ~10–14 hours total).
- Everything that can run without Isaac Sim runs in CI / fakesim.
- LLM calls: free tier only; mock in tests; per-run caps on calls and episodes.

## 8. Out of scope
Real hardware, sim-to-real claims, training RL policies (stretch only, separate spec), multi-robot fleets, GUI streaming.
