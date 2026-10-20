# NavStress

Where does a standard robot navigation stack break, and does a fix actually help?

NavStress drives NVIDIA's Nova Carter through Nav2 in the Isaac Sim small-warehouse
sample. An LLM agent proposes test scenarios as validated JSON (obstacles, sensor
noise, goals), the system runs them, and the agent narrows in on the failures. A
navigation-config fix is then measured on the identical scenario set.

The full build spec is [SPEC.md](SPEC.md); working rules are in [CLAUDE.md](CLAUDE.md).

## Status

| Phase | What | State |
| --- | --- | --- |
| 0 | colcon workspace, interfaces, CI | done |
| 1 | schema, agent loop, report (pure Python) | done |
| 2 | ROS nodes + fakesim end-to-end test | done |
| 3 | pod setup script + RUNPOD.md | done (script not yet run on a pod) |
| 4–7 | Isaac Sim runs on a GPU pod | not started (needs the pod) |

No simulation results exist yet. Nothing in this repo should be read as a measured
result until docs/RESULTS.md says otherwise.

## Build and test locally (ROS 2 Jazzy, Ubuntu 24.04)

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build
colcon test && colcon test-result --verbose
pipx run ruff==0.17.1 check . && pipx run ruff==0.17.1 format --check .
```

CI runs the same steps (`.github/workflows/ci.yml`), including an end-to-end
launch test: the fake simulator, every NavStress node, the agent with a mock LLM over
the RunBatch action, MCAP bags, results files and the report.

To watch the CI pipeline by hand:

```bash
source install/setup.bash
ros2 launch navstress_bringup fakesim.launch.py results_dir:=$PWD/results
# second terminal
ros2 run navstress_agent stress_agent --mock --results-dir $PWD/results \
  --goals install/navstress_fakesim/share/navstress_fakesim/config/goals.yaml
ros2 run navstress_report report --agent results/<agent_run_id> --out-md /tmp/fakesim.md
```

Fakesim output tests the plumbing only; it is not a navigation result.

## Docs

- [docs/WALKTHROUGH.md](docs/WALKTHROUGH.md) — every file, what it does and why
- [docs/DESIGN.md](docs/DESIGN.md) — architecture and the costmap-config reading
- [docs/DECISIONS.md](docs/DECISIONS.md) — choices, evidence, dependencies
- [docs/OPEN_QUESTIONS.md](docs/OPEN_QUESTIONS.md) — unresolved items and defaults taken
- [docs/RUNPOD.md](docs/RUNPOD.md) — owner checklist and pod procedure (GPU, image, volume, run, stop)
- [docs/RESULTS.md](docs/RESULTS.md) — empty until Phase 4 runs on the pod
