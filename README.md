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
| 1 | schema, agent loop, report (pure Python) | not started |
| 2 | ROS nodes + fakesim end-to-end test | not started |
| 3 | pod setup script + RUNPOD.md | not started |
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

CI runs the same steps (`.github/workflows/ci.yml`).

## Docs

- [docs/WALKTHROUGH.md](docs/WALKTHROUGH.md) — every file, what it does and why
- [docs/DESIGN.md](docs/DESIGN.md) — architecture and the costmap-config reading
- [docs/DECISIONS.md](docs/DECISIONS.md) — choices, evidence, dependencies
- [docs/OPEN_QUESTIONS.md](docs/OPEN_QUESTIONS.md) — unresolved items and defaults taken
