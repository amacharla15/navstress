# NavStress

A ROS 2 test harness for the Nav2 navigation stack, running NVIDIA's Nova Carter in the
Isaac Sim small-warehouse sample. You describe a test as a scenario (goal, obstacles,
sensor noise); NavStress sets up the simulator through ROS 2, sends the goal to Nav2,
records an MCAP bag, and scores the run.

##Demo
https://github.com/user-attachments/assets/a1cdb4e0-b596-40bd-9243-c1d6b6638484

<img width="1920" height="1200" alt="front_camera" src="https://github.com/user-attachments/assets/3272f518-082e-4c61-bab1-fbbf92ce5035" />


All results are from simulation. What was run and what it shows:
[docs/RESULTS.md](docs/RESULTS.md).

## Packages

| Package | What it does |
| --- | --- |
| `navstress_scenarios` | Scenario schema (validated, hashed so a run can be repeated exactly); `scenario_manager`, which spawns, moves and deletes obstacles through Isaac Sim's `simulation_interfaces` services; lidar noise relays; obstacle mover |
| `navstress_eval` | `RunEpisode` / `RunBatch` actions: apply the scenario, start a bag, send `NavigateToPose`, write `results.csv` |
| `navstress_monitor` | C++ (rclcpp) node: minimum clearance and collision events from the range sensor, odometry and tf2 |
| `navstress_fakesim` | Fake simulator and fake Nav2 for CI, so the whole pipeline runs without a GPU |
| `navstress_report` | Results to markdown tables and charts |
| `navstress_bringup` | Launch files for Isaac Sim, Nav2 and NavStress, and the CI end-to-end test |
| `navstress_agent` | An LLM loop that proposes scenarios as validated JSON. Tested only against a mock LLM in CI; not used in the GPU runs |

## Verified on a GPU

RunPod RTX PRO 4500, Isaac Sim 6.2.0, ROS 2 Jazzy:

- Episodes run end to end through the pipeline, each with an MCAP bag and a results row.
- Spawned obstacles reach Nav2: with a box near the route the robot steers around it.
- Lidar noise injection reaches Nav2's inputs (20.2 % of readings dropped for a 20 % setting).
- Integration bugs found on the GPU and fixed: NumPy values in YAML, `file://` URIs for
  spawned assets, kinematic bodies rejected by Isaac Sim's set-state service, and a setup
  script that failed on RunPod volumes.
- A suspected Nav2 costmap issue (stale marks from deleted obstacles) was tested and is
  **inconclusive**. Details are in RESULTS.md.

## Build and test locally (ROS 2 Jazzy, Ubuntu 24.04)

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build
colcon test && colcon test-result --verbose
pipx run ruff==0.17.1 check . && pipx run ruff==0.17.1 format --check .
```

CI runs the same steps (`.github/workflows/ci.yml`), including an end-to-end launch test
against the fake simulator. Fake-simulator output tests the plumbing only; it is not a
navigation result.

## Run on a GPU pod

[docs/RUNPOD.md](docs/RUNPOD.md): GPU choice, container image, setup script, launching
Isaac Sim and Nav2, running a batch, and stopping the pod.

## Docs

- [docs/RESULTS.md](docs/RESULTS.md): what was run and what it shows
- [docs/DESIGN.md](docs/DESIGN.md): architecture
- [docs/DECISIONS.md](docs/DECISIONS.md): choices and evidence
- [docs/OPEN_QUESTIONS.md](docs/OPEN_QUESTIONS.md): unresolved items
- [docs/RUNPOD.md](docs/RUNPOD.md): pod procedure

## Limitations

- Simulation only. Nothing here is a real-robot or sim-to-real result.
- One route, single runs per configuration: these are checks, not statistics.
- Moving obstacles are collision-only geometry repositioned by the set-state service.
  Sensors see them, but they are not physically simulated bodies.
