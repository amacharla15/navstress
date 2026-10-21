# Results

Everything here is from **simulation** (NVIDIA Isaac Sim 6.2.0), on **one route**, with
**single runs per configuration**. It shows the pipeline working end to end on a GPU; it
is not a statistical evaluation of Nav2. Each run's files are in `results/<run_id>/`
(`results.csv`, `scenarios.jsonl`, and `run_metadata.json` with the git commit, ROS
distro, simulator and GPU). The MCAP bags are kept locally and not committed.

## Setup

| | |
| --- | --- |
| Simulator | NVIDIA Isaac Sim 6.2.0, Nova Carter in the small-warehouse navigation sample |
| ROS | ROS 2 Jazzy, Nav2 with NVIDIA's `carter_navigation` parameters |
| GPU | RunPod RTX PRO 4500 Blackwell (32 GB), driver 595.91.07, Ubuntu 24.04.3 |
| Scenario control | `isaacsim.ros2.sim_control` (`simulation_interfaces` services) |

Pod procedure: [RUNPOD.md](RUNPOD.md). Batches were run with
`ros2 run navstress_eval run_batch --scenarios <file>.jsonl --goals <goals>.yaml --run-id <id>`.

## Bring-up

- Isaac Sim loaded the warehouse headless; `/scan` arrived at about 9.5 Hz; AMCL
  localized at (−6.0, −1.0, π); the Nav2 lifecycle nodes went active.
- A manual `NavigateToPose` goal (about 2.7 m) succeeded in 12.65 s with 0 recoveries.
- Episodes then ran through NavStress's own pipeline: `RunBatch` → `ApplyScenario` → Nav2
  → MCAP bag → `results.csv`.

## Episode runs

| Run | Scenario | Config hash | Result | Time (s) | Path (m) | Closest approach (m) | Collisions | Recoveries | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `phase4_pilot` | baseline/000 | `705bfa6f` | success | 19.95 | 10.34 | 0.83 | 0 | 0 | No obstacles |
| `phase4_noise` | noise/000 | `437478a3` | success | 19.88 | 10.35 | 0.82 | 0 | 0 | 8 cm range noise, 20 % dropout |
| `phase4_box` | obstacle/000 | `1939b514` | success | 19.92 | 10.40 | 0.82 | 0 | 0 | Static 0.5 × 0.5 × 0.8 m box, before the file-URI fix; the costmap never marked it |
| `phase4_box_fixed` | obstacle/000 | `1939b514` | success | 19.88 | 10.36 | 0.82 | 0 | 0 | Static box, after the file-URI fix |
| `phase4_moving` | moving/000 | `93913640` | success | 20.05 | 10.34 | 0.82 | 0 | 0 | Moving box, before the kinematic-body fix |
| `phase4_moving_fixed` | moving/000 | `93913640` | success | 19.63 | 10.37 | 0.83 | 0 | 0 | Moving box, after the kinematic-body fix |
| `ghost_orig_1` | ghost/000 | `eff1c069` | success | 19.80 | 10.52 | 0.58 | 0 | 0 | Box deleted 15 s in; 3D layer `clearing: false` (original) |
| `ghost_orig_2` | ghost/000 | `eff1c069` | success | 19.53 | 10.58 | 0.67 | 0 | 0 | Box deleted 15 s in; 3D layer `clearing: false` (original) |
| `ghost_fix_1` | ghost/000 | `eff1c069` | success | 19.97 | 10.47 | 0.58 | 0 | 0 | Box deleted 15 s in; 3D layer `clearing: true` |
| `ghost_fix_2` | ghost/000 | `eff1c069` | success | 19.80 | 10.60 | 0.54 | 0 | 0 | Box deleted 15 s in; 3D layer `clearing: true` |
| `ghost_normal_1` | baseline/000 | `705bfa6f` | success | 20.13 | 10.38 | 0.83 | 0 | 0 | No obstacles; original configuration |

All runs use the same goal and start pose (seed 827307999). The four `ghost_*` runs share
one scenario (same config hash); their spread was about ±0.2 s and ±0.07 m, so smaller
differences between runs are noise.

## Pipeline checks

- **Sensor noise reaches Nav2's inputs.** Comparing `/scan_raw` with the relayed `/scan`
  in the `phase4_noise` bag (178 matched scans): 13,023 finite readings were dropped and
  51,364 were changed, i.e. 20.2 % dropout for a 20 % setting. The global costmap and AMCL
  subscribe to `/scan`; the local costmap's 3D layer subscribes to the relayed point cloud.
  On this route, 8 cm range noise with 20 % dropout did not change the outcome.
- **Spawned obstacles reach Nav2.** In the `ghost_*` runs the closest approach fell from
  about 0.83 m (no obstacle) to 0.54–0.67 m and the path got 0.1–0.25 m longer: Nav2
  steered around the box. In `phase4_box`, run before the file-URI fix below, the global
  costmap never marked the box's location.

## Integration bugs found and fixed

| Symptom | Cause | Fix |
| --- | --- | --- |
| Goal sampling failed when writing YAML (`RepresenterError`) | NumPy scalars from the map passed to `yaml.safe_dump` | Cast to `float` (`goals.py`) |
| Spawned obstacle not usable in the scene (`phase4_box`: the costmap never marked it) | Asset passed to `SpawnEntity` as a bare file path instead of a `file://` URI | `Path.as_uri()` (`scenario_manager.py`) |
| Repeated PhysX errors and a disrupted simulation with moving obstacles: `setLinearVelocity: Body must be non-kinematic!` | Isaac Sim's `SetEntityState` sets rigid-body velocities, which PhysX rejects for kinematic bodies | Spawn obstacles as collision-only geometry, so `SetEntityState` only sets the pose (`scenario_manager.py`). Trade-off: sensors see the boxes, but they are not physically simulated bodies |
| `pod_setup.sh` aborted on RunPod volumes | `chown` and ownership-preserving copies return "Operation not permitted" there | Copy without ownership; fall back to `chmod` when `chown` is refused |

## Stale-obstacle investigation (inconclusive)

Hypothesis, from reading the installed `carter_navigation` parameters: the local
costmap's 3D lidar layer (`nav2_costmap_2d::VoxelLayer`) has `marking: true` and
`clearing: false`, and its decay settings (`voxel_decay`, `decay_model`) are not
parameters of Nav2's VoxelLayer
([Nav2 docs](https://docs.nav2.org/configuration/packages/costmap-plugins/voxel.html)).
So cells marked by an obstacle that later disappears might never be cleared.

**Parked test.** Robot stationary, box spawned 1.5 m ahead, then deleted; occupied cells
sampled in `/local_costmap/front_3d_lidar_layer`. Raw output:
`results/parked_box_test.txt` and `results/parked_box_test_fix.txt`.

| 3D layer config | Occupied before spawn | With box | Newly marked | Still marked 5 s / 20 s / 35 s after deletion |
| --- | --- | --- | --- | --- |
| Original (`clearing: false`) | 0 | 11 | 11 | 11 / 11 / 11 |
| `clearing: true` | 12 | 15 | 9 | 9 / 9 / 9 |

Spawn and delete both returned `RESULT_OK`. The marks persisted in both configurations.
With clearing enabled, cells the lidar sees as empty should be cleared, so this result is
equally consistent with the deleted box still being visible to the sensor. The 2D lidar
layers, which always clear, were not checked, so the test cannot tell a stale mark from an
object that never disappeared. The two parked tests also started from different robot
positions.

**Driving check.** The same box scenario ran twice per configuration (`ghost_*` rows
above), with the box deleted 15 s into the run. All four succeeded with no recoveries and
no difference between configurations. The deletion most likely happened after the robot
had already passed the box, so these runs could not show a stale mark blocking the path.
`ghost_normal_1` ran with the original configuration. The `nav2_params_file` field in the
metadata of the original-configuration runs is wrong (inherited from the earlier
launcher); their actual configuration (`clearing: false`) was confirmed when Nav2 was
relaunched.

**Conclusion:** no confirmed stale-obstacle bug and no verified fix.

## What these results do not show

- Anything about real robots: all results are from simulation.
- Robustness statistics: one route, single runs.
- The LLM scenario agent on the GPU: it was tested only against a mock LLM in CI and was
  not used in these runs.
