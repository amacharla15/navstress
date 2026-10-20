# Open questions

Questions that block or could change something. Each has the default chosen so work
could continue, and what resolves it. Owner-only items (accounts, credentials) live
in docs/RUNPOD.md as a checklist instead.

| # | Question | Default taken | Resolved by |
| --- | --- | --- | --- |
| Q1 | Which license should the repo use? | Apache-2.0 (D6) | Owner decision; change `LICENSE` and every `package.xml`/`setup.py`. |
| Q2 | Is the Isaac Sim 6.2.0 NGC image really Ubuntu 24.04? Only the Dockerfile source was checked; `nvcr.io` is unreachable from the build sandbox. | Yes → Jazzy (D1) | `docker/pod_setup.sh` reads `/etc/os-release` on the pod (Phase 3/4). |
| Q3 | Package maintainer name and email. | Resolved in Phase 2: the owner supplied `Akshith Macharla <amacharla@csuchico.edu>` as the repo identity, now used in every `package.xml`/`setup.py` and the C++ copyright lines. | — |
| Q4 | Which LLM model id to use on the free tier? | None set; the agent refuses to start without `NAVSTRESS_LLM_MODEL` (or `--mock`). | Owner runs `--list-models` against their key (RUNPOD.md checklist) and sets it in `.env`. |
| Q5 | Is a 0.45 m clearance radius right for sampling reachable goals on the warehouse map? | 0.45 m | Phase 5: if baseline failures cluster at goals near shelves, resample with a larger radius before interpreting results. |
| Q6 | Which USD asset is a real pallet in the Isaac Sim 6.2 asset library? | Pallet-sized `box` primitive (D10) | Phase 4: list assets on the pod (`/get_spawnables` or the asset browser), then add it to `ALLOWED_ASSETS`. |
| Q7 | What is the Nova Carter's prim path in the 6.2 warehouse scene, and can `SetEntityState` teleport the articulated robot? | `robot_entity` empty: no teleport; episodes start at the reset pose and a scenario with `start` is refused (D19). | Phase 4: `ros2 service call /get_entities ...`, then try one `set_entity_state` on the robot and watch `/chassis/odom` and TF. |
| Q8 | Is the `map` frame aligned with Isaac Sim's world frame in this scene? | `map_to_world: [0, 0, 0]` (identity) in scenario_manager and obstacle_mover | Phase 4: spawn a tall box at a known map pose and check where it appears in `/scan` and the costmap. |
| Q9 | Does Isaac Sim's `SetEntityState` accept kinematic rigid bodies without errors? Its handler calls `set_velocities` on any prim with `PhysicsRigidBodyAPI`. | Moving obstacles are kinematic (D20) | Phase 4: run one crossing scenario and read the Isaac Sim log. Fallback: generate moving obstacles as static colliders too. |
| Q10 | Is the warehouse floor at world z = 0? Obstacles are spawned with their base at z = 0. | Yes | Phase 4: compare a spawned box's top with the 3D lidar returns. |
| Q11 | The collision monitor only sees what the range sensors see, so contact with an obstacle the sensors miss (the low-obstacle case) shows up as `stuck`/`nav2_aborted`, not `collision`. Isaac Sim contact sensors would need a scene change. | Report it as stuck/aborted, and say so when interpreting results | Owner decision in Phase 5: accept, or add a contact sensor and record that Isaac Sim was modified. |
| Q12 | Nav2's local costmap reads `/front_3d_lidar/lidar_points` directly from Isaac Sim, so noise injected on `/scan` alone wouldn't reach it. | Phase 3 launch routes both `/scan` and the 3D cloud through noise injectors by rewriting topic names in a copy of the Carter params | Phase 4 bring-up verifies the rewritten params load and costmaps fill. |
| Q13 | Does RunPod run the Isaac Sim image as root, or as the image's `USER isaac-sim` (uid 1234)? `pod_setup.sh` needs root for apt. | Assume root (RunPod's SSH docs say the pod user is "typically root"); the script refuses clearly otherwise. | Phase 4: `id` on the pod. Fallback in RUNPOD.md: a two-line `FROM ... USER root` image. |
| Q14 | Does `isaac-sim.sh --no-window` render the RTX lidar headless, without the streaming app? | Yes (`--no-window` is what Isaac Sim's own install checks use for headless runs) | Phase 4: `ros2 topic hz /front_3d_lidar/lidar_points` after launch. |
| Q15 | Is a 60 GB container disk enough (ROS and Nav2 install outside the volume)? | 60 GB | Phase 4: `df -h /` after `pod_setup.sh`. |
| Q16 | Does `--enable isaacsim.ros2.sim_control` on the command line load the extension in 6.2? | Yes (standard Kit flag; the extension isn't in the full app's default list) | Phase 4: `ros2 service list` shows `/spawn_entities`. |
| Q17 | Is AMCL's configured initial pose (−6.0, −1.0, π) where the robot sits after `ResetSimulation`? NVIDIA's goal launcher uses (−6.4, −1.04) instead. | AMCL's value from `carter_navigation_params.yaml` (`default_start`) | Phase 4: compare `/amcl_pose` with the ground-truth `get_entity_state` right after a reset. |
