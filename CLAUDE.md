# NavStress — working rules

## What this repo is
NavStress is a ROS 2 project: an LLM agent that stress-tests a Nav2 navigation stack running against NVIDIA Isaac Sim, finds failure modes, and measures a fix. The full build spec is in SPEC.md. Build it phase by phase exactly as SPEC.md describes.

## Hard rules
1. **No AI attribution anywhere.** Never add "Co-Authored-By: Claude", "Generated with Claude Code", Claude session links, or any Claude/Anthropic mention to commit messages, PR descriptions, code comments, docs, or any file. If your tooling adds such a trailer by default, remove it before every commit.
2. **Every number must come from a real run.** Never hand-write, estimate, or "fill in" results. Every figure in README.md or docs/RESULTS.md must trace to a results file under `results/` plus the git commit and scenario-config hash that produced it. If a run fails or is incomplete, report that honestly. Never describe simulation results as real-world or sim-to-real results.
3. **Check current official docs before using any API** (ROS 2 rclpy/rclcpp, tf2, rosbag2, launch, Nav2, simulation_interfaces, Isaac Sim and its `isaacsim.ros2.*` extensions, IsaacSim-ros_workspaces, RunPod). Do not guess function names, topic names, frame names, parameter names, or CLI flags. Isaac Sim renamed many modules across versions — always check the docs for the installed version. If the docs contradict SPEC.md, follow the docs and record the change in `docs/DECISIONS.md`.
4. **Never call a real LLM provider in tests.** Tests use the mock LLM only.
5. **No secrets in the repo.** Commit `.env.example` with placeholders only.
6. **Never handle or ask for the owner's passwords or account logins.** Steps needing the owner's accounts (RunPod, NGC, LLM API keys) go in `docs/RUNPOD.md` as a checklist for the owner.
7. **If blocked** (missing credentials, no GPU in your environment, ambiguous requirement): write the question in `docs/OPEN_QUESTIONS.md`, choose the most reasonable default, note it, and continue with every part that does not depend on the blocker.
8. **Cost discipline on the GPU pod:** keep runs batched, never leave long idle loops running, and at the end of every pod session print a one-line reminder that the pod should be stopped or terminated.

## Code style
- ROS 2 Python nodes: rclpy, type hints, small modules. C++ nodes: rclcpp, C++17, follow ROS 2 style (ament_cmake, ament_lint).
- Use the correct QoS for every subscription (Isaac Sim publishes sensors with Sensor Data QoS — best effort). State the QoS choice in a comment.
- Comments explain *why*, not *what*.
- Lint: ruff for Python, ament_lint / clang-format for C++.
- Minimal dependencies; justify each new one in one line in `docs/DECISIONS.md`.

## After every phase
1. Build (`colcon build`) and tests (`colcon test`) pass wherever ROS 2 is available; otherwise rely on GitHub Actions CI and say so in the phase notes.
2. Update `docs/WALKTHROUGH.md`: for every file added or changed, 3–6 plain-English sentences — what it does, why it exists, and one interview question about it with the answer.
3. Commit with a plain conventional message, e.g. `feat(eval): episode runner with NavigateToPose`. No trailers.
4. Continue to the next phase without waiting, unless the phase is marked "pod only" and you are not on the GPU pod — then stop and say so.
