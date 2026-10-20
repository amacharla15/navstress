# Running NavStress on a RunPod GPU pod

Phases 4–7 need Isaac Sim, which needs an NVIDIA RTX GPU. This page is the owner's
checklist for renting one on RunPod, setting it up with `docker/pod_setup.sh`, running
headless, and shutting it down so it stops costing money.

**Status:** written before any pod session. The RunPod and Isaac Sim facts below are
from their docs (sources at the end). The pod-specific steps are the plan for Phase 4,
not a record of a session that happened. Items marked *verify* are known unknowns,
also listed in docs/OPEN_QUESTIONS.md.

## Owner checklist (accounts and secrets: only you can do these)

- [ ] RunPod account with billing set up and a low-balance alert
      (RunPod → Billing). If the balance stays at $0, RunPod may terminate the network
      volume and its data can't be recovered.
- [ ] Your SSH public key added in RunPod → Settings → SSH Public Keys.
- [ ] An LLM API key for an OpenAI-compatible endpoint (default: Google Gemini's
      `https://generativelanguage.googleapis.com/v1beta/openai/`), stored as a RunPod
      **secret**, e.g. `gemini_key`. Never paste it into the repo or a chat.
- [ ] Optional, for running Claude Code on the pod: a Claude account with Claude
      Code access, or an Anthropic API key stored as a RunPod secret.
- [ ] Optional: an NGC account. Isaac Sim 6.2's container docs pull the image with a
      plain `docker pull` and no login step, so it should not be needed (*verify*:
      see "If the image pull fails" below).
- [ ] Accept the NVIDIA Isaac Sim license by setting `ACCEPT_EULA=Y` on the pod
      (below). That is your acceptance, not something the scripts do for you.

## 1. Choose the GPU

- **RTX A5000 (24 GB)** or another RTX card with **at least 16 GB VRAM** (RTX 4090,
  RTX A4500, L40S, ...). Isaac Sim renders the lidar with RTX ray tracing, so it needs
  RT cores.
- **Not A100 or H100.** They are compute GPUs with no RT cores, so Isaac Sim's RTX
  rendering can't run on them.
- Host driver: the 6.2.0 image declares `MIN_DRIVER_VERSION=570.169` (its Dockerfile).
  In RunPod's deploy page use **Additional filters → CUDA Versions** and pick 12.8 or
  newer (driver 570 is the CUDA 12.8 driver). A too-old driver shows up as "OCI
  runtime create failed".
- Price: SPEC budgets about $0.27/h for an A5000-class pod. RunPod prices change;
  check the deploy page.

## 2. Create a network volume (persistent storage)

RunPod → Storage → **New Network Volume**: pick a data center that has the GPU you
chose, name it `navstress`, size **60 GB**, Standard tier.

- At $0.07/GB/month (RunPod's standard-tier price) that is about $4.20/month while
  the volume exists. Delete it when the project is done.
- What lives there: `/workspace/navstress/` — the apt `.deb` cache, the two colcon
  workspaces, Isaac Sim's shader/asset caches, results and bags.
- Network volumes only exist in Secure Cloud, and must be attached when the pod is
  deployed. They can't be added later.

## 3. Deploy the pod (a template makes repeats one click)

RunPod → Templates → New Template (or **Set overrides** on deploy):

| Field | Value |
| --- | --- |
| Container image | `nvcr.io/nvidia/isaac-sim:6.2.0` |
| Container start command | `{"entrypoint": ["bash", "-c"], "cmd": ["sleep infinity"]}` |
| Container disk | 60 GB (*verify*: ROS and Nav2 install here, outside the volume) |
| Volume mount path | `/workspace` (with the network volume attached) |
| HTTP / TCP ports | none (headless; nothing is served) |
| Environment variables | see below |

Why the start command: the image's entrypoint is `runheadless.sh`, which starts Isaac
Sim's **WebRTC streaming** app straight away. SPEC wants headless only, with no GUI
streaming, so the pod idles and NavStress starts Isaac Sim itself with `--no-window`.
RunPod's template docs accept this JSON form with `entrypoint` and `cmd`.

Environment variables:

| Variable | Value | Why |
| --- | --- | --- |
| `ACCEPT_EULA` | `Y` | Required by the Isaac Sim container; this is your license acceptance. |
| `PRIVACY_CONSENT` | `Y` | Opts in to NVIDIA's data collection (SPEC lists it). Leave it out to opt out. |
| `NVIDIA_DRIVER_CAPABILITIES` | `all` | Isaac Sim needs the graphics/Vulkan driver capabilities, not just compute. The image sets this too; setting it on the pod guards against overrides. |
| `NAVSTRESS_LLM_API_KEY` | `{{ RUNPOD_SECRET_gemini_key }}` | Your LLM key, from the RunPod secret. |
| `NAVSTRESS_LLM_MODEL` | a model id | Pick one with `--list-models` (section 6). |
| `NAVSTRESS_REPO_REF` | `main` (or the branch to test) | Which NavStress commit `pod_setup.sh` checks out. |

Then **Deploy On-Demand**.

### If the image pull fails

If the pod logs show an authorization error from `nvcr.io`, add registry credentials
in RunPod (Settings → Container Registry Auth): username `$oauthtoken`, password your
NGC API key. Then redeploy.

## 4. Connect

RunPod → Pods → your pod → **Connect** → copy the **SSH** command (basic SSH, proxied
by RunPod, works for every pod):

```bash
ssh <pod-id>-<hash>@ssh.runpod.io -i ~/.ssh/id_ed25519
```

Start `tmux` right away so a dropped connection doesn't kill running batches:

```bash
apt-get update -qq && apt-get install -y -qq tmux   # needs root; skip if it fails
tmux new -s navstress
```

## 5. Set up (first time ~20–30 min; later fresh pods are much faster)

```bash
id    # who am I? see "If the pod is not root" if this isn't uid=0
cd /workspace
git clone --depth 1 -b "${NAVSTRESS_REPO_REF:-main}" https://github.com/amacharla15/navstress.git /tmp/navstress-bootstrap
bash /tmp/navstress-bootstrap/docker/pod_setup.sh --dry-run   # read the plan first
bash /tmp/navstress-bootstrap/docker/pod_setup.sh
```

What it does (each step skips itself if already done):

1. Checks Ubuntu 24.04 (DECISIONS D1), root, the volume, the GPU and `/isaac-sim`.
2. Installs ROS 2 Jazzy with the official `ros2-apt-source` package, then
   `ros-base`, `ros-dev-tools`, `navigation2`, `nav2-bringup`,
   `pointcloud-to-laserscan`, `simulation-interfaces`, `rosbag2-storage-mcap`.
   Downloaded `.deb`s are kept on the volume, so a new pod reinstalls without
   downloading again.
3. Clones IsaacSim-ros_workspaces at tag `IsaacSim-6.2.0` and builds only
   `carter_navigation`, `isaacsim_bringup` and `isaac_ros_navigation_goal`.
4. Clones NavStress into `/workspace/navstress/ws/src/navstress` and builds it.
5. Points Isaac Sim's caches (the same directories NVIDIA's docs mount as volumes) at
   the network volume, so shader and asset caches survive pod restarts.
6. Writes `/workspace/navstress/env.sh` and sources it from `~/.bashrc`.
7. Gives the workspaces to uid 1234 (`isaac-sim`), the user Isaac Sim 6.x runs as.

Re-run it on every new pod; on an existing volume it mostly verifies and relinks.
`ROS_APT_SOURCE_VERSION=<tag>` skips the GitHub API lookup if it is rate-limited.

### If the pod is not root

The Isaac Sim image's Dockerfile ends with `USER isaac-sim` (uid 1234), and it is
unverified whether RunPod keeps that user or runs as root (*verify*, OPEN_QUESTIONS
Q13). `pod_setup.sh` needs root for apt and stops with a clear message otherwise.
Fallback: build a two-line image on any machine with Docker, push it to a registry
RunPod can pull, and use it as the container image:

```dockerfile
FROM nvcr.io/nvidia/isaac-sim:6.2.0
USER root
```

## 6. Run (Phase 4 onward)

Run Isaac Sim and the ROS nodes as the same user, the container's `isaac-sim`:

```bash
su - isaac-sim          # or: runuser -u isaac-sim -- bash -l   (env.sh is sourced)
```

Check the LLM settings once:

```bash
ros2 run navstress_agent stress_agent --list-models   # needs NAVSTRESS_LLM_API_KEY
```

Bring-up (tmux window 1). Isaac Sim headless + Nav2 + NavStress; the first start
compiles shaders and can take several minutes:

```bash
ros2 launch navstress_bringup isaac.launch.py
```

Checks for Phase 4 (tmux window 2):

```bash
ros2 service list | grep -E 'reset_simulation|spawn_entities|set_entity_state'
ros2 topic info -v /front_3d_lidar/lidar_points | grep -i reliability   # expect BEST_EFFORT
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo base_link front_3d_lidar     # lidar mount height for DESIGN.md
ros2 action list | grep -E 'navigate_to_pose|navstress'
```

Goals, baseline, agent, report (Phase 5):

```bash
MAP=$(ros2 pkg prefix carter_navigation)/share/carter_navigation/maps/carter_warehouse_navigation.yaml
ros2 run navstress_scenarios sample_goals --map "$MAP" --start -6.0 -1.0 --n 60 --seed 0 \
  -o "$NAVSTRESS_RESULTS_DIR/goals_warehouse.yaml"
ros2 run navstress_scenarios make_baseline --goals "$NAVSTRESS_RESULTS_DIR/goals_warehouse.yaml" \
  --n 50 --seed 0 -o "$NAVSTRESS_RESULTS_DIR/baseline.jsonl"
ros2 run navstress_eval run_batch --scenarios "$NAVSTRESS_RESULTS_DIR/baseline.jsonl" --run-id baseline
ros2 run navstress_agent stress_agent --goals "$NAVSTRESS_RESULTS_DIR/goals_warehouse.yaml" \
  --start -6.0 -1.0 3.14159 --baseline "$NAVSTRESS_RESULTS_DIR/baseline" --rounds 2 --episodes 40
ros2 run navstress_report report --run "$NAVSTRESS_RESULTS_DIR/baseline" \
  --agent "$NAVSTRESS_RESULTS_DIR/<agent_run_id>" --out-md docs/RESULTS.md --site site
```

Results and bags are on the volume under `/workspace/navstress/results/`. Copy what
you need off the pod with `scp` over full SSH, or `runpodctl send`.

### Running Claude Code on the pod

Install it on the pod's local disk (the Claude Code docs advise against network
storage for the binary):

```bash
bash /workspace/navstress/ws/src/navstress/docker/pod_setup.sh --with-claude-code
# or directly, per code.claude.com/docs/en/setup:
curl -fsSL https://claude.ai/install.sh | bash
```

Run it inside `tmux` from the repo (`cd /workspace/navstress/ws/src/navstress`, then
`claude`). Log in through the printed browser link, or set `ANTHROPIC_API_KEY` from
a RunPod secret. CLAUDE.md applies on the pod too: keep runs batched, and end every
session with the stop reminder.

## 7. Stop or terminate (do this every session)

| Action | What happens | Billing |
| --- | --- | --- |
| **Stop** | GPU released; container disk wiped; network volume kept | Volume storage only |
| **Terminate** | Pod deleted; network volume kept | Volume storage only, no pod disk |
| Delete the network volume | Everything gone | Nothing |

```bash
runpodctl pod stop $RUNPOD_POD_ID         # runpodctl is preinstalled on every pod
# safety net against forgetting: stop automatically after 2 hours
nohup bash -c 'sleep 2h; runpodctl pod stop $RUNPOD_POD_ID' >/dev/null 2>&1 &
```

Or in the web UI: Pods → your pod → Stop (square icon), or Terminate (trash icon).
With a network volume attached there is no reason to keep a stopped pod around:
terminate it. Nothing on the volume is lost, and `pod_setup.sh` rebuilds a fresh pod
from the volume quickly.

## Sources

- RunPod docs (github.com/runpod/docs, read 2026-10-20): `pods/manage-pods.mdx`
  (stop/terminate, `runpodctl`), `storage/network-volumes.mdx` (pricing, Secure Cloud,
  attach at deploy), `pods/templates/manage-templates.mdx` (start command JSON),
  `pods/templates/environment-variables.mdx` and `secrets.mdx` (env vars, secrets),
  `pods/configuration/use-ssh.mdx` (basic SSH).
- Isaac Sim `v6.2.0` source (github.com/isaac-sim/IsaacSim): `tools/docker/Dockerfile`
  (Ubuntu 24.04 base, uid 1234 user, `MIN_DRIVER_VERSION`, entrypoint),
  `tools/docker/README.md` (env vars, cache mounts), `tools/docker/data/runheadless.sh`.
- IsaacSim-ros_workspaces tag `IsaacSim-6.2.0`: `carter_navigation` launch and params,
  `isaacsim_bringup/run_isaacsim.py`.
- ROS 2 Jazzy install guide (github.com/ros2/ros2_documentation, branch `jazzy`,
  `source/Installation/Ubuntu-Install-Debs.rst`).
- Claude Code setup docs (code.claude.com/docs/en/setup).
