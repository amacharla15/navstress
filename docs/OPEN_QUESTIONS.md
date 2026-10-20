# Open questions

Questions that block or could change something. Each has the default chosen so work
could continue, and what resolves it. Owner-only items (accounts, credentials) live
in docs/RUNPOD.md as a checklist instead.

| # | Question | Default taken | Resolved by |
| --- | --- | --- | --- |
| Q1 | Which license should the repo use? | Apache-2.0 (D6) | Owner decision; change `LICENSE` and every `package.xml`/`setup.py`. |
| Q2 | Is the Isaac Sim 6.2.0 NGC image really Ubuntu 24.04? Only the Dockerfile source was checked; `nvcr.io` is unreachable from the build sandbox. | Yes → Jazzy (D1) | `docker/pod_setup.sh` reads `/etc/os-release` on the pod (Phase 3/4). |
| Q3 | Package maintainer email. | `amacharla15@users.noreply.github.com` (keeps a personal address out of package metadata) | Owner may replace in `package.xml`/`setup.py`. |
| Q4 | Which LLM model id to use on the free tier? | None set; the agent refuses to start without `NAVSTRESS_LLM_MODEL` (or `--mock`). | Owner runs `--list-models` against their key (RUNPOD.md checklist) and sets it in `.env`. |
| Q5 | Is a 0.45 m clearance radius right for sampling reachable goals on the warehouse map? | 0.45 m | Phase 5: if baseline failures cluster at goals near shelves, resample with a larger radius before interpreting results. |
| Q6 | Which USD asset is a real pallet in the Isaac Sim 6.2 asset library? | Pallet-sized `box` primitive (D10) | Phase 4: list assets on the pod (`/get_spawnables` or the asset browser), then add it to `ALLOWED_ASSETS`. |
