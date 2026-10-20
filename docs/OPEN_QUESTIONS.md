# Open questions

Questions that block or could change something. Each has the default chosen so work
could continue, and what resolves it. Owner-only items (accounts, credentials) live
in docs/RUNPOD.md as a checklist instead.

| # | Question | Default taken | Resolved by |
| --- | --- | --- | --- |
| Q1 | Which license should the repo use? | Apache-2.0 (D6) | Owner decision; change `LICENSE` and every `package.xml`/`setup.py`. |
| Q2 | Is the Isaac Sim 6.2.0 NGC image really Ubuntu 24.04? Only the Dockerfile source was checked; `nvcr.io` is unreachable from the build sandbox. | Yes → Jazzy (D1) | `docker/pod_setup.sh` reads `/etc/os-release` on the pod (Phase 3/4). |
| Q3 | Package maintainer email. | `amacharla15@users.noreply.github.com` (keeps a personal address out of package metadata) | Owner may replace in `package.xml`/`setup.py`. |
