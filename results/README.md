# results/

Each batch writes `results/<run_id>/`:

- `results.csv` — one row per episode (see `navstress_eval.results_io.RESULT_FIELDS`)
- `run_metadata.json` — git commit, ROS distro, simulator + version, GPU, Nav2 params hash
- `scenarios.jsonl` — the exact validated scenarios, so the set can be rerun
- `bags/` — one MCAP bag per episode (never committed)

Only summaries that docs/RESULTS.md cites are committed, with `git add -f`.
