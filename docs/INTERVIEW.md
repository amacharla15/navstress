# Interview questions

Phase 7 answers about 20 questions from this codebase, citing files. Until then this
is the list of topics SPEC requires, with where each answer will come from.

| Topic | Where the answer lives |
| --- | --- |
| QoS, and why Sensor Data QoS for Isaac Sim sensors | DECISIONS D17; `noise_injector.py`; `collision_monitor_node.cpp` |
| tf2 frames (map, odom, base_link, sensor frames) | `collision_monitor_node.cpp` (`lookup`); `fake_sim.py`; RUNPOD.md TF checks |
| Nav2 costmap layers, and why the failure happened | DESIGN.md "Costmap inputs"; RESULTS.md Phase 5 |
| Actions vs services vs topics | `ApplyScenario.srv`, `RunEpisode.action`, `RunBatch.action`; WALKTHROUGH Phase 0 |
| Why the LLM never drives the robot | DESIGN.md; DECISIONS D14; `schema.py` |
| How scenario hashes make runs reproducible | DECISIONS D8; `schema.py` `canonical_json`; `compare.py` |
| What the fakesim proves and what it cannot | DECISIONS D23; `world.py` docstring |
| Limitations of simulation results | RESULTS.md (Phase 5–6); OPEN_QUESTIONS |
