# A validated test scenario. The authoritative schema (allowlists, clamps) lives in
# navstress_scenarios.schema; this message is its ROS transport form.
uint32 SCHEMA_VERSION=1

string scenario_id        # human-readable label, not part of the hash
string family             # scenario family used for grouping, not part of the hash
string config_hash        # sha256 of the canonical JSON of the validated spec

PlanarPose goal
string goal_id            # name of the goal in the reachable goal set ("" if sampled)
bool has_start            # false: keep wherever the simulator reset puts the robot
PlanarPose start

Obstacle[] obstacles
SensorNoise sensor_noise
NavOverride[] nav_overrides

int64 seed
float64 episode_timeout_s
