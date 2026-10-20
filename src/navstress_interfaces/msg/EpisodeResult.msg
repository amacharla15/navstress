# Outcome of one RunEpisode. Written as one row of results/<run_id>/results.csv.
string FAILURE_NONE=""
string FAILURE_TIMEOUT="timeout"
string FAILURE_NAV2_ABORTED="nav2_aborted"
string FAILURE_COLLISION="collision"
string FAILURE_STUCK="stuck"
string FAILURE_SETUP="setup_failed"

string run_id
string scenario_id
string family
string config_hash
int64 seed

bool success
string failure_reason
uint16 nav2_error_code
string nav2_error_msg

float64 time_to_goal_s           # sim seconds from goal acceptance to terminal state
float64 path_length_m            # integrated from odometry
float64 final_distance_to_goal_m
float64 min_clearance_m          # from navstress_collision_monitor; +inf if no data
uint32 collisions                # rising edges of the in-collision flag
uint32 recoveries_triggered      # max number_of_recoveries seen in Nav2 feedback

string bag_path                  # "" when recording was disabled
float64 wall_time_s
