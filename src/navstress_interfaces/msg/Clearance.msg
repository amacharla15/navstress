# Published by navstress_collision_monitor. Distances are to the robot footprint
# polygon in the robot base frame; negative means the point is inside the footprint.
std_msgs/Header header
float64 min_clearance_m
bool in_collision
geometry_msgs/Point closest_point   # in the base frame
float64 robot_speed_mps             # from odometry, to tell driving-into from being-hit
