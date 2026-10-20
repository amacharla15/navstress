# One obstacle spawned for a scenario. Footprint is centred on `pose` (map frame).
string name          # unique within the scenario; becomes part of the sim entity name
string asset         # allowlisted asset key, e.g. "box" or "cylinder"
PlanarPose pose
float64 size_x       # m, extent along the obstacle's local x axis (diameter for cylinders)
float64 size_y       # m, extent along local y (ignored for cylinders)
float64 height       # m, from the floor
# Moving obstacles shuttle between `pose` and (end_x, end_y) at speed_mps.
bool moving
float64 end_x
float64 end_y
float64 speed_mps
