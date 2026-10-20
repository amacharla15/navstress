cmake_minimum_required(VERSION 3.8)
project(navstress_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PlanarPose.msg"
  "msg/Obstacle.msg"
  "msg/SensorNoise.msg"
  "msg/NavOverride.msg"
  "msg/ScenarioSpec.msg"
  "msg/EpisodeResult.msg"
  "msg/Clearance.msg"
  "srv/ApplyScenario.srv"
  "action/RunEpisode.action"
  "action/RunBatch.action"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
