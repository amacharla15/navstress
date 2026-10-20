cmake_minimum_required(VERSION 3.8)
project(navstress_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(navstress_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Geometry is a plain library with no ROS types, so it is unit-testable on its own.
add_library(clearance src/clearance.cpp)
target_include_directories(clearance PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

add_library(collision_monitor_node src/collision_monitor_node.cpp)
target_link_libraries(collision_monitor_node PUBLIC
  clearance
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${navstress_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
  rclcpp::rclcpp
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros)

add_executable(collision_monitor src/main.cpp)
target_link_libraries(collision_monitor collision_monitor_node)

install(TARGETS clearance collision_monitor_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS collision_monitor DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # clang-format is the C++ formatter for this repo (see CLAUDE.md); uncrustify
  # would fight it over the same lines, so it is switched off.
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_ros REQUIRED)
  ament_add_gtest(test_clearance test/test_clearance.cpp)
  target_link_libraries(test_clearance clearance)
  # Isolated: gets its own ROS domain so it can't hear other tests' topics.
  ament_add_ros_isolated_gtest(test_collision_monitor_node test/test_collision_monitor_node.cpp
    TIMEOUT 120)
  target_link_libraries(test_collision_monitor_node collision_monitor_node)
endif()

ament_package()
