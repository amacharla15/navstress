cmake_minimum_required(VERSION 3.8)
project(navstress_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch params DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(launch_files test/test_launch_files.py)
  find_package(launch_testing_ament_cmake REQUIRED)
  # The whole pipeline against the fake simulator: agent -> RunBatch -> results -> report.
  add_launch_test(test/test_e2e_fakesim.py TIMEOUT 900)
endif()

ament_package()
