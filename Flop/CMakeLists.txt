cmake_minimum_required(VERSION 3.8)
project(rtabmap_drone_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/map_thinning_node.py
  scripts/wall_boundary_node.py
  scripts/px4_vision_bridge.py
  scripts/obstacle_distance_bridge.py
  scripts/diagnostics/duplex_check.py
  scripts/diagnostics/c2_validate.py
  scripts/diagnostics/bench_check.py
  scripts/diagnostics/slam_health_monitor.py
  scripts/diagnostics/verify_ekf2_params.py
  scripts/diagnostics/live_status.py
  scripts/diagnostics/apply_ekf2_params.py
  scripts/diagnostics/px4_control.py
  scripts/diagnostics/drone_gcs_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
