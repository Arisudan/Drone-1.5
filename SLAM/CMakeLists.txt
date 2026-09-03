cmake_minimum_required(VERSION 3.8)
project(rtabmap_drone_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(PROGRAMS
  scripts/wall_boundary_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
