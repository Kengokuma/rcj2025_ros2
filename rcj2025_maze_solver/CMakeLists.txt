cmake_minimum_required(VERSION 3.8)
project(rcj2025_maze_solver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(frontier_explorer SHARED
  src/frontier_explorer.cpp
)

ament_target_dependencies(frontier_explorer
  rclcpp
  nav2_msgs
  geometry_msgs
  rclcpp_components
  tf2_ros
  tf2
  tf2_geometry_msgs
)

target_include_directories(frontier_explorer
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

add_library(${PROJECT_NAME}::frontier_explorer ALIAS frontier_explorer)

rclcpp_components_register_node(frontier_explorer
  PLUGIN "rcj2025_maze_solver::FrontierExplorer"
  EXECUTABLE frontier_explorer_node
)

install(TARGETS frontier_explorer
  DESTINATION lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
