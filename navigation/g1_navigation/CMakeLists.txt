cmake_minimum_required(VERSION 3.8)
project(g1_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(dwb_core REQUIRED)
find_package(dwb_msgs REQUIRED)
find_package(nav_2d_msgs REQUIRED)

add_library(g1_navigation_axis_critics SHARED
  src/translate_or_rotate_critic.cpp
)
target_include_directories(g1_navigation_axis_critics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(g1_navigation_axis_critics
  pluginlib
  dwb_core
  dwb_msgs
  nav_2d_msgs
)

pluginlib_export_plugin_description_file(dwb_core g1_dwb_critics.xml)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY launch g1_description maps rviz config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS g1_navigation_axis_critics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(PROGRAMS
  scripts/cmd_vel_sender_node.py
  scripts/depth_sender_node.py
  scripts/global_costmap.py
  scripts/image_sender_node.py
  scripts/laserscan_sender_node.py
  scripts/laserscan_filter.py
  scripts/nav2_sender_node.py
  scripts/nav_through_poses_bridge.py
  scripts/occupancy_grid_sender_node.py
  scripts/robot_pose_sender_node.py
  scripts/ros_ipc_bridge.py
  scripts/semantic_map_pc_client.py
  scripts/semnav_rviz_node.py
  scripts/trajectory_visualization_node.py
  DESTINATION lib/${PROJECT_NAME}
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

ament_export_include_directories(include)
ament_export_libraries(g1_navigation_axis_critics)
ament_export_dependencies(pluginlib dwb_core dwb_msgs nav_2d_msgs)

ament_package()
