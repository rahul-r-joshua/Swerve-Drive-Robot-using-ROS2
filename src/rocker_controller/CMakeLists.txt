cmake_minimum_required(VERSION 3.8)
project(rocker_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)


add_library(simple_swerve_controller SHARED
  src/simple_swerve_controller.cpp
)

ament_target_dependencies(simple_swerve_controller
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  rclcpp_lifecycle
  pluginlib
  nav2_core
  nav2_costmap_2d
  nav2_util
)
install(TARGETS simple_swerve_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)


install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)


pluginlib_export_plugin_description_file(nav2_core plugin.xml)


install(PROGRAMS
  ${PROJECT_NAME}/controller.py
  ${PROJECT_NAME}/simple_pub_circle.py
  ${PROJECT_NAME}/simple_pub_S.py
  ${PROJECT_NAME}/simple_follow_path.py
  ${PROJECT_NAME}/twist_relay.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_export_include_directories(include)
ament_export_dependencies(rclcpp geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs pluginlib nav2_core nav2_costmap_2d nav2_util)
ament_export_libraries(simple_swerve_controller)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
