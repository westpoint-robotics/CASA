cmake_minimum_required(VERSION 3.8)
project(system_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(utils REQUIRED)
find_package(domain_bridge REQUIRED)

include_directories(include)

add_executable(system src/system.cpp src/agent_tracker.cpp src/system_node.cpp)
add_executable(occupancy_grid src/occupancy_grid.cpp)
add_executable(heartbeat_server src/heartbeat_server.cpp src/heartbeat_server_node.cpp)
add_executable(heartbeat_client src/heartbeat_client.cpp src/heartbeat_client_node.cpp)

ament_target_dependencies(system rclcpp sensor_msgs px4_msgs utils domain_bridge std_msgs)
ament_target_dependencies(occupancy_grid nav_msgs geometry_msgs utils)
ament_target_dependencies(heartbeat_server rclcpp std_msgs)
ament_target_dependencies(heartbeat_client rclcpp std_msgs)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
  system
  occupancy_grid
  heartbeat_server
  heartbeat_client
  DESTINATION lib/${PROJECT_NAME})

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
