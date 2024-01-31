cmake_minimum_required(VERSION 3.5)
project(rrc_px4_mocap)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)


add_executable(mocap_to_px4 ./src/mocap_to_px4.cpp)
ament_target_dependencies(mocap_to_px4 rclcpp px4_msgs geometry_msgs)
install(TARGETS
        mocap_to_px4
        DESTINATION lib/${PROJECT_NAME})

add_executable(px4_dds_sub ./src/px4_dds_sub.cpp)
ament_target_dependencies(px4_dds_sub rclcpp px4_msgs)
install(TARGETS
        px4_dds_sub
        DESTINATION lib/${PROJECT_NAME})

add_executable(rrc_offboard_test ./src/rrc_offboard_test.cpp)
ament_target_dependencies(rrc_offboard_test rclcpp px4_msgs)
install(TARGETS
        rrc_offboard_test
        DESTINATION lib/${PROJECT_NAME})

add_executable(rrc_flight_mode ./src/rrc_flight_mode.cpp)
ament_target_dependencies(rrc_flight_mode rclcpp px4_msgs std_msgs)
install(TARGETS
        rrc_flight_mode
        DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(ament_cmake rclcpp px4_msgs geometry_msgs sensor_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
